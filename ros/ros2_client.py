#! /usr/bin/env python3
# coding: utf-8
import argparse
import errno
import ipaddress
import socket
import struct
import sys, os
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import JointState

####################################################################################################
# ROS 2 Manager:
# i. Subscribes to JointStates messages (robot's current position and velocities)
# ii. Publishes Float64MultiArray messages (torque commands to the robot)
# iii. Defines best Quality of Service for low-latency control
# iv. Threading for managing messages between controller and the robot

class ROSManager(Node):
    def __init__(self, subscriber_topic, publisher_topic, joint_command_size, joint_state_size):
        super().__init__('vmc_control')
        self.joint_command_size = joint_command_size
        self.joint_state_size = joint_state_size
        self.new_msg = None
        self.msg_lock = threading.Lock()
        
        # QoS for real-time control
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(JointState, subscriber_topic, self._callback, qos)
        self.publisher = self.create_publisher(Float64MultiArray, publisher_topic, qos)
        
        # Prepare message template
        self.joint_command_message = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.size = joint_command_size
        dim.stride = 1
        dim.label = "joint_effort"
        self.joint_command_message.layout.dim.append(dim)
        self.joint_command_message.data = [0.0] * joint_command_size

    def _callback(self, msg):
        with self.msg_lock:
            self.new_msg = msg

    def get_new_msg(self):
        with self.msg_lock:
            msg = self.new_msg
            self.new_msg = None
            return msg

####################################################################################################
# IPC Manager:
# i. Creates TCP connection for control commands in state machine (START, WARMUP_DONE, STOP)
# ii. Creates UDP (data channel) for high-frequency data exchange
# iii. Receives torque commands from Julia
# iv. Sends joint states to Julia

STATE_WAITING = 0
STATE_WARMUP = 1
STATE_ACTIVE = 2
STATE_STOPPED = 3

class JointCommand:
    def __init__(self, sequence_number, timestamp, torques):
        self.sequence_number = sequence_number
        self.timestamp = timestamp
        self.torques = torques

class IPCManager:
    def __init__(self, listen_ip, listen_port, joint_command_size, joint_state_size):
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.joint_command_size = joint_command_size
        self.joint_state_size = joint_state_size
        self.torque_fmt = '!QQ' + 'd' * joint_command_size
        self.state_fmt = '!QQ' + 'd' * joint_state_size
        self.sequence_number = 1

    def __enter__(self):
        print("Waiting for connection")
        self.command_socket = self._wait_for_connection()
        (bound_ip, bound_port) = self.command_socket.getsockname()
        
        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.data_socket.bind((bound_ip, bound_port))
        self.data_socket.setblocking(False)
        
        self.send_data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_data_socket.setblocking(False)
        (remote_addr, remote_port) = self.command_socket.getpeername()
        self.send_data_socket.connect((remote_addr, remote_port))
        
        self.command_stream = self.command_socket.makefile('rw')
        return self

    def _wait_for_connection(self):
        address_family = socket.AF_INET if self.listen_ip.version == 4 else socket.AF_INET6
        tcp_server = socket.socket(address_family, socket.SOCK_STREAM)
        tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tcp_server.bind((self.listen_ip.exploded, self.listen_port))
        tcp_server.listen()
        tcp_server.settimeout(5.0)
        
        while rclpy.ok():
            try:
                command_socket, (remote_addr, remote_port) = tcp_server.accept()
                print(f"Connection from {remote_addr}:{remote_port}")
                break
            except socket.timeout:
                print("Timeout waiting for connection, retrying.")
        
        if not rclpy.ok():
            tcp_server.close()
            raise Exception("ROS was shut down.")
            
        command_socket.setblocking(False)   # non-blocking socket to preserve the frequency
        tcp_server.close()
        return command_socket

    def __exit__(self, exc_type, exc_value, traceback):
        print("Closing connection")
        try:
            self.command_stream.write("STOP\n")
            self.command_stream.flush()
        except:
            pass
        time.sleep(0.5)
        if hasattr(self, 'command_stream'): self.command_stream.close()
        if hasattr(self, 'command_socket'): self.command_socket.close()
        if hasattr(self, 'data_socket'): self.data_socket.close()
        if hasattr(self, 'send_data_socket'): self.send_data_socket.close()

    def recv_joint_command(self):
        n_bytes = struct.calcsize(self.torque_fmt)
        try:
            data = self.data_socket.recv(n_bytes)
            data_unpacked = struct.unpack(self.torque_fmt, data)
            return JointCommand(data_unpacked[0], data_unpacked[1], data_unpacked[2:2+self.joint_command_size])
        except socket.error as e:
            if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                raise e
            return None

    def send_robot_state(self, timestamp, joint_state_vector):
        message = struct.pack(self.state_fmt, timestamp, self.sequence_number, *joint_state_vector)
        try:
            (remote_addr, remote_port) = self.command_socket.getpeername()
            self.send_data_socket.sendto(message, (remote_addr, remote_port))
            self.sequence_number += 1
        except:
            pass

####################################################################################################
# Control Loops:
# i. STATE MACHINE: four states:
#           1. WAITING: Julia not ready yet
#           2. WARMUP: Julia initialization with zero torque
#           3. ACTIVE: real control loop
#           4. STOPPED: controller shuts down
# ii. Bridges ROS2 (robot hardware) with Julia at 1000 Hz
# iii. Exchanges torques with joint states

def forward_state_to_julia(socket_manager, ros_manager):
    msg = ros_manager.get_new_msg()
    if msg is None:
        return
    msg_vec = list(msg.position) + list(msg.velocity)
    socket_manager.send_robot_state(time.time_ns(), msg_vec)

def loop_waiting(socket_manager, ros_manager):
    print("State: WAITING")
    while rclpy.ok():
        command = socket_manager.command_stream.readline()
        if command == "START\n":
            return STATE_WARMUP
        elif command == "":
            forward_state_to_julia(socket_manager, ros_manager)
            time.sleep(0.1)
        else:
            print(f"Unexpected command in WAITING: \"{command}\"")
            return STATE_STOPPED

def send_recv_send_recv_wait(socket_manager, ros_manager, rate_sleep, set_zero=False):
    forward_state_to_julia(socket_manager, ros_manager)
    
    command = socket_manager.recv_joint_command()
    if command is not None:
        ros_manager.joint_command_message.data = [0.0] * len(command.torques) if set_zero else list(command.torques)
        ros_manager.publisher.publish(ros_manager.joint_command_message)
    
    time.sleep(rate_sleep)

def loop_warmup(socket_manager, ros_manager, rate_sleep):
    print("State: WARMUP")
    while rclpy.ok():
        command = socket_manager.command_stream.readline()
        if command == "":
            send_recv_send_recv_wait(socket_manager, ros_manager, rate_sleep, set_zero=True)
        elif command == "WARMUP_DONE\n":
            return STATE_ACTIVE
        elif command == "STOP\n":
            return STATE_STOPPED
        else:
            print(f"Unexpected command in WARMUP: \"{command}\"")
            return STATE_STOPPED

def loop_active(socket_manager, ros_manager, rate_sleep):
    print("State: ACTIVE")
    while rclpy.ok():
        command = socket_manager.command_stream.readline()
        if command == "":
            send_recv_send_recv_wait(socket_manager, ros_manager, rate_sleep)
        elif command == "STOP\n":
            return STATE_STOPPED
        else:
            print(f"Unexpected command in ACTIVE: \"{command}\"")
            return STATE_STOPPED

####################################################################################################
# Main

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("joint_command_size", type=int)
    parser.add_argument("joint_commands_topic", type=str)
    parser.add_argument("joint_state_topic", type=str)
    parser.add_argument("--rate", type=float, default=1000)
    parser.add_argument("--joint_state_size", type=int, default=None)
    parser.add_argument("--listen_port", type=int, default=25342)
    parser.add_argument("--listen_ip", type=str, default="127.0.0.1")
    parser.add_argument("--auto-restart", action='store_true')

    args = parser.parse_args()
    joint_state_size = args.joint_state_size if args.joint_state_size else 2 * args.joint_command_size
    listen_ip = ipaddress.ip_address(args.listen_ip)
    rate_sleep = 1.0 / args.rate

    rclpy.init()
    
    ros_manager = ROSManager(
        args.joint_state_topic,
        args.joint_commands_topic,
        args.joint_command_size,
        joint_state_size
    )

    # Use executor in separate thread for callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(ros_manager)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    socket_manager = IPCManager(listen_ip, args.listen_port, args.joint_command_size, joint_state_size)

    try:
        while rclpy.ok():
            with socket_manager:
                state = STATE_WAITING
                while rclpy.ok():
                    try:
                        if state == STATE_WAITING:
                            state = loop_waiting(socket_manager, ros_manager)
                        elif state == STATE_WARMUP:
                            state = loop_warmup(socket_manager, ros_manager, rate_sleep)
                        elif state == STATE_ACTIVE:
                            state = loop_active(socket_manager, ros_manager, rate_sleep)
                        elif state == STATE_STOPPED:
                            break
                        else:
                            raise Exception(f"Invalid state: {state}")
                    except Exception as e:
                        print(f"Unhandled Exception: {e}")
                        import traceback
                        traceback.print_exc()
                        break
            if not args.auto_restart:
                break
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        ros_manager.destroy_node()
        rclpy.shutdown()