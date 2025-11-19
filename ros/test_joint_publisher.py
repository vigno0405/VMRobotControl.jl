#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('test_joint_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.01, self.publish_states)
        self.t = 0.0
        
    def publish_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [0.1 * math.sin(self.t + i) for i in range(21)]
        msg.velocity = [0.0] * 21
        msg.name = [f'joint_{i}' for i in range(21)]
        self.publisher.publish(msg)
        self.t += 0.01

def main():
    rclpy.init()
    node = JointStatePublisher()
    print("Publishing test joint states on /joint_states...")
    rclpy.spin(node)

if __name__ == '__main__':
    main()