###################################################
## By Omar Faris
## This code is a working example of an impedance controller using ROS2
## The code is self-contained in Julia and uses PythonCall.jl to communicate with ROS2
## The code subscribes to the topic /NS_1/franka/joint_states to get the Franka joint states
## The code publishes the desired external torques to the topic /NS_1/julia_torque_controller/external_torques
## Publisher topic is defined in a custom franka_ros2 example controlle
## Use franka joint states to initialize the Julia VMC, then run the loop to calculate the torques
## Publish your desired target end-effector position from ROS to the topic /target_from_ros
###################################################
## I will add more details about how it works and make it more readable later
###################################################
## Always run ros2 launch franka_bringup example.launch.py controller_name:=julia_torque_controller
## The custom julia_torque_controller executes published torques from Julia
## If the topic is empty, it executes zero torques (gravity compensation only)
###################################################

# --- Julia Imports ---
using PythonCall
using StaticArrays
using LinearAlgebra
using VMRobotControl
using VMRobotControl: robot_ndof
using Base.Threads
using FileIO, UUIDs
using MeshIO 

# --- ROS 2 Python Modules ---
const rclpy = pyimport("rclpy")
const Node = pyimport("rclpy.node").Node

# --- ROS 2 Message Types ---
const JointState = pyimport("sensor_msgs.msg").JointState
const Float64MultiArray = pyimport("std_msgs.msg").Float64MultiArray

const DESIRED_JOINT_ORDER = [
        "fr3_joint1",
        "fr3_joint2",
        "fr3_joint3",
        "fr3_joint4",
        "fr3_joint5",
        "fr3_joint6",
        "fr3_joint7"
    ]

# The Channel is typed to our custom struct
state_channel = Channel{Vector{Float64}}(10)
target_channel = Channel{Vector{Float64}}(1)

# # --- VMRobotControl Setup ---

try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch
end
cfg = URDFParserConfig(;suppress_warnings=true) # This is just to hide warnings about unsupported URDF features
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseURDF(joinpath(module_path, "URDFs/franka_description/urdfs/fr3.urdf"), cfg)

add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)
for i in 1:7
    add_coordinate!(robot, JointSubspace("fr3_joint$i"); id="J$i")
    add_component!(robot, LinearDamper(2.0, "J$i"); id="Joint damper $i")
end

vms = VirtualMechanismSystem("franka_impedance_control", robot)
vm = vms.virtual_mechanism
add_coordinate!(robot, FramePoint("fr3_link8", SVector(0.0, 0.0, 0.0)); id="TCP position")
root = root_frame(vms.robot)

# We will now update this "Target position" from a ROS topic
add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.5, 0.0, 0.4))); id="Target position")
add_coordinate!(vms, CoordDifference(".robot.TCP position", ".virtual_mechanism.Target position"); id="Position error")

K = SMatrix{3,3}(0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001)
add_component!(vms, TanhSpring("Position error"; max_force=7.0, stiffness=250.0); id="Linear Spring")
D = SMatrix{3,3}(5.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 5.0)
add_component!(vms, LinearDamper(D, "Position error"); id="Linear Damper")

# --- Control Logic Functions ---
# These functions now read from/write to the shared control_args dictionary

function f_setup(cache)
    # This function populates the shared dictionary with initial values
    target_position = get_compiled_coordID(cache, ".virtual_mechanism.Target position")
    return target_position
end

function f_control(cache, t, args, dt)
    target_position = args
    if isready(target_channel)
        new_target = take!(target_channel)
        cache[target_position].coord_data.val[] = SVector(new_target[1], new_target[2], new_target[3])
    end
end


function ros_vm_controller(
        vms,
        qᵛ; 
        gravity=VMRobotControl.DEFAULT_GRAVITY,
        f_setup=VMRobotControl.DEFAULT_F_SETUP,
        f_control=VMRobotControl.DEFAULT_F_CONTROL,
        E_max=30.0,
    )

    rclpy.init()
    node = nothing

    try
        node = Node("my_julia_subscriber_async")

        torque_topic = "/NS_1/julia_torque_controller/external_torques"
        torque_publisher = node.create_publisher(
            Float64MultiArray,
            torque_topic,
            10  # QoS profile depth
        )
        println("Publisher created for topic: $torque_topic")
        
        py_callback_wrapper = (pymsg) -> joint_state_callback(pymsg, state_channel)

        sub = node.create_subscription(
            JointState,
            "NS_1/franka/joint_states",
            py_callback_wrapper,
            10
        )

        target_callback_wrapper = (target_msg) -> target_callback(target_msg, target_channel)

        sub_target = node.create_subscription(
            Float64MultiArray,
            "/target_from_ros",
            target_callback_wrapper,
            10
        )

        println("Starting ROS 2 spin in a background task...")
        ros_task = @async rclpy.spin(node)

        println("Main Julia loop is running. Waiting for data...")

        control_cache = new_control_cache(vms, qᵛ, gravity)
    
        let # Set initial joint state in cache
            # Create cache
            NDOF = robot_ndof(control_cache)
            # Get the initial state for virtual robot as well
            latest_state_array = take!(state_channel)
            qʳ = latest_state_array[1:NDOF]
            q̇ʳ = latest_state_array[NDOF+1:2*NDOF]
            q̇ʳ = zeros(eltype(control_cache), NDOF)
            control_cache = new_control_cache(vms, qᵛ, gravity)
            control_step!(control_cache, 0.0, qʳ, q̇ʳ) # Step at t=0 to set initial state
            @info "Initial joint state set in control cache."
        end

        args = f_setup(control_cache) # Call user setup function

        # Create control callback
        control_func! = let control_cache=control_cache, args=args
            function control_func!(t, dt)
                NDOF = robot_ndof(control_cache)
                # @show NDOF, desired_states
                #@assert length(state) == 2*NDOF
                latest_state_array = take!(state_channel)
                qʳ = latest_state_array[1:NDOF]
                q̇ʳ = latest_state_array[NDOF+1:2*NDOF]
                # Main control step
                f_control(control_cache, t, args, dt) # Call user control function
                
                desired_torques = control_step!(control_cache, t, qʳ, q̇ʳ) # Get desired_torques
                pymsg = Float64MultiArray()
                pymsg.data = pylist(desired_torques)
                torque_publisher.publish(pymsg)
                #print("Published torques at time $t\n")
                return false
            end
        end

        (E = stored_energy(control_cache)) > E_max && error("Initial stored energy exceeds $(E_max)J, was $(E)J")

        t = 0.0
        dt = 0.001 # Control loop time step
        print("Starting main control loop...\n")
        # --- YOUR MAIN CONTROLLER LOOP (Consumer) ---
        while !istaskdone(ros_task)
            control_func!(t, dt)
            t = t + dt
        end
    catch e
        if e isa InterruptException
            println("\nCaught Interrupt. Shutting down...")
        else
            rethrow(e)
        end
    finally
        # Cleanup
        println("Destroying node and shutting down rclpy.")
        if !isnothing(node)
            node.destroy_node()
        end
        rclpy.shutdown()
    end
end


# --- 3. The Callback (Producer) ---
# KEEPS IT SIMPLE! Just convert and send.
function joint_state_callback(pymsg, state_channel::Channel{Vector{Float64}})
    
    local names::Vector{String}
    local positions::Vector{Float64}
    local velocities::Vector{Float64}
    
    # 1. Convert ALL required data from Python types
    names = pyconvert(Vector{String}, pymsg.name)
    positions = pyconvert(Vector{Float64}, pymsg.position)
    velocities = pyconvert(Vector{Float64}, pymsg.velocity)
    
    try
        # 2. Create lookup maps (Dictionaries)
        #    This maps each joint name to its current position and velocity
        pos_map = Dict(zip(names, positions))
        vel_map = Dict(zip(names, velocities))

        # 3. Pre-allocate the 14-element array
        state_array = Vector{Float64}(undef, 14)

        # 4. Fill the array in the correct order
        for (i, joint_name) in enumerate(DESIRED_JOINT_ORDER)
            # Check if the joint exists in our map (prevents KeyErrors)
            if !haskey(pos_map, joint_name) || !haskey(vel_map, joint_name)
                @warn "Joint '$joint_name' not found in received message. Skipping."
                return
            end

            # Assign position
            state_array[i] = pos_map[joint_name]
            
            # Assign velocity (offset by 7)
            state_array[i + 7] = vel_map[joint_name]
        end
        put!(state_channel, state_array)

    catch e
        println("Error in callback while creating array:")
        showerror(stdout, e)
    end
end


function target_callback(pymsg, target_channel::Channel{Vector{Float64}})
    
    local target::Vector{Float64}

    target = pyconvert(Vector{Float64}, pymsg.data)
    
    try
        # 3. Pre-allocate the 14-element array
        target_array = Vector{Float64}(undef, 3)
        target_array .= target
        put!(target_channel, target_array)

    catch e
        println("Error in callback while creating array:")
        showerror(stdout, e)
    end
end

cvms = compile(vms)
qᵛ = Float64[]
ros_vm_controller(cvms, qᵛ; f_control, f_setup, E_max=30.0)