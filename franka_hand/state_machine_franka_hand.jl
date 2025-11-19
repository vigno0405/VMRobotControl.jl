# We try to shift the target of the robot with a state machine, simply according to accuracy.

using DifferentialEquations
using GeometryBasics: Vec3f, Point3f
using GLMakie
using LinearAlgebra
using MeshIO
using StaticArrays
using VMRobotControl

using FileIO, UUIDs
try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch
end
cfg = URDFParserConfig(;suppress_warnings=true) # This is just to hide warnings about unsupported URDF features
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseURDF(joinpath(module_path, "/home/vigno/github/VMRobotControl.jl/URDFs/franka_hand_description/panda_adapt_hand.urdf"), cfg)

# ## Loading/Building the Robot and Controller

# First we have to load the model of the robot, which uses the URDF file format.
# Because the meshes for the franka robot are in the DAE file format, which is not natively
# supported by Julia's MeshIO/FileIO, we have to manually register the DAE file format to be able to load

# Now, we begin adding gravity compensation to the robot model, as the franka robot does its own
# gravity compensation. We also add some damping to each joint to make the simulation more realistic
# so that the robot does not oscillate indefinitely.
add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)
for i in 1:7    # add damping
    add_coordinate!(robot, JointSubspace("fr3_joint$i");    id="J$i")
    add_component!(robot, LinearDamper(0.1, "J$i");         id="Joint damper $i")
end;

# Building the Virtual Mechanism System. It will control the position of the end effector of the robot.
# Remember that it can be used for underactuated systems as well.

vms = VirtualMechanismSystem("franka_state_machine", robot) # VMS
add_coordinate!(robot, FramePoint("fr3_link8", SVector(0., 0., 0.));            id="TCP position")  # defines coordinate tracking the tool center point (TCP). Position is [0, 0, 0] in fr3_link8 frame

# Targets
const TARGETS = [
    SVector(0.4, 0.0, 0.2),   # Target 1
    SVector(0.4, 0.0, 0.3),   # Target 2  
    SVector(0.4, 0.0, 0.5),   # Target 3
]

# Mutable target
target_ref = Ref(TARGETS[1])                        # Ref allows changing this value later
add_coordinate!(vms, ReferenceCoord(target_ref); id="active_target")    # current target, which can change
add_coordinate!(vms, CoordDifference(".robot.TCP position", "active_target"); id="position_error")  # virtual difference

# Single spring/damper on the error
K = SMatrix{3, 3}(50., 0., 0., 0., 20., 0., 0., 0., 30.)
D = SMatrix{3, 3}(50., 0., 0., 0., 20.0, 0., 0., 0., 30.)
add_component!(vms, LinearSpring(K, "position_error"); id="spring")
add_component!(vms, LinearDamper(D, "position_error"); id="damper")

# State machine structure
mutable struct StateMachine # mutable because fields can change
    current_state::Int  # 1, 2, or 3
    targets::Vector{SVector{3, Float64}}    # all the targets
    target_ref::Base.RefValue{SVector{3, Float64}}  # reference to the current target
    tcp_coord_id::Any  # Will be set in f_setup: it is the compiled TCP coordinate ID
    threshold::Float64  # Distance threshold in meters
end

# Setup function - called once before simulation
function f_setup(cache)

    # Get TCP coordinate ID
    tcp_coord_id = get_compiled_coordID(cache, ".robot.TCP position")
    
    # Initialize state machine
    sm = StateMachine(1, TARGETS, target_ref, tcp_coord_id, 0.001)  # 0.001m = 0.1cm threshold
    
    return sm
end

# Control function - called every timestep
function f_control(cache, t, sm::StateMachine, extra)
    # Get current TCP position
    tcp_pos = configuration(cache, sm.tcp_coord_id)
    
    # Get current target
    current_target = sm.targets[sm.current_state]
    
    # Calculate distance to current target
    distance = norm(tcp_pos - current_target)
    
    # Check if we're close enough to switch to next target
    if distance < sm.threshold
        # Move to next state (loop back to 1 after last state)
        next_state = mod1(sm.current_state + 1, length(sm.targets))
        
        # Only switch if we're moving to a different state
        if next_state != sm.current_state
            sm.current_state = next_state
            sm.target_ref[] = sm.targets[sm.current_state]
            
            println("t=$(round(t, digits=2))s: Reached target! Distance: $(round(distance*100, digits=1))cm")
            println("  Switching to target $(sm.current_state): $(sm.targets[sm.current_state])")
        end
    end
    
    nothing
end

# Simulation setup
tspan = (0., 15.)
q = ([0.0, 0.3, 0.0, -1.8, 0.0, π/2, 0.0, zeros(22)...], Float64[])
q̇ = ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, zeros(22)...], Float64[])
g = VMRobotControl.DEFAULT_GRAVITY

# Create dynamics cache and ODE problem
dcache = new_dynamics_cache(compile(vms))
prob = get_ode_problem(dcache, g, q, q̇, tspan; f_setup, f_control)

# Solve
@info "Starting simulation with condition-based state switching"
sol = solve(prob, Tsit5(); maxiters=1e5, abstol=1e-6, reltol=1e-6)

# Visualization
fig = Figure(size = (1500, 1500), figure_padding=0)
display(fig)
ls = LScene(fig[1, 1]; show_axis=true)
cam = cam3d!(ls, camera=:perspective, center=false)
cam.lookat[] = [0.24, -0.02, 0.37]
cam.eyeposition[] = [0.54, 0.76, 0.53]

plotting_t = Observable(0.0)
plotting_kcache = Observable(new_kinematics_cache(compile(robot)))
robotvisualize!(ls, plotting_kcache)

# Visualize target positions as spheres
for (i, target) in enumerate(TARGETS)
    meshscatter!(ls, [Point3f(target)]; 
                 markersize=0.005, 
                 color=:green, 
                 transparency=true)
end

# Animate
animate_robot_odesolution(fig, sol, plotting_kcache, "state_machine_franka.mp4")


