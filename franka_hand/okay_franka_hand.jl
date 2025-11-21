# Libraries
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

cfg = URDFParserConfig(suppress_warnings=true, error_on_not_recognized=false) # This is just to hide warnings about unsupported URDF features
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseURDF(joinpath(module_path, "/home/vigno/github/VMRobotControl.jl/URDFs/franka_hand_description/panda_adapt_hand_gazebo.urdf"), cfg)

# Add gravity compensation and damping
add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)
for i in 1:7
    add_coordinate!(robot, JointSubspace("fr3_joint$i"); id="J$i")
    add_component!(robot, LinearDamper(0.1, "J$i"); id="Joint damper $i")
end
hand_joints = [
    "Thumb_CMC1", "Thumb_CMC2", "Thumb_MCP", "Thumb_IP",
    "Index_MCP_Spread", "Index_MCP", "Index_PIP", "Index_DIP",
    "Middle_MCP_Spread", "Middle_MCP", "Middle_PIP", "Middle_DIP",
    "Ring_MCP_Spread", "Ring_MCP", "Ring_PIP", "Ring_DIP",
    "Pinky_MCP_Spread", "Pinky_MCP", "Pinky_PIP", "Pinky_DIP",
    "Wrist_Pitch", "Wrist_Yaw"
]
for (i, joint) in enumerate(hand_joints)
    add_coordinate!(robot, JointSubspace(joint); id="H$i")
    add_component!(robot, LinearDamper(0.05, "H$i"); id="Hand damper $i")  # Lower damping for fingers
end

# Add joint limit springs
joint_limits = cfg.joint_limits
for joint_id in keys(joints(robot))
    limits = joint_limits[joint_id]
    isnothing(limits) && continue
    add_coordinate!(robot, JointSubspace(joint_id); id="$(joint_id)_coord")
    @assert ~isnothing(limits.lower) && ~isnothing(limits.upper)
    add_deadzone_springs!(robot, 100.0, (limits.lower+0.1, limits.upper-0.1), "$(joint_id)_coord")
    
    # Use smaller damping for hand joints
    damping = startswith(string(joint_id), "fr3_") ? 0.01 : 0.002
    add_component!(robot, LinearDamper(damping, "$(joint_id)_coord"); id="$(joint_id)_damper")
end

# Virtual Mechanism Controller

# 1. Reference end-effector position and orientation
target_rot = AxisAngle(SVector(1/sqrt(2), 0., 1/sqrt(2)), Float64(pi))
target_pos = ReferenceCoord(Ref(SVector(0.5, 0.0, 0.3)))

# 2. Build VMS
vms = VirtualMechanismSystem("franka_impedance_control", robot)
vm = vms.virtual_mechanism
root = root_frame(vms.robot)

# 3a. Target position and orientation
add_coordinate!(robot, FramePoint("fr3_link8", SVector(0., 0., 0.));                                id="TCP_position")
add_coordinate!(vm, target_pos,                                                                     id="target_pos")

# 3b. Position and orientation errors
add_coordinate!(vms, CoordDifference(".robot.TCP_position", ".virtual_mechanism.target_pos");       id="pos_error")
add_coordinate!(vms, QuaternionAttitude(".robot.fr3_link8", target_rot);                            id="orient_error")

# 4. Spring/damper on the errors
add_component!(vms, TanhSpring("pos_error"; max_force=50.0, stiffness=20.0);          id="pos_spring")
add_component!(vms, LinearDamper(10.0, "pos_error");                                 id="pos_damper")
add_component!(vms, TanhSpring("orient_error"; max_force=50.0, stiffness=10.0);   id="orient_spring")
add_component!(vms, LinearDamper(0.1, "orient_error");                       id="orient_damper")

# 5. Add the cart for the wrist
# Note: the virtual mechanism has both the robot and the virtual mechanism part, therefore containing "vm"
frame_x = add_frame!(vm, "SlideX")
add_joint!(vm, Prismatic(SVector(1., 0., 0.));
    parent=root_frame(vm), child=frame_x, id="PrismaticX")
add_coordinate!(vm, JointSubspace("PrismaticX");            id="CartXDistance")
add_coordinate!(vm, FrameOrigin(frame_x);                   id="CartXPosition")
add_component!(vm, LinearInerter(1.0, "CartXPosition");     id="CartXInertance")   # virtual mass
add_component!(vm, LinearDamper(100.0, "CartXPosition");    id="CartXDamper")

frame_y = add_frame!(vm, "SlideY")
add_joint!(vm, Prismatic(SVector(0., 1., 0.));
    parent=frame_x, child=frame_y, id="PrismaticY")
add_coordinate!(vm, JointSubspace("PrismaticY");                                id="CartYDistance")
add_coordinate!(vm, FramePoint(frame_y, SVector(0., 0., target_pos.val[][3])),  id="CartYPosition")
add_component!(vm, LinearInerter(1.0, "CartYPosition");                         id="CartYInertance")   # virtual mass
add_component!(vm, LinearDamper(100.0, "CartYPosition");                        id="CartYDamper")

add_coordinate!(robot, FrameOrigin("Middle_Base_1"); id="palm_origin")                    # to the robot
add_coordinate!(vms, CoordDifference(".robot.palm_origin", ".virtual_mechanism.CartYPosition"), id="palm_to_cart")

add_component!(vms, TanhSpring("palm_to_cart"; max_force=5.0, stiffness=1.0);      id="palm_spring")
add_component!(vms, LinearDamper(1.0, "palm_to_cart");                         id="palm_damper")

add_coordinate!(robot, FramePoint("Thumb_Distal_1", SVector(0., 0., 0.)), id="finger1")
add_coordinate!(robot, FramePoint("Index_Distal_1", SVector(0., 0., 0.)), id="finger2")
add_coordinate!(vms, CoordDifference(".robot.finger1", ".robot.finger2"), id="between_fingers")
add_component!(vms, TanhSpring("between_fingers"; max_force=1.0, stiffness=10.0); id="fingers_spring")

# Add springs to keep middle, ring, and pinky finger joints near zero
finger_groups = [
    ("Middle", 9:12),
    ("Ring", 13:16),
    ("Pinky", 17:20)
]

for (finger_name, joint_indices) in finger_groups
    for idx in joint_indices
        joint_name = hand_joints[idx]
        coord_id = ".robot.H$idx"
        add_component!(vms, TanhSpring(coord_id; max_force=0.5, stiffness=5.0); 
                      id="$(joint_name)_zero_spring")
        add_component!(vms, LinearDamper(0.05, coord_id); 
                      id="$(joint_name)_zero_damper")
    end
end

# Simulation setup
tspan = (0., 15.)
m_robot = compile(robot)
q_robot = zero_q(m_robot)
q_robot[4] = -1.8
q_robot[6] = pi/2
q̇_robot = zero_q̇(m_robot)

# Initialize the virtual mechanism and its coordinates
q_vms = zero_q(compile(vms.virtual_mechanism))
q̇_vms = zero_q̇(compile(vms.virtual_mechanism))
q = (q_robot, q_vms)
q̇ = (q̇_robot, q̇_vms)
g = VMRobotControl.DEFAULT_GRAVITY

# Create dynamics cache and ODE problem
dcache = new_dynamics_cache(compile(vms))
prob = get_ode_problem(dcache, g, q, q̇, tspan)

# Solve
@info "Starting simulation. Hope it works..."
sol = solve(prob, Tsit5(); maxiters=1e5, abstol=1e-6, reltol=1e-6)

# Visualization
fig = Figure(size = (1440, 1440), figure_padding=0)
display(fig)
ls = LScene(fig[1, 1]; show_axis=false)
cam = cam3d!(ls, camera=:perspective, center=false)
cam.lookat[] = [0.24, -0.02, 0.37]          # point where the camera is looking
cam.eyeposition[] = [0.8, 1.2, 0.8]         # position of the camera

plotting_kcache = Observable(new_kinematics_cache(compile(robot)))
robotvisualize!(ls, plotting_kcache)

# Visualize target position
target_pos = Point3f(target_pos.val[]...)
meshscatter!(ls, [target_pos]; markersize=0.01, color=:green, transparency=true)

# Animate
animate_robot_odesolution(fig, sol, plotting_kcache, "okay_franka_hand.mp4")