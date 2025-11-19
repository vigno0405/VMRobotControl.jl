# We try to define obstacle avoidance.

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

cfg = URDFParserConfig(;suppress_warnings=true)
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseURDF(joinpath(module_path, "/home/vigno/github/VMRobotControl.jl/URDFs/franka_hand_description/panda_adapt_hand.urdf"), cfg)

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
    add_component!(robot, LinearDamper(0.05, "H$i"); id="Hand damper $i")
end

# Add joint limit springs
joint_limits = cfg.joint_limits
for joint_id in keys(joints(robot))
    limits = joint_limits[joint_id]
    isnothing(limits) && continue
    add_coordinate!(robot, JointSubspace(joint_id); id="$(joint_id)_coord")
    @assert ~isnothing(limits.lower) && ~isnothing(limits.upper)
    add_deadzone_springs!(robot, 100.0, (limits.lower+0.1, limits.upper-0.1), "$(joint_id)_coord")
    damping = startswith(string(joint_id), "fr3_") ? 0.01 : 0.002
    add_component!(robot, LinearDamper(damping, "$(joint_id)_coord"); id="$(joint_id)_damper")
end

# Add the TCP coordinate (Index finger tip)
add_coordinate!(robot, FramePoint("Index_Distal_1", SVector(0., 0., 0.)); id="TCP position")

# Build virtual mechanism controller
vms = VirtualMechanismSystem("franka_hand_reaching", robot)

# Define target
vm = vms.virtual_mechanism
add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.4, 0.0, 0.2))); id="target_pos")
add_coordinate!(vms, CoordDifference(".robot.TCP position", ".virtual_mechanism.target_pos"); id="pos error")

# Define spring/damper on the error
add_component!(vms, TanhSpring("pos error"; max_force=10.0, stiffness=1000.0); id="spring")
add_component!(vms, LinearDamper(10.0, "pos error"); id="damper")

# Define obstacle position
obstacle_pos = SVector(0.3, 0.0, 0.2)

# Add obstacle coordinate to virtual mechanism
add_coordinate!(vm, FramePoint("root_frame", obstacle_pos); id="obstacle")

# List frames to protect from collision
collision_frames = String[]
for i in 1:8
    push!(collision_frames, "fr3_link$i")
end

for id in collision_frames
    add_coordinate!(robot, FrameOrigin("$id"); id="$id origin")
    add_coordinate!(vms, CoordDifference(".robot.$id origin", ".virtual_mechanism.obstacle"); id="$id to obstacle")
    add_component!(vms, GaussianSpring("$id to obstacle"; max_force=-10.0, width=0.05); id="repulsive spring $id")
end

# Setup simulation
target_pos(t) = SVector(0.5, 0.0, 0.3) + SVector(0.2*cos(t), 0.0, 0.2*sin(t))
target_vel(t) = SVector(-0.2*sin(t), 0.0, 0.2*cos(t))

function f_setup(cache)
    id = get_compiled_coordID(cache, ".virtual_mechanism.target_pos")
    val = cache[id].coord_data.val
    vel = cache[id].coord_data.vel
    return (val, vel)
end

function f_control(cache, t, args, extra)
    (val, vel) = args
    val[] = target_pos(t)
    vel[] = target_vel(t)
    false
end

tspan = (0, 15)
m_robot = compile(robot)
q_robot = zero_q(m_robot)
q_robot[4] = -1.8
q_robot[6] = pi/2
q̇_robot = zero_q̇(m_robot)

q_vms = zero_q(compile(vms.virtual_mechanism))
q̇_vms = zero_q̇(compile(vms.virtual_mechanism))
q = (q_robot, q_vms)
q̇ = (q̇_robot, q̇_vms)
g = VMRobotControl.DEFAULT_GRAVITY

dcache = new_dynamics_cache(compile(vms))
prob = get_ode_problem(dcache, g, q, q̇, tspan; f_setup, f_control)
@info "Simulating Franka hand with obstacle avoidance."
sol = solve(prob, Tsit5(); maxiters=1e5, abstol=1e-3, reltol=1e-3);

# Plotting
fig = Figure(; size = (2*720, 720), figure_padding=1.5, fontsize=32)
display(fig)
lscenes = (LScene(fig[1, 1]; show_axis=false), LScene(fig[1, 2]; show_axis=false))
cams = map(ls -> cam3d!(ls; center=false), lscenes)
plotting_t = Observable(0.0)

plotting_kcache = Observable(new_kinematics_cache(compile(vms)))

target_scatter_kwargs = (;
    color=:green, 
    marker=:+, 
    markersize=15, 
    label="Targets",
    transparency=true
)
tcp_scatter_kwargs = (;
    color=:blue, 
    marker=:x, 
    markersize=15, 
    label="TCPs",
    transparency=true
)

for ls in lscenes
    robotvisualize!(ls, plotting_kcache; overdraw=false)

    target_pos_id = get_compiled_coordID(plotting_kcache[], ".virtual_mechanism.target_pos")
    tcp_pos_id = get_compiled_coordID(plotting_kcache[], ".robot.TCP position")

    scatter!(ls, plotting_kcache, [target_pos_id]; target_scatter_kwargs...)
    scatter!(ls, plotting_kcache, [tcp_pos_id]; tcp_scatter_kwargs...)

    small_sphere = Sphere(Point3f(obstacle_pos), 0.05)
    large_sphere = Sphere(Point3f(obstacle_pos), 0.1)
    mesh!(ls, small_sphere; color=:magenta, transparency=true)
    mesh!(ls, large_sphere; color=:cyan, alpha=0.1, transparency=true)
end

cams[1].lookat[] = [0., 0., 0.3]
cams[1].eyeposition[] = [1.5, 0., 0.3]
cams[2].lookat[] = [0.4, 0, 0.3]
cams[2].eyeposition[] = [0.4, 1.5, 0.3]

leg = Legend(fig[1, 1], lscenes[1]; merge=true, tellwidth=false, halign=:left, valign=:top)

animate_robot_odesolution(fig, sol, plotting_kcache, "reaching_franka_hand.mp4"; t=plotting_t, f_setup, f_control)