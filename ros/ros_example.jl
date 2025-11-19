# Load ROS communication code
include("ROS.jl")

# Load robot model
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseURDF(joinpath(module_path, "URDFs/sciurus17_description/urdf/sciurus17.urdf"))

# Include gravity compensation done by robot software (not done by our controller) - so we do not take it into account
add_gravity_compensation!(robot, DEFAULT_GRAVITY) 

# Build our virtual mechanism controller
vms = VirtualMechanismSystem("sciurus", robot)
add_coordinate!(vms, ReferenceCoord(Ref(SVector(.3, 0.1, 0.2))); id="l_tgt")	# left one
add_coordinate!(vms, FramePoint(".robot.l_link7", SVector(0., 0., 0.)); id="l_hand")
add_coordinate!(vms, CoordDifference("l_tgt", "l_hand"), id="l_err")
add_component!(vms, TanhSpring("l_err"; stiffness=200.0, max_force=5.0); id="l_spring")
add_component!(vms, LinearDamper(5., "l_err"); id="l_damper")

add_coordinate!(vms, ReferenceCoord(Ref(SVector(.3, 0.1, 0.2))); id="r_tgt")	# right one
add_coordinate!(vms, FramePoint(".robot.r_link7", SVector(0., 0., 0.)); id="r_hand")
add_coordinate!(vms, CoordDifference("r_tgt", "r_hand"), id="r_err")
add_component!(vms, TanhSpring("r_err"; stiffness=200.0, max_force=5.0); id="r_spring")
add_component!(vms, LinearDamper(5., "r_err"); id="r_damper")

function f_setup(cache)
    l_ref_coord_id = get_compiled_coordID(cache, "l_tgt")
    r_ref_coord_id = get_compiled_coordID(cache, "r_tgt")
    return (l_ref_coord_id, r_ref_coord_id)
end

function f_control(cache, t, setup_ret, extra)
    l_ref_coord_id, r_ref_coord_id = setup_ret
    coord = cache[l_ref_coord_id].coord_data.val[] = SVector(0.3, .1 + 0.1*sin(t), 0.2)
    coord = cache[r_ref_coord_id].coord_data.val[] = SVector(0.3 + 0.1*cos(t), -.1, 0.2 + 0.1*sin(t))
    nothing 
end

# Compile the virtual mechanism system, and run the controller via ROS
# Make sure rospy_client.py is running first.
cvms = compile(vms)
qᵛ = Float64[]
with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT, 21, 42) do connection
    ros_vm_controller(connection, cvms, qᵛ; f_control, f_setup, E_max=20.0)
end
