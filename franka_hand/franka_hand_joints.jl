using DifferentialEquations
using GLMakie
using VMRobotControl
using StaticArrays

using FileIO  # handling I/O
using UUIDs   # for 3D meshes
    try FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch e
end

cfg = URDFParserConfig(suppress_warnings=true) # suppress parsing warnings
mechanism = parseURDF("/home/vigno/github/VMRobotControl.jl/URDFs/franka_hand_description/panda_adapt_hand.urdf", cfg)
m = compile(mechanism)

# Observable kinematics cache: updates plots automatically
# when changed
kcache = Observable(new_kinematics_cache(m))

t = 0.0
q = Float64[0.0, 0.0, 0.0, -pi/2, pi/2, pi/2, 0.0, zeros(22)...]
kinematics!(kcache[], t, q)
notify(kcache)

# Setup the figure
fig = Figure(size=(1500, 1500))
ls = LScene(fig[1, 1]; show_axis=false)
robotvisualize!(ls, kcache)

# Get internal IDs
all_frames = frames(m)
all_frame_ids = [get_compiled_frameID(m, name) for name in all_frames]

# Visualize all frames
scatter!(ls, kcache, all_frame_ids; color=:red, markersize=50)
text!(ls, kcache, all_frame_ids; text=all_frames, color=:blue)

display(fig)