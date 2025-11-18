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
mechanism = parseURDF("/home/vigno/github/VMRobotControl.jl/URDFs/franka_description/urdfs/fr3.urdf", cfg)
m = compile(mechanism)

# Observable kinematics cache: updates plots automatically
# when changed
kcache = Observable(new_kinematics_cache(m))

# Setup the figure
fig1 = Figure(size=(700, 350))
ls1 = LScene(fig1[1, 1]; show_axis=false) # scene in left parallel
ls2 = LScene(fig1[1, 2]; show_axis=false) # scene in right parallel

# Plot the robot's visuals
robotvisualize!(ls1, kcache)

# Or a sketch of it's kinematic structure (simplified skeleton)
robotsketch!(ls2, kcache; scale=0.3, linewidth=2.5, transparency=true)