import numpy as np
from pydrake.all import (
    DiagramBuilder,
    AddMultibodyPlantSceneGraph,
    Parser,
    MultibodyPlant,
    SceneGraph,
    GlobalInverseKinematics,
    RigidTransform,
    RollPitchYaw,
    Solve
)
import matplotlib.pyplot as plt

# Initialize DiagramBuilder
builder = DiagramBuilder()

# Add MultibodyPlant and SceneGraph
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

# Use Parser to load the robot model
parser = Parser(plant)
robot_urdf = "path_to_your_robot_model.urdf"  # Replace with your URDF/SDF path
parser.AddModelFromFile(robot_urdf, model_name="my_robot")

# Finalize the plant
plant.Finalize()

# Build the diagram
diagram = builder.Build()

# Create a GlobalInverseKinematics instance
gik = GlobalInverseKinematics(plant)

# Define the frame you want to control (e.g., end-effector)
end_effector_frame = plant.GetFrameByName("end_effector_frame_name")


desired_position = np.array([0.5, 0.0, 0.5])

# !TODO Add Containts


# Solve the Global IK problem
result = gik.Solve()

if result:
    # Retrieve the solution
    q_sol = result.GetSolution(gik.q())
    print("Inverse Kinematics Solution:")
    print(q_sol)
else:
    print("No solution found for the given IK problem.")