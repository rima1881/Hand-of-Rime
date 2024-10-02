
from pydrake.all import DiagramBuilder
from pydrake.all import AddMultibodyPlantSceneGraph
from pydrake.all import MultibodyPlant
from pydrake.all import RotationMatrix
from pydrake.all import Parser
from pydrake.all import Simulator
from pydrake.all import StartMeshcat
from pydrake.all import AddDefaultVisualization
from pydrake.all import Solve
from pydrake.all import RigidTransform
import numpy as np

from pydrake.multibody.inverse_kinematics import InverseKinematics

"""

urdf_path = "robot.v3.model.urdf"

f = open(urdf_path, "r")
robot_urdf_string = f.read()
robot_urdf_string.replace("\n","")

# Visualize the cylinder from the SDFormat string you just defined.
visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.parser().AddModelsFromString(robot_urdf_string, "urdf")

test_mode = True if "TEST_SRCDIR" in os.environ else False


# Click the "Stop Running" button in MeshCat when you're finished.
visualizer.Run(loop_once=test_mode)


"""

#   -----------------------------------------------------
#   Loading robot and building the world

meshcat = StartMeshcat()


sim_time_step = 0.01

builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(
    builder, time_step=sim_time_step
)
parser = Parser(plant)

urdf_path = "robot.v3.model.urdf"

f = open(urdf_path, "r")
robot_urdf_string = f.read()
robot_urdf_string.replace("\n","")
Parser(plant).AddModelsFromString(robot_urdf_string , "urdf")

robot_base = plant.GetFrameByName("meca_base_link")
plant.WeldFrames(
    plant.world_frame(),
    robot_base,
    RigidTransform.Identity()
)

plant.Finalize()

AddDefaultVisualization(builder=builder, meshcat=meshcat)

diagram = builder.Build()
#   -------------------------------------------------------------
#   Creating Simulation
#   We won't run it yet

simulator = Simulator(diagram)
simulator_context = simulator.get_context()

plant_context = plant.GetMyMutableContextFromRoot(simulator_context)

#   ------------------------------------------------------------
#   Inverse Kinematic
IK = InverseKinematics(plant , plant_context)

prog = IK.prog()
q = IK.q()

p_AQ_lower = np.array([-50, 0, -50])
p_AQ_upper = np.array([50,   0.01, 50])


desired_orientation = RotationMatrix(np.eye(3))  # Target orientation as an identity rotation (no rotation)

print(desired_orientation)

IK.AddOrientationConstraint(
    plant.world_frame(),
    desired_orientation,
    plant.GetFrameByName("meca_axis_6_link"),
    desired_orientation,
    0.01
)


IK.AddPositionConstraint(
    plant.GetFrameByName("meca_axis_6_link"),
    [0, 0, 0],
    plant.world_frame(),
    p_AQ_lower,
    p_AQ_upper
)

result = Solve(prog)

new_q = result.GetSolution(q)

print(new_q)

if result.is_success():
    print("yay")
else:
    print("ahhhh")


def generateRandomJointValue( plant):

    upperLimits = plant.GetPositionUpperLimits()
    lowerLimits = plant.GetPositionLowerLimits()

    random_q = np.random.uniform(low=lowerLimits, high=upperLimits)
    print(np.degrees(random_q))
    return random_q
random_pos = generateRandomJointValue(plant)

#plant.SetPositions( plant_context , np.array([ -5.26315790e-12 , -2.41441875e-01  , 2.30523678e-01 , 0 , 5.24514380e-01 ,  0]))
#plant.SetPositions( plant_context , np.array([ 3.04676705 , 0.29143209 , -0.02131716 , -1.97426958 , -1.99391916 ,  0.61141654]))
#plant.SetPositions( plant_context , np.deg2rad([0.0, -13.612000465, 12.754300117, 0.0, 30.857599258, 0.0]))

end_effector_transform = plant.CalcRelativeTransform(
        plant_context,
        plant.world_frame(),
        plant.GetFrameByName("meca_axis_6_link")
    )

# Step 4: Print the result
print("End-effector position and orientation:")
print(end_effector_transform)


#   ------------------------------------------------------------
#   Simulation Section

simulator.Initialize()
simulator.set_target_realtime_rate(1.0)

meshcat.StartRecording()



while True:
    # Adjust joint values dynamically if needed
    # plant.SetPositions(plant_context, new_joint_values)

    # Advance the simulation in small time steps
    simulator.AdvanceTo(simulator.get_context().get_time() + 0.01)

#   ------------------------------------------------------------
#   move plant

