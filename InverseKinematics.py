import math
from pydrake.all import MultibodyPlant, RotationMatrix, Parser, Solve, RigidTransform, MosekSolver
from pydrake.multibody.inverse_kinematics import InverseKinematics

import time
import numpy as np

np.set_printoptions(suppress=True, precision=8)

URDF_WORLD_OFFSET_X = 12.498

URDF_J6_ROTATION = np.array([
        [math.cos(math.pi / 2.0),    0.0,   math.sin(math.pi / 2.0),  0.0],
        [0.0,                        1.0,   0.0,                      0.0],
        [-math.sin(math.pi / 2.0),   0.0,   math.cos(math.pi / 2.0),  0.0],
        [0.0,                        0.0,   0.0,                      1.0]
])


URDF_J6_ROTATION_INV = np.linalg.inv(URDF_J6_ROTATION)


TestNum = 10000.0

HOME_MATRIX = np.array([
    [-0.499998, 0.0,    0.866026,   148.2679],
    [0.0,       1.0,    0.0,        0.0],
    [-0.866026, 0.0,    -0.499998,  261.000229],
    [0.0,       0.0,    0.0,        1.0]
])


HOME_JOINTS = np.deg2rad([0.0, -13.612000465, 12.754300117, 0.0, 30.857599258, 0.0])

RANDOM_POS = np.array([3.04676705, 0.29143209, -0.02131716, -1.97426958, -1.99391916,  0.61141654])

SEC_HOME = np.array([-5.26315790e-12, -2.41441875e-01, 2.30523678e-01, 0, 5.24514380e-01,  0])


SAMPLE_JOINT_VALUE1 = np.radians(
    [8.083023071, 48.846698761,  -0.171329379,   124.528343201,   -45.0,  -37.538223267]
)


def init_drake(urdf_path):

    #   Drake initialization
    sim_time_step = 0.01
    plant = MultibodyPlant(sim_time_step)

    #   Read URDF File
    f = open(urdf_path, "r")
    robot_urdf_string = f.read()
    robot_urdf_string.replace("\n", "")

    #   Add Robot model to plant
    Parser(plant).AddModelsFromString(robot_urdf_string, "urdf")

    #   Pining robot base
    robot_base = plant.GetFrameByName("meca_base_link")
    plant.WeldFrames(
        plant.world_frame(),
        robot_base,
        RigidTransform.Identity()
    )

    plant.Finalize()

    plant_context = plant.CreateDefaultContext()

    return plant, plant_context


def ik(
        plant,
        plant_context,
        destination_flange_position_transform,
        add_orientation_constraint=True,
        guess=None,
        orientation_error=1e-12,
        position_error=1e-12):

    destination_flange_transform = np.matmul(destination_flange_position_transform, URDF_J6_ROTATION_INV)
    destination_flange_transform[0, 3] += URDF_WORLD_OFFSET_X
    destination_flange_transform[:3, 3] /= 1000.0

    p_aq = np.array(destination_flange_transform[:3, 3])

    lower_bound = p_aq - position_error
    upper_bound = p_aq + position_error
    desired_orientation = RotationMatrix(destination_flange_transform[:3, :3])

    inverse_kinematics = InverseKinematics(plant, plant_context)

    prog = inverse_kinematics.prog()

    ik_joints = inverse_kinematics.q()

    # Sometimes
    if add_orientation_constraint:
        inverse_kinematics.AddOrientationConstraint(
            plant.world_frame(),
            desired_orientation,
            plant.GetFrameByName("meca_axis_6_link"),
            RotationMatrix(np.eye(3)),
            1e-3
        )

    inverse_kinematics.AddPositionConstraint(
        plant.GetFrameByName("meca_axis_6_link"),
        [0, 0, 0],
        plant.world_frame(),
        lower_bound,
        upper_bound
    )

    if guess is not None:
        prog.SetInitialGuess(inverse_kinematics.q(), guess)



    result = Solve(prog)

    new_q = result.GetSolution(ik_joints)

    return result.is_success(), new_q


def ik_general(plant, plant_context, target_matrix):

    is_reachable, q = ik(plant, plant_context, target_matrix)

    # try again with guessing the results
    if not is_reachable:
        # Even though the ik without orientation can fail sometimes, it still might successed with the initial guess of
        # failed result
        _, initial_guess = ik(plant, plant_context, target_matrix, add_orientation_constraint=False)
        is_reachable, q = ik(plant, plant_context, target_matrix, guess=initial_guess)

    return is_reachable, q


def forward_kinematics(plant, plant_context, joints):

    plant.SetPositions(plant_context, joints)

    end_effector_transform = plant.CalcRelativeTransform(
        plant_context,
        plant.world_frame(),
        plant.GetFrameByName('meca_axis_6_link')
    ).GetAsMatrix4()

    # convert of meter to millimeter
    end_effector_transform[:3, 3] *= 1000.0

    # change based on world offset
    end_effector_transform[0, 3] -= URDF_WORLD_OFFSET_X

    # rotate in brainsight coordinates
    end_effector_transform = np.matmul(end_effector_transform, URDF_J6_ROTATION)

    return end_effector_transform


def generate_joints(plant, plant_context):

    upper_limits = plant.GetPositionUpperLimits()
    lower_limits = plant.GetPositionLowerLimits()

    upper_limits[0] = math.pi/3
    lower_limits[0] = -math.pi/3

    valid_positions = []
    while len(valid_positions) < TestNum:
        random_position = np.random.uniform(low=lower_limits, high=upper_limits)
        plant.SetPositions(plant_context, random_position)
        valid_positions.append(random_position)
    return valid_positions


def perform_test(plant, plant_context):

    targets = generate_joints(plant, plant_context)
    failed_targets = []
    success_num = TestNum
    delay_arr = []

    for t in targets:

        false_positive = 0

        target_matrix = forward_kinematics(plant, plant_context, t)

        t1 = time.time()
        is_reachable, q = ik(plant, plant_context, target_matrix)
        t2 = time.time()

    # try again with guessing the results
        if not is_reachable:

            # Even though the ik without orientation can fail sometimes,
            # it still might successed with the initial guess of failed result
            _, initial_guess = ik(plant, plant_context, target_matrix, add_orientation_constraint=False)
            is_reachable, q = ik(plant, plant_context, target_matrix, guess=initial_guess)

            t2 = time.time()

        delay_arr.append(t2-t1)

        # failed both times
        if not is_reachable:
            failed_targets.append(t)
            success_num -= 1
            continue

        if np.any(np.abs( target_matrix - forward_kinematics(plant, plant_context, q) > 1e-12)):
            false_positive += 1

    print("result success rate: ", (success_num / TestNum) * 100)

    avg_delay = np.mean(delay_arr)
    std_delay = np.std(delay_arr)

    print("average delay: ", avg_delay)
    print("standard deviation of delay: ", std_delay)
    print("false positives", false_positive)


robot, robot_context = init_drake("robot.v3.model.urdf")


# perform_test(robot, robot_context)

test_matrix = forward_kinematics(robot, robot_context, HOME_JOINTS)

success, test_joints = ik_general(robot, robot_context, test_matrix)

test_matrix_v2 = forward_kinematics(robot, robot_context, test_joints)
print("diff Home and test 1:\n", HOME_MATRIX - test_matrix)
print("diff test 1 and 2:\n", test_matrix - test_matrix_v2)
print("diff Home and 2\n", HOME_MATRIX - test_matrix_v2)
