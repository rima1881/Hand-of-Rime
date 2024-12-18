
import numpy as np
from pydrake.all import MultibodyPlant, RotationMatrix, Parser, Solve, RigidTransform
from pydrake.multibody.inverse_kinematics import InverseKinematics
import pinocchio as pin
import pink
import math
from VetRobotEmulatorCommon import DRAKE_ENGINE, PINK_ENGINE

HOME_JOINTS = np.array ( [ 0.0000000, -13.6120005, 12.7543001, 0.0000000, 30.8575993, 0.0000000 ] )

MECADEMIC_URDF_LAST_JOINT_NAME = "meca_axis_6_link"
MECADEMIC_URDF_BASE_JOINT_NAME = "meca_base_link"

DRAKE_SIM_TIME_STEP = 0.01


# The origin of the URDF-based robot model includes an additional X-axis offset of 12.498mm compared to the DH-based robot model.
URDF_WORLD_OFFSET_X = 12.498

# In the URDF-based robot model the J6 frame is rotated 90 degrees around the Y axis.
URDF_J6_ROTATION = np.array([
        [math.cos(math.pi / 2.0),    0.0,   math.sin(math.pi / 2.0),  0.0],
        [0.0,                        1.0,   0.0,                      0.0],
        [-math.sin(math.pi / 2.0),   0.0,   math.cos(math.pi / 2.0),  0.0],
        [0.0,                        0.0,   0.0,                      1.0]
])


URDF_J6_ROTATION_INV = np.linalg.inv(URDF_J6_ROTATION)


def rotateAroundZ (transform: np.ndarray, rotationRadians: float) -> np.ndarray:

    rotationOnlyTransform = np.copy ( transform )
    rotationOnlyTransform [ :3, 3 ] = 0.0

    appliedRotationTransform = np.array ( [
        [ math.cos ( rotationRadians ), -math.sin ( rotationRadians ), 0.0, 0.0 ],
        [ math.sin ( rotationRadians ),  math.cos ( rotationRadians ), 0.0, 0.0 ],
        [                          0.0,                           0.0, 1.0, 0.0 ],
        [                          0.0,                           0.0, 0.0, 1.0 ] ] )

    assert appliedRotationTransform is not None

    newTransform = np.matmul ( rotationOnlyTransform,  np.linalg.inv ( appliedRotationTransform ) )
    newTransform [ :3, 3 ] = transform [ :3, 3 ]

    return newTransform


def convertToURFDWorld (inputTransform):
    output = np.matmul(inputTransform, URDF_J6_ROTATION_INV)
    output[0, 3] += URDF_WORLD_OFFSET_X
    output[:3, 3] /= 1000.0
    return output


def convertFromURDFWorld(inputTransform):
    inputCopy = np.copy(inputTransform)
    # convert of meter to millimeter
    inputCopy[:3, 3] *= 1000.0

    # change based on world offset
    inputCopy[0, 3] -= URDF_WORLD_OFFSET_X

    # rotate in brainsight coordinates
    inputCopy = np.matmul(inputCopy, URDF_J6_ROTATION)

    return inputCopy


class KinematicEngine:

    """
        Basic forward kinematics:
            Takes joint angles in degrees and returns the position transform in millimeters.
    """
    def forwardKinematics(self, jointsInDegrees: np.ndarray) -> np.ndarray:
        pass


    """
        The most basic implementation for inverse kinematics, aimed at receiving a target transform
        and returning corresponding robot joint values. While inverse kinematics often has multiple 
        solutions for a given transform, two current engines provide only a single solution. Achieving 
        multiple joint solutions requires global inverse kinematics.

        Drake offers a global inverse kinematics module, but it relies on commercial solvers to compute solutions
        https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_global_inverse_kinematics.html
        https://drake.mit.edu/doxygen_cxx/group__solvers.html
    """
    def inverseKinematics(self, flangeTransform, positionError, orientationError) -> (np.ndarray, bool):
        pass


    """
        Most tools in VetRobot are needle-like, so maintaining a fixed orientation for the tool is unnecessary. 
        Instead, a constraint based on the angle between vectors is more appropriate. However, the current engines
        rely on standard orientation constraints and simply rotate the tool at different angles. this is due to the
        fact that adding tool to robot model is needed before changing constraints.
    """
    def inverseKinematicsIgnoreTwist(self,
                                     flangeTransform,
                                     toolCalibration,
                                     positionError = 1e-6,
                                     orientationError = 1e-6) -> (np.ndarray, bool):

        toolTransform = np.matmul(flangeTransform, toolCalibration)
        tentativeTwistAngles = np.radians(np.arange(0, 360, 30))

        for angle in tentativeTwistAngles:

            rotatedToolTransform = rotateAroundZ(toolTransform, angle)
            rotatedFlangeTransform = np.matmul(rotatedToolTransform, np.linalg.inv(toolCalibration))

            solutionJointsDegrees, didInverseKinematicsConverge = self.inverseKinematics(rotatedFlangeTransform,
                                                                                         positionError,
                                                                                         orientationError)

            if didInverseKinematicsConverge:
                return solutionJointsDegrees, didInverseKinematicsConverge

        return None, False


    """
        In the alignment step, BrainSight prepares the VetRobot for injection. This function tests various adjustments
        and thoroughly checks the joint angles to ensure they don't change excessively. During the injection,
        it's crucial that the robot avoids erratic movements, as they could harm the subject.

        Currently, to prevent such movements, a 5-degree difference in angles is imposed to ensure the movements
        remain safe.
    """
    # TODO Take depth out
    def inverseKinematicForInjection(self,
                                     flangeTransform,
                                     toolCalibration,
                                     positionError = 1e-6,
                                     orientationError = 1e-6):

        injectionStep = lambda t: [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -t],
            [0, 0, 0, 1]
        ]

        depth = 20

        newSolutionJointsDegrees, tLimit = self.inverseKinematicForPath(
            flangeTransform,
            toolCalibration,
            injectionStep,
            depth,
            positionError,
            orientationError
        )

        return newSolutionJointsDegrees, tLimit >= depth


    def inverseKinematicForPath(self,
                                flangeTransform,
                                toolCalibration,
                                stepMotion,
                                stepNum,
                                positionError = 1e-6,
                                orientationError = 1e-6):

        toolTransform = np.matmul(flangeTransform, toolCalibration)
        tentativeTwistAngles = np.radians(np.arange(0, 360, 30))

        maxT = 0
        bestJoints = None

        for angle in tentativeTwistAngles:
            rotatedToolTransform = rotateAroundZ(toolTransform, angle)

            rotatedFlangeTransform = np.matmul(rotatedToolTransform, np.linalg.inv(toolCalibration))

            solutionJointsDegrees, didInverseKinematicsConverge = self.inverseKinematics(rotatedFlangeTransform,
                                                                                         positionError,
                                                                                         orientationError)

            if not didInverseKinematicsConverge:
                continue

            prevSolutionJointsDegrees = np.copy(solutionJointsDegrees)

            t = 0
            while True:
                pointToolTransform = np.matmul(rotatedToolTransform, stepMotion(t))
                pointFlangeTransform = np.matmul(pointToolTransform, np.linalg.inv(toolCalibration))

                newSolutionJointsDegrees, didInverseKinematicsConverge = self.inverseKinematics(pointFlangeTransform,
                                                                                                positionError,
                                                                                                orientationError)


                if not didInverseKinematicsConverge:
                    break

                diffAnglesDegrees = np.abs( prevSolutionJointsDegrees - newSolutionJointsDegrees )

                if np.any(diffAnglesDegrees > 5):
                    break

                if t >= stepNum:
                    return solutionJointsDegrees, t

                prevSolutionJointsDegrees = newSolutionJointsDegrees
                t += 1

            maxT = t
            bestJoints = np.copy(solutionJointsDegrees)

        return bestJoints, maxT


    def linearJointsKinematic(self, joints, toolCalibration, stepMotion, t):

        baseFlangeTransform = self.forwardKinematics(joints)
        baseToolTransform = np.matmul(baseFlangeTransform, toolCalibration)

        destinationToolTransform = np.matmul( baseToolTransform, stepMotion(t) )
        destinationFlangeTransform = np.matmul( destinationToolTransform, np.linalg.inv(toolCalibration) )

        sol, suc = self.inverseKinematics(destinationFlangeTransform)
        if suc:
            return sol
        return None


# https://github.com/stephane-caron/pink
class KinematicsEnginePink(KinematicEngine):

    def __init__(self, urdfPath, lastJoint, damping, randomTimes, dt, homeJointsDegrees):
        self.randomTimes = randomTimes
        self.dt = dt
        self.damping = damping
        self.homeJointsRad = np.deg2rad(homeJointsDegrees)

        self.model = pin.buildModelFromUrdf(urdfPath)
        self.data = self.model.createData()
        self.lastJoint = lastJoint


    def forwardKinematics(self, jointsInDegrees: np.ndarray) -> np.ndarray:
        pin.forwardKinematics(self.model, self.data, np.radians(jointsInDegrees))
        pin.updateFramePlacements(self.model, self.data)
        goal = self.data.oMf[self.model.getFrameId(self.lastJoint)]

        endEffectorTransformMillimeter = convertFromURDFWorld(goal.np)

        return endEffectorTransformMillimeter


    """
        Pink Inverse Kinematics uses an initial position to determine the final solution.
        Its normal success rate is 54%, but with sufficient random initial positions, reachability increases to 100%.
        
        success rate: 99%
        problems:
            - Pink's default kinematic function has a very low success rate, which was addressed with a workaround. 
            However, this approach significantly increases the time required to find a solution.
            
        TODOs:
            - Exploring methods to tune variables for improving algorithm speed.
    """
    def inverseKinematics(self, flangeTransform, positionError = 1e-6, orientationError = 1e-6) -> (np.ndarray, bool):

        FlangeTransformInPink = convertToURFDWorld(flangeTransform)
        flangeTransformGoal = pin.SE3(FlangeTransformInPink)

        joints, hasSolution = self.inverseKinematicsAttempt(flangeTransformGoal,
                                                            self.homeJointsRad,
                                                            positionError,
                                                            orientationError)

        if hasSolution:
            return np.rad2deg(joints), True

        upperLimits = self.model.upperPositionLimit
        lowerLimits = self.model.lowerPositionLimit


        # Limiting J1 Joint
        lowerLimits[0] = -np.pi / 4
        upperLimits[0] = np.pi / 4

        for i in range(self.randomTimes):
            randomInitialGuess = np.random.uniform(low = lowerLimits, high = upperLimits)
            joints, hasSolution = self.inverseKinematicsAttempt(flangeTransformGoal,
                                                                randomInitialGuess,
                                                                positionError,
                                                                orientationError)
            if hasSolution:
                return np.rad2deg(joints), True

        return None, False


    # TODO
    # Error has to be fixed
    # Tolerance has to be fixed
    def inverseKinematicsAttempt(self,
                                 FlangeTransformGoal,
                                 initialGuess,
                                 positionError,
                                 orientationError) -> (np.ndarray, bool):

        tolerance = 1e-6

        task = pink.tasks.FrameTask(
            frame = self.lastJoint,
            position_cost = 1.0,
            orientation_cost = 0.75,
        )

        task.set_target(FlangeTransformGoal)

        configuration = pink.configuration.Configuration(self.model, self.data, initialGuess)

        # TODO: Understand the (0, 30) range and dt step.
        for _ in np.arange(0, 30, self.dt):
            velocity = pink.solve_ik(configuration, (task,), self.dt, solver = 'quadprog', damping = self.damping)
            configuration.integrate_inplace(velocity, self.dt)

            tcp_pose = configuration.get_transform_frame_to_world(self.lastJoint)
            error = pin.log(tcp_pose.actInv(FlangeTransformGoal)).vector
            if np.linalg.norm(error[:3]) < tolerance and np.linalg.norm(error[3:]) < tolerance:
                return configuration.q, True

        return None, False


# https://drake.mit.edu/
class KinematicsEngineDrake(KinematicEngine):

    def __init__(self, urdfPath, baseJointName, lastJointName, timeStep):
        drakePlant = MultibodyPlant(timeStep)

        robotUrdfAsString = open(urdfPath, "r").read()
        robotUrdfAsString.replace("\n", "")

        Parser(drakePlant).AddModelsFromString(robotUrdfAsString, "urdf")

        # To prevent kinematic solutions where the robot floats in space,
        # drake requires pinning down the robot base to a fixed position.
        robot_base = drakePlant.GetFrameByName(baseJointName)
        drakePlant.WeldFrames(
            drakePlant.world_frame(),
            robot_base,
            RigidTransform.Identity()
        )

        drakePlant.Finalize()

        drakePlantContext = drakePlant.CreateDefaultContext()

        self.plant = drakePlant
        self.context = drakePlantContext
        self.lastJoint = lastJointName

    def forwardKinematics(self, jointsInDegrees: np.ndarray):

        self.plant.SetPositions(self.context, np.radians(jointsInDegrees))

        endEffectorTransform = self.plant.CalcRelativeTransform(
            self.context,
            self.plant.world_frame(),
            self.plant.GetFrameByName(self.lastJoint)
        ).GetAsMatrix4()

        endEffectorTransformMillimeter = convertFromURDFWorld(endEffectorTransform)

        return endEffectorTransformMillimeter


    """
        Success rate : 94%
        Problems : 
            - The current implementation of Drake lacks the necessary precision, particularly when joint constraints 
            are applied for the injection.
            
            - Adding Tool in drake caused problems for forward kinematics due to the difference in the drake world and
            VetRobot world 
        TODOs :
            - Develop a method to constrain the algorithm for greater precision.
            
            - If adding tool in Drake is implemented, the fixed orientation constraint can be replaced with 
            AddAngleBetweenVectorsConstraint, potentially reducing errors.
    """
    def inverseKinematics(self,
                          flangeTransform,
                          positionError = 1e-6,
                          orientationError = 1e-6):


        flangeTransform = convertToURFDWorld(flangeTransform)

        solutionJointsDegrees, didInverseKinematicsConverge = self.inverseKinematicsAttempt(
            flangeTransform,
            positionError,
            orientationError,
        )

        # If inverse kinematics fails the first time, we try to solve a simpler problem, without the orientation constraints.
        # Then we use this intermediate solution as the starting point (guess) for a second attempt.
        # Tests show that this approach helps reduce the number of cases where inverse kinematics fails.
        if not didInverseKinematicsConverge:

            # Even though the inverse kinematics without orientation may fail,
            # it might still succeed when using the failed result as the initial guess.
            initialGuess, _ = self.inverseKinematicsAttempt(
                flangeTransform,
                positionError,
                orientationError,
                addOrientationConstraint = False
            )

            solutionJointsDegrees, didInverseKinematicsConverge = self.inverseKinematicsAttempt(
                flangeTransform,
                positionError,
                orientationError,
                initialGuess = initialGuess
            )

        return solutionJointsDegrees, didInverseKinematicsConverge


    def inverseKinematicsAttempt(self,
                                 flangeTransform: np.ndarray,
                                 positionError,
                                 orientationError,
                                 addOrientationConstraint = True,
                                 initialGuess = None
                                 ):

        destinationTranslation = np.array(flangeTransform[:3, 3])

        translationLowerBound = destinationTranslation - positionError
        translationUpperBound = destinationTranslation + positionError

        desired_orientation = RotationMatrix(flangeTransform[:3, :3])

        drakeInverseKinematics = InverseKinematics(self.plant, self.context)

        # We might attempt to solve the IK without the orientation constraint
        # to obtain an initial guess for the solution.
        if addOrientationConstraint:
            drakeInverseKinematics.AddOrientationConstraint(
                self.plant.world_frame(),
                desired_orientation,
                self.plant.GetFrameByName(self.lastJoint),
                RotationMatrix(np.eye(3)),
                orientationError
            )

        drakeInverseKinematics.AddPositionConstraint(
            self.plant.GetFrameByName(self.lastJoint),
            [0.0, 0.0, 0.0],
            self.plant.world_frame(),
            translationLowerBound,
            translationUpperBound
        )

        drakeInverseKinematicsProgram = drakeInverseKinematics.prog()
        drakeInverseKinematicsVariable = drakeInverseKinematics.q()

        # Include the joint guesses if we have them available.
        if initialGuess is not None:
            drakeInverseKinematicsProgram.SetInitialGuess(drakeInverseKinematicsVariable, np.radians(initialGuess))

        result = Solve(drakeInverseKinematicsProgram)

        didInverseKinematicsConverge = result.is_success()

        solutionJointsRadians = result.GetSolution(drakeInverseKinematicsVariable)
        solutionJointsDegrees = np.degrees(solutionJointsRadians)

        return solutionJointsDegrees, didInverseKinematicsConverge


def createEngine( engineType, urdfPath ):

    if engineType == DRAKE_ENGINE:
        return KinematicsEngineDrake(urdfPath, MECADEMIC_URDF_BASE_JOINT_NAME, MECADEMIC_URDF_LAST_JOINT_NAME, DRAKE_SIM_TIME_STEP)

    if engineType == PINK_ENGINE:
        return KinematicsEnginePink(urdfPath, MECADEMIC_URDF_LAST_JOINT_NAME, 1e-12, 10, 1, HOME_JOINTS )
