import random
import numpy as np
import sklearn.decomposition
import math
import tractor as tr
import tractor.types_double as tt
import matplotlib.pyplot as plt
from . import ik, mapping, observations, multicam, tactile


class JointFrame:

    def __init__(self, time, joint_states):
        self.time = time
        self.joint_states = joint_states


class JointTrajectory:

    def __init__(self):
        self.frames = []


class MotionSolver:

    def __init__(self, glove_model):
        self.mapping = mapping.KeypointMapping(glove_model)
        self.end_effectors = self.mapping.links
        self.glove_model = glove_model
        self.glove_ik = ik.GloveIK(
            glove_model, self.mapping.links, self.mapping.offsets, self.mapping.weights)
        self.robot_model = self.glove_ik.robot_model

        self.regularization = 1

    def compute_arm_trajectory(self, multicam: multicam.MultiCameraModel, observation_sequence: observations.ObservationSequence):
        joint_trajectory = JointTrajectory()
        joint_states = None
        for frame in observation_sequence.frames:
            keypoint_positions = {}
            for keypoint, obs in frame.keypoint_observation_map.items():
                position = obs.triangulate(multicam)
                keypoint_positions[keypoint] = position
            if len(keypoint_positions) > 0 and all([(q is not None) for q in keypoint_positions.values()]):
                joint_states = self.glove_ik.solve([
                    keypoint_positions[p]
                    for p in self.mapping.keypoints
                ],
                    optimize=True
                )
            if joint_states:
                joint_trajectory.frames.append(
                    JointFrame(frame.time, joint_states))
        return joint_trajectory

    def optimize_joint_trajectory(self,
                                  solve_multicam: multicam.MultiCameraModel,
                                  test_multicam: multicam.MultiCameraModel,
                                  kinematic_joint_trajectory: JointTrajectory,
                                  observation_sequence: observations.ObservationSequence,
                                  tac_interp: tactile.TactileInterpolator = None,
                                  tac_layout: tactile.TactileLayout = None
                                  ):

        optimized_joint_trajectory = JointTrajectory()
        for kinematic_frame in kinematic_joint_trajectory.frames:
            optimized_frame = JointFrame(
                kinematic_frame.time, tt.JointStates(self.glove_ik.robot_model))
            optimized_frame.joint_states.deserialize(
                kinematic_frame.joint_states.serialize())
            optimized_joint_trajectory.frames.append(optimized_frame)

        if tac_interp:
            bend_data = []
            for observation_frame in optimized_joint_trajectory.frames:
                tac = tac_interp.interpolate(observation_frame.time)
                prop_row = tac_layout.serialize_proprioceptive_cells(tac)
                bend_data.append(prop_row)
            bend_data = np.array(bend_data)

            pca = sklearn.decomposition.PCA(8)
            bend_data = pca.fit_transform(bend_data)

            bend_data = bend_data - np.mean(bend_data)
            bend_data = bend_data / np.std(bend_data)
            bend_data = bend_data * 1

            bend_data_0 = bend_data

            scalar_joint_count = len([1 for joint_index in range(self.robot_model.joint_count) if isinstance(
                self.robot_model.joint(joint_index), tt.ScalarJointModelBase) and not self.robot_model.is_mimic_joint(joint_index)])

            bend_joint_mat = tt.Tensor(

                np.random.normal(
                    size=[bend_data.shape[1], scalar_joint_count]) * 0.0001
            )
            bend_data = tt.Tensor(bend_data.astype(np.double))
            joint_bias = tt.Tensor(np.zeros([scalar_joint_count]))

            a = np.array(np.matmul(bend_data_0, bend_joint_mat.value))
            b = np.array(tr.matmul(bend_data, bend_joint_mat).value)
            print(bend_data_0.shape, bend_joint_mat.value.shape, a.shape, b.shape)
            assert (np.allclose(a, b))
            print(a)
            print(b)

        def update_states(test=False):

            joint_limit_penalty = tt.Scalar(1)

            print("bend", np.std(joint_bias.value),
                  np.std(bend_joint_mat.value))

            joint_position_matrix = tr.matmul(
                bend_data, bend_joint_mat)

            joint_position_data = tr.unpack(joint_position_matrix)

            joint_position_biases = tr.unpack(joint_bias)

            for frame_index in range(len(optimized_joint_trajectory.frames)):
                joint_frame = optimized_joint_trajectory.frames[frame_index]
                joint_states = joint_frame.joint_states
                joint_column = 0
                for joint_index in range(self.robot_model.joint_count):
                    if not self.robot_model.is_mimic_joint(joint_index):
                        joint_model = self.robot_model.joint(joint_index)
                        joint_state = joint_states.joint_state(joint_index)
                        if isinstance(joint_model, tt.FloatingJointModel):
                            if not test:
                                tr.variable(joint_state.pose)
                        if isinstance(joint_model, tt.ScalarJointModelBase):
                            i = frame_index * scalar_joint_count + joint_column

                            pos = joint_position_data[i] + \
                                joint_position_biases[joint_column]

                            joint_state.position = pos
                            joint_column += 1

                            if joint_model.limits:
                                tr.goal(
                                    joint_limit_penalty * tr.relu(joint_model.limits.lower - joint_state.position))
                                tr.goal(joint_limit_penalty * tr.relu(joint_state.position -
                                        joint_model.limits.upper))

                joint_states.update_mimic_joints()

        def f(test=False):

            multicam = (test_multicam if test else solve_multicam)

            if not test:
                print("free variables")
                if tac_interp:

                    tr.variable(bend_joint_mat)
                    tr.variable(joint_bias)

                    update_states(test)
                else:
                    for joint_frame in optimized_joint_trajectory.frames:
                        self.glove_ik.free_joint_states(
                            joint_frame.joint_states)

            print("fk")
            link_state_map = {}
            link_state_list = []
            for joint_frame in optimized_joint_trajectory.frames:
                link_states = self.robot_model.forward_kinematics(
                    joint_frame.joint_states)
                link_state_map[joint_frame.time] = link_states
                link_state_list.append(link_states)

            print("observations a")

            camera_names = [cam.name for cam in multicam.cameras]

            projection_frame_map = {}
            for observation_frame in observation_sequence.frames:
                if observation_frame.time in link_state_map:
                    link_states = link_state_map[observation_frame.time]
                    projection_frame = {}
                    for imap in range(len(self.mapping.links)):
                        keypoint = self.mapping.keypoints[imap]
                        end_effector = self.mapping.links[imap]
                        offset = tt.Vector3(self.mapping.offsets[imap])
                        if keypoint in observation_frame.keypoint_observation_map:
                            observations = observation_frame.keypoint_observation_map[keypoint]
                            point = link_states.link_pose(
                                end_effector) * offset
                            camera_projection_map = {}
                            for camera_name, observed_projection in observations.camera_projection_map.items():
                                if camera_name in multicam.camera_map:
                                    camera = multicam.camera_map[camera_name]
                                    estimated_projection = camera.project_point(
                                        point)
                                    camera_projection_map[camera_name] = (
                                        observed_projection, estimated_projection)
                            projection_frame[end_effector] = camera_projection_map
                    projection_frame_map[observation_frame.time] = projection_frame

            print("observations b")

            def lookup_projections(frame_time, link_name, camera_name):
                if frame_time not in projection_frame_map:
                    return None
                projection_frame = projection_frame_map[frame_time]
                if link_name not in projection_frame:
                    return None
                projection_map = projection_frame[link_name]
                if camera_name not in projection_map:
                    return None
                return projection_map[camera_name]

            if test:
                test_err_sum = 0.0
                test_err_div = 0.0

            dimensional_error_sum = 0.0
            dimensional_error_div = 0.0

            for observation_frame in observation_sequence.frames:
                camera_selection = camera_names

                for imap in range(len(self.mapping.links)):

                    link_name = self.mapping.links[imap]
                    weight = self.mapping.weights[imap]

                    evaluate = "tip" in link_name and observation_frame.time > observation_sequence.frames[
                        0].time + 1 and observation_frame.time < observation_sequence.frames[
                        -1].time - 1

                    for camera_name in camera_selection:
                        projections = lookup_projections(
                            observation_frame.time, link_name, camera_name)
                        if projections is not None:
                            observed_projection, estimated_projection = projections
                            for dimension in range(2):
                                estimation_error = (
                                    tt.Scalar(observed_projection[dimension]) - estimated_projection[dimension])
                                err = estimation_error * \
                                    tt.Scalar(0.0002 * weight)
                                tr.goal(err)
                                if test and evaluate:
                                    e = abs(estimation_error.value)
                                    test_err_sum = test_err_sum + e
                                    test_err_div = test_err_div + 1

                    if evaluate:
                        if observation_frame.time in link_state_map:
                            link_states = link_state_map[observation_frame.time]
                            keypoint = self.mapping.keypoints[imap]
                            end_effector = self.mapping.links[imap]
                            offset = tt.Vector3(self.mapping.offsets[imap])
                            if keypoint in observation_frame.keypoint_observation_map:
                                observations = observation_frame.keypoint_observation_map[keypoint]
                                point = link_states.link_pose(
                                    end_effector) * offset
                                err_sum, err_div = test_multicam.compute_dimensional_errors(
                                    observations.camera_projection_map, point)
                                dimensional_error_sum += err_sum
                                dimensional_error_div += err_div

            print("dimensional error",
                  dimensional_error_sum / dimensional_error_div)

            if test:
                return test_err_sum / test_err_div, dimensional_error_sum / dimensional_error_div

            print("link regularizers")

            def pose_velocity(pose_a, pose_b):
                vt = tr.position(pose_b) - tr.position(pose_a)
                vr = tr.residual(tr.orientation(pose_a),
                                 tr.orientation(pose_b))

                vt *= tt.Scalar(10)
                return tt.Twist(vt, vr)

            def link_velocity(frame_index, link_name):
                assert (frame_index - 1 >= 0)
                pose_a = link_state_list[frame_index - 1].link_pose(link_name)
                pose_b = link_state_list[frame_index - 0].link_pose(link_name)
                return pose_velocity(pose_a, pose_b)

            def link_acceleration(frame_index, link_name):
                return link_velocity(frame_index, link_name) - link_velocity(frame_index - 1, link_name)

            def link_jerk(frame_index, link_name):
                return link_acceleration(frame_index, link_name) - link_acceleration(frame_index - 1, link_name)

            for frame_index in range(3, len(optimized_joint_trajectory.frames)):
                for map_index in range(len(self.mapping.links)):
                    link_name = self.mapping.links[map_index]
                    weight = self.mapping.weights[map_index]
                    j = link_jerk(frame_index, link_name)
                    if j is not None:
                        tr.goal(j * tt.Scalar(weight * 0.5 * self.regularization))

            print("joint regularizers")

            def joint_velocity(frame_index, joint_index):
                if frame_index - 1 < 0 or frame_index >= len(optimized_joint_trajectory.frames):
                    return None
                frame_a = optimized_joint_trajectory.frames[frame_index - 1]
                frame_b = optimized_joint_trajectory.frames[frame_index - 0]
                joint_state_a = frame_a.joint_states.joint_state(
                    joint_index)
                joint_state_b = frame_b.joint_states.joint_state(
                    joint_index)
                joint_model = self.robot_model.joint(joint_index)
                if isinstance(joint_model, tt.ScalarJointModelBase):
                    return joint_state_b.position - joint_state_a.position

                return None

            def joint_acceleration(frame_index, joint_index):
                velocity_a = joint_velocity(frame_index - 1, joint_index)
                velocity_b = joint_velocity(frame_index - 0, joint_index)
                if velocity_a is not None:
                    return velocity_b - velocity_a
                return None

            def joint_jerk(frame_index, joint_index):
                acceleration_a = joint_acceleration(
                    frame_index - 1, joint_index)
                acceleration_b = joint_acceleration(
                    frame_index - 0, joint_index)
                if acceleration_a is not None:
                    return acceleration_b - acceleration_a
                return None

            for frame_index in range(len(optimized_joint_trajectory.frames)):
                for joint_index in range(self.robot_model.joint_count):
                    j = joint_jerk(frame_index, joint_index)
                    if j is not None:
                        tr.goal(j * tt.Scalar(self.regularization))

            print("optimizing program")

        print("recording program")
        prog = tr.record(f)

        print("building solver")

        solver = tt.SparseLeastSquaresSolver(tr.DefaultEngine())
        solver.tolerance = 0
        solver.timeout = 10
        solver.regularization = 0.001
        solver.step_scaling = 1
        solver.max_iterations = 0
        solver.test_gradients = 0
        solver.linear_solver = tt.SparseLinearLU()
        solver.compile(prog)
        solver.matrix_builder.multi_threading = True
        print(solver.matrix_builder.complexity)
        print(len(prog.inputs))
        print(len(prog.outputs))
        iterations = 50

        self.test_errors = []

        def eval():
            err = f(test=True)
            self.test_errors.append(err)
            print("err", err)
            pp = []
            for joint_frame in optimized_joint_trajectory.frames:
                link_states = self.robot_model.forward_kinematics(
                    joint_frame.joint_states)
                for link_index in range(len(self.mapping.links)):
                    link_name = self.mapping.links[link_index]
                    pose = link_states.link_pose(link_name)
                    pos_curr = (
                        pose * tt.Vector3(self.mapping.offsets[link_index])).value
                    pp.append(pos_curr)
            tr.visualize_points("kp", 0.001, (1, 1, 1, 1), pp)

        eval()

        print("solving")
        for i in range(iterations):
            solver.gather()
            solver.parameterize()
            print("solve step begin", i)
            step = solver.step()
            print("solve step ready")
            solver.scatter()

            print("step", i, "size", step, "loss",
                  solver.loss)

            if tac_interp:
                update_states()

            if (i + 1) % 10 == 0:
                eval()

        eval()

        print("mimic")
        for joint_frame in optimized_joint_trajectory.frames:
            joint_frame.joint_states.update_mimic_joints()

        print("ready")
        return optimized_joint_trajectory
