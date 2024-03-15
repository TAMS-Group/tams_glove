import rospkg
import pyassimp
import lxml.etree as et
import tf.transformations
import numpy as np
import geometry_msgs.msg
import visualization_msgs.msg
import typing
import tractor as tr
import tractor.types_double as tt
import mittenwire


class GloveIK:

    def compute_link_states(self, joint_states):
        return self.robot_model.forward_kinematics(joint_states)

    def solve(self, goal_positions, optimize=True):

        if len(goal_positions) != len(self.end_effectors):
            raise Exception("wrong number of goal positions")

        goal_positions = [
            p if isinstance(p, tt.Vector3) else tt.Vector3(p)
            for p in goal_positions
        ]

        floating_joint_indices = [i for i in range(self.robot_model.joint_count) if isinstance(
            self.robot_model.joint(i), tt.FloatingJointModel)]
        if len(floating_joint_indices) != 1:
            raise Exception(
                "model should have exactly one floating joint, currently has " + str(len(floating_joint_indices)))
        floating_joint_state = self.joint_states.joint_state(
            floating_joint_indices[0])

        start_pose = tf.transformations.superimposition_matrix(
            np.transpose(
                [p.value for p in self.default_end_effector_positions]),
            np.transpose([p.value for p in goal_positions]),
            False, True
        )
        start_pos = tf.transformations.translation_from_matrix(start_pose)
        start_rot = tf.transformations.quaternion_from_matrix(start_pose)

        start_pose = tt.Pose(tt.Vector3(start_pos), tt.Orientation(start_rot))
        start_pose_inv = tr.inverse(start_pose)

        for i in range(len(self.end_effectors)):
            self.goal_positions[i].value = (
                start_pose_inv * goal_positions[i]).value

        self.joint_states.deserialize(self.variable_rest_states)

        if optimize:
            with mittenwire.Profiler("ik solve", 0):
                self.ik_solver.solve()
            self.joint_states.update_mimic_joints()

        floating_joint_state.pose.value = (
            start_pose * floating_joint_state.pose).value

        ret = tt.JointStates(self.robot_model)
        ret.deserialize(self.joint_states.serialize())
        return ret

    def find_joint_rest_states(self):
        joint_rest_states = tt.JointStates(self.robot_model)
        for i in range(self.robot_model.joint_count):
            joint_model = self.robot_model.joint(i)
            joint_state = joint_rest_states.joint_state(i)
            if not self.robot_model.is_mimic_joint(i):
                if isinstance(joint_model, tt.ScalarJointModelBase):
                    joint_state.position = (
                        joint_model.limits.upper + joint_model.limits.lower) * tt.Scalar(0.5) * tt.Scalar(0.5)
        joint_rest_states.update_mimic_joints()
        return joint_rest_states

    def find_link_rest_states(self):
        return self.robot_model.forward_kinematics(self.find_joint_rest_states())

    def find_rest_positions(self):
        rest = self.find_link_rest_states()
        return [rest.link_pose(self.end_effectors[i]) * self.retargeting_offsets[i] for i in range(len(self.end_effectors))]

    def free_joint_states(self, joint_states):
        joint_limit_penalty = tt.Scalar(1)
        scalar_joint_count = 0
        floating_joint_count = 0
        scalar_variable_count = 0
        for joint_index in range(self.robot_model.joint_count):
            joint_model = self.robot_model.joint(joint_index)
            joint_state = joint_states.joint_state(joint_index)
            if isinstance(joint_model, tt.ScalarJointModelBase):
                scalar_joint_count += 1
            if not self.robot_model.is_mimic_joint(joint_index):
                if isinstance(joint_model, tt.ScalarJointModelBase):
                    scalar_variable_count += 1
                    tr.variable(joint_state.position)
                    tr.goal(joint_state.position * tt.Scalar(0.005))
                    if joint_model.limits:
                        tr.goal(
                            joint_limit_penalty * tr.relu(joint_model.limits.lower - joint_state.position))
                        tr.goal(joint_limit_penalty * tr.relu(joint_state.position -
                                joint_model.limits.upper))
                if isinstance(joint_model, tt.FloatingJointModel):
                    tr.variable(joint_state.pose)
                    floating_joint_count += 1
        joint_states.update_mimic_joints()

    def __init__(self, glove_model, end_effectors, retargeting_offsets=None, retargeting_weights=None):

        if retargeting_offsets:
            self.retargeting_offsets = [
                tt.Vector3(o) for o in retargeting_offsets]
        else:
            self.retargeting_offsets = [tt.Vector3.zero for e in end_effectors]

        if retargeting_weights:
            self.retargeting_weights = [
                tt.Scalar(w) for w in retargeting_weights]
        else:
            self.retargeting_weights = [tt.Scalar(1) for e in end_effectors]

        self.glove_model = glove_model
        self.end_effectors = end_effectors

        robot_model = tt.RobotModel(glove_model.build_urdf(), "")
        self.robot_model = robot_model

        self.joint_states = tt.JointStates(self.robot_model)

        self.variable_rest_states = self.find_joint_rest_states().serialize()

        self.goal_positions = self.find_rest_positions()

        self.default_end_effector_positions = self.find_rest_positions()

        def f():
            self.free_joint_states(self.joint_states)
            link_states = robot_model.forward_kinematics(self.joint_states)
            for i in range(len(end_effectors)):
                tr.parameter(self.goal_positions[i])
                tr.goal((
                    self.goal_positions[i] - link_states.link_pose(end_effectors[i]) * self.retargeting_offsets[i]) * self.retargeting_weights[i])

        if 0:
            solver = tt.LeastSquaresSolver(tr.DefaultEngine())
            solver.compile(tr.record(f))
            solver.tolerance = 0
            solver.timeout = 10
            solver.max_iterations = 10
            solver.regularization = 0.001
            solver.line_search = False
            self.ik_solver = solver

        if 1:
            solver = tt.SparseLeastSquaresSolver(tr.DefaultEngine())
            solver.compile(tr.record(f))
            solver.tolerance = 0
            solver.timeout = 10
            solver.regularization = 0.001
            solver.step_scaling = 1
            solver.max_iterations = 20

            solver.matrix_builder.multi_threading = False

            solver.linear_solver = tt.SparseLinearLU()

            self.ik_solver = solver
