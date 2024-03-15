from . import tracking, utils, tactile, glovemodel, observations, paths
import mittenwire
import yaml
import numpy as np
import tractor as tr
import tractor.types_double as tt


class GloveFusion:

    motion_solver: tracking.MotionSolver
    joint_trajectory: tracking.JointTrajectory
    tac_layout: tactile.TactileLayout
    tac_renderer: tactile.TactileRenderer
    bag_path: str
    data_path: str
    out_path: str
    observation_sequence: observations.ObservationSequence

    def __init__(self, glove_model: glovemodel.GloveModel):
        self.tac_layout = tactile.TactileLayout()
        self.tac_renderer = tactile.TactileRenderer()
        self.motion_solver = tracking.MotionSolver(glove_model)

    def load_data(self, bag_path):
        self.bag_path = bag_path
        self.data_path = paths.extpath(bag_path, ".detect.yaml")
        self.out_path = paths.extpath(bag_path, ".solve.yaml")

        self.observation_sequence = observations.ObservationSequence.load(
            self.data_path)

        print("trim trajectory")
        while self.observation_sequence.frames[0].is_empty():
            print("trim start")
            self.observation_sequence.frames = self.observation_sequence.frames[1:]
        while self.observation_sequence.frames[-1].is_empty():
            print("trim end")
            self.observation_sequence.frames = self.observation_sequence.frames[:-1]

        self.observation_sequence.frames = self.observation_sequence.frames[3:]

        print("frames", len(self.observation_sequence.frames))

        self.observation_sequence.fill_gaps()

        print("frames", len(self.observation_sequence.frames))

        self.tac_seq = tactile.TactileSequence.load(bag_path).process()
        self.tac_interp = tactile.TactileInterpolator(
            self.tac_seq, self.observation_sequence.frames[0].time)

    def solve_trajectory(self, multicam_solve, multicam_test, use_tactile):
        print("triangulate joint trajectory")
        self.joint_trajectory = self.motion_solver.compute_arm_trajectory(
            multicam_solve,
            self.observation_sequence)

        print("optimize joint trajectory")
        if 1:
            self.joint_trajectory = self.motion_solver.optimize_joint_trajectory(
                multicam_solve,
                multicam_test,
                self.joint_trajectory,
                self.observation_sequence,
                *([self.tac_interp, self.tac_layout] if use_tactile else [])
            )

        print("finished")

        self.test_errors = self.motion_solver.test_errors

    def export_trajectory(self):
        sequence_data = []
        for joint_frame in self.joint_trajectory.frames:

            link_states = self.motion_solver.glove_ik.compute_link_states(
                joint_frame.joint_states)

            keypoint_positions = []
            for link_index in range(len(self.motion_solver.mapping.links)):
                link_name = self.motion_solver.mapping.links[link_index]
                pose = link_states.link_pose(link_name)
                pos_curr = (
                    pose * tt.Vector3(self.motion_solver.mapping.offsets[link_index])).value
                keypoint_positions.append(pos_curr)

            frame_data = {}
            frame_data["time"] = joint_frame.time
            frame_data["points"] = [utils.point2dict(
                p) for p in keypoint_positions]
            frame_data["joints"] = dict(zip(
                self.motion_solver.robot_model.variable_names,
                [v.value for v in joint_frame.joint_states.serialize()]
            ))
            sequence_data.append(frame_data)

        with open(self.out_path, "w") as out_file:
            yaml.dump(sequence_data, out_file)
