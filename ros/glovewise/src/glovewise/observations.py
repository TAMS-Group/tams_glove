import yaml
import typing
import numpy as np


class KeypointObservations:

    def unpack(self, data):
        self.camera_projection_map = data

    def triangulate(self, multicam):
        return multicam.triangulate(self.camera_projection_map)

    def is_empty(self):
        return len(self.camera_projection_map) == 0


class ObservationFrame:

    time: float
    keypoint_observation_map: typing.Dict

    def unpack(self, data):
        self.time = data["time"]
        self.keypoint_observation_map = {}
        for p in data["hand"]["keypoints"].items():
            o = KeypointObservations()
            o.unpack(p[1])
            self.keypoint_observation_map[p[0]] = o

    def is_empty(self):
        empty = True
        for v in self.keypoint_observation_map.values():
            if not v.is_empty():
                empty = False
        return empty


class ObservationSequence:

    frames: typing.List[ObservationFrame]

    def fill_gaps(self):
        tt = np.array([f.time for f in self.frames])
        dt = np.median(tt[1:] - tt[:-1])

        frames2 = []
        for frame in self.frames:
            while len(frames2) and frame.time - frames2[-1].time > dt * 1.5:
                f2 = ObservationFrame()
                f2.time = frames2[-1].time + dt
                f2.keypoint_observation_map = {}
                frames2.append(f2)
            frames2.append(frame)
        self.frames = frames2

    def unpack(self, data):
        self.frames = []
        for frame_data in data["detections"]:
            f = ObservationFrame()
            f.unpack(frame_data)
            self.frames.append(f)

    def load(filename):
        self = ObservationSequence()
        with open(filename, "r") as data_file:
            data = yaml.load(data_file, Loader=yaml.CLoader)
            self.unpack(data)
        return self

    def print(self):
        print("sequence")
        for frame in self.frames:
            print(" frame", frame.time)
            for keypoint, observations in frame.keypoint_observation_map.items():
                print("  keypoint", keypoint)
                for camera, projection in observations.camera_projection_map.items():
                    print("   camera", camera, "projection", projection)
