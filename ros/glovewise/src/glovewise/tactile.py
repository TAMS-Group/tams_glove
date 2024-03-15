import cv2
import numpy as np
import cv_bridge
import rospy
import glovewise
import rosbag
import scipy.interpolate
import mittenwire.msg
from . import utils


class TactileSequence:

    width: int
    height: int
    times: np.ndarray
    matrices: np.ndarray
    imu_times: np.ndarray
    imu_linear_accelerations: np.ndarray
    imu_angular_velocities: np.ndarray

    def load(bagpath):

        seq = TactileSequence()
        seq.width = 16
        seq.height = 16
        seq.times = []
        seq.matrices = []
        seq.imu_times = []
        seq.imu_linear_accelerations = []
        seq.imu_angular_velocities = []

        print("reading bag")
        with rosbag.Bag(bagpath, "r") as bag:
            for topic, message, mtime in bag.read_messages(topics=["/impedance_matrix", "/glove_status"]):
                if topic == "/impedance_matrix":
                    data = np.array(message.inphase, dtype=np.float32) + \
                        np.array(message.quadrature, dtype=np.float32) * 1j
                    data[~np.array(message.validity, dtype=bool)] = np.nan
                    data = data.reshape((seq.height, seq.width))
                    seq.times.append(mtime.to_sec())
                    seq.matrices.append(data)
                if topic == "/glove_status":
                    status: mittenwire.msg.GloveStatus = message
                    seq.imu_times.append(mtime.to_sec())
                    seq.imu_linear_accelerations.append(
                        utils.vector_message_to_array(status.imu_linear_acceleration))
                    seq.imu_angular_velocities.append(
                        utils.vector_message_to_array(status.imu_angular_velocity))

        seq.times = np.array(seq.times)
        seq.matrices = np.array(seq.matrices)
        seq.imu_times = np.array(seq.imu_times)
        seq.imu_linear_accelerations = np.array(seq.imu_linear_accelerations)
        seq.imu_angular_velocities = np.array(seq.imu_angular_velocities)

        return seq

    def process(self):

        print("interpolate")
        for y in range(self.height):
            for x in range(self.width):
                mask = ~np.isnan(self.matrices[:, y, x])
                times = self.times[mask]
                values = self.matrices[:, y, x][mask]
                interp = scipy.interpolate.interp1d(
                    times, values, axis=0, bounds_error=False, fill_value="extrapolate", kind="linear")
                self.matrices[:, y, x] = interp(self.times)

        print("filter")
        if True:
            window = np.kaiser(25, 3)
            window = window / np.sum(window)
            l = len(window)
            for y in range(self.height):
                for x in range(self.width):
                    d = self.matrices[:, y, x]

                    d = np.pad(d, l, mode="edge")
                    d = np.convolve(d, window, "same")
                    d = d[l:-l]

                    self.matrices[:, y, x] = d

        return self


class InertialInterpolator:

    def __init__(self, sequence: TactileSequence):
        self.data = np.hstack([sequence.imu_linear_accelerations,
                               sequence.imu_angular_velocities])

        self.interp = [
            scipy.interpolate.interp1d(
                sequence.imu_times, series, axis=0, bounds_error=False, fill_value="extrapolate", kind="linear")
            for series in np.transpose(self.data)
        ]

    def interpolate(self, t: float):

        data = [interp(t) for interp in self.interp]
        return [data[0:3], data[3:6]]


class TactileInterpolator:

    def __init__(self, sequence: TactileSequence, tstart: float, tend: float = None):
        self.times = sequence.times
        self.width = sequence.width
        self.height = sequence.height
        self.interpolators = []
        for y in range(self.height):
            row = []
            self.interpolators.append(row)
            for x in range(self.width):
                times = sequence.times[:]
                values = sequence.matrices[:, y, x]
                interp = scipy.interpolate.interp1d(
                    times, values, axis=0, bounds_error=False, fill_value="extrapolate", kind="linear")
                row.append(interp)

        self.hi = np.nanpercentile(np.abs(sequence.matrices).flatten(), 100)

        refs = []
        for dt in np.arange(0, 3, 0.1):
            refs.append(self.interpolate_raw(tstart + dt))
            if tend is not None:
                refs.append(self.interpolate_raw(tend - dt))

        self.ref = np.percentile(refs, 99, axis=0)

    def interpolate_raw(self, t: float):
        if t > self.times[-1]:
            t = self.times[-1]
        ret = np.zeros((self.height, self.width), dtype=np.float32)
        for y in range(self.height):
            for x in range(self.width):
                v = np.abs(self.interpolators[y][x](t))
                ret[y, x] = v
        return ret

    def interpolate(self, t: float):
        v = self.interpolate_raw(t)
        v = (v - self.ref) / (self.hi - self.ref)
        v[v < 0] = 0
        return v


class TactileLayout:

    def __init__(self):

        layout = """
-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
-- -- a2 b2 c2 -- d2 e2 f2 -- a7 b7 c7 -- d7 e7 f7 -- -- --
-- -- a3 b3 c3 -- d4 e4 f4 -- a8 b8 c8 -- d8 e8 f8 -- -- --
-- -- a4 b4 c4 -- d3 e3 f3 -- a9 b9 c9 -- d9 e9 f9 -- -- --
-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
-- -- b0 b1 c1 -- e0 e1 f1 -- b5 b6 c6 -- e5 e6 f6 -- -- --
-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
-- -- a1 a0 c0 -- d1 d0 f0 -- a6 a5 c5 -- d6 d5 f5 -- -- --
-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
-- -- -- -- -- -- 0c 0b 0a 0f -- -- -- -- 72 82 92 -- -- --
-- -- -- -- -- -- 1c 1b 1a 1f -- -- -- -- 71 81 91 -- -- --
-- -- -- -- -- -- 2c 2b 2a 2f -- -- -- -- 70 80 90 -- -- --
-- -- -- -- -- -- 3c 3b 3a 3f 3e 3d -- -- -- -- -- -- -- --
-- -- -- -- -- -- 4c 4b 4a 4f 4e 4d -- -- 84 83 93 -- -- --
-- -- -- -- -- -- 5c 5b 5a 5f 5e 5d -- -- -- -- -- -- -- --
-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
"""

        layout = layout.strip()
        layout = layout.split("\n")
        layout = [row.split() for row in layout]
        layout = [[([int(c, 16) for c in cell] if cell != "--" else False)
                   for cell in row] for row in layout]

        self.layout = layout

        self.num_active_cells = len(
            np.array([1 for row in self.layout for cell in row if cell is not False]))

        active_cells = [
            cell for row in self.layout for cell in row if cell is not False]
        all_cells = [[i, j] for i in range(16) for j in range(16)]
        self.proprioceptive_cells = [
            cell for cell in all_cells if cell not in active_cells]

    def serialize_active_cells(self, matrix):
        return np.array([matrix[cell[1], cell[0]] for row in self.layout for cell in row if cell is not False])

    def serialize_proprioceptive_cells(self, matrix):
        return np.array([matrix[cell[1], cell[0]] for cell in self.proprioceptive_cells])

    def deserialize_active_cells(self, data):
        matrix = np.zeros([len(self.layout), len(self.layout[0])])
        i = 0
        for irow in range(len(self.layout)):
            row = self.layout[irow]
            for icol in range(len(row)):
                cell = row[icol]
                if cell is not False:
                    matrix[irow, icol] = data[i]
                    i += 1
        return matrix

    def map_matrix(self, matrix):
        return np.array([[(matrix[cell[1], cell[0]] if cell is not False else 0)
                          for cell in row] for row in self.layout])

    def map_matrices(self, matrices):
        return np.array([self.map_matrix(matrix) for matrix in matrices])


class TactileRenderer:

    def convert_matrix(self, matrix):
        image = matrix
        image = np.round(image * 255)
        image[image < 0] = 0
        image[image > 255] = 255
        image = image.astype(np.uint8)
        return image

    def render_smooth_image(self, image, scale=30):
        image = cv2.resize(image, (0, 0), fx=scale, fy=scale,
                           interpolation=cv2.INTER_CUBIC)
        image = np.square(image)
        image = self.convert_matrix(image)
        image = cv2.applyColorMap(image, cv2.COLORMAP_JET)
        return image

    def render_smooth_images(self, images, scale=30):
        return [self.render_smooth_image(image, scale) for image in images]

    def render_block_image(self, image, scale=30):
        image = np.square(image)
        image = self.convert_matrix(image)
        image = cv2.applyColorMap(image, cv2.COLORMAP_JET)
        image = cv2.resize(image, (0, 0), fx=scale, fy=scale,
                           interpolation=cv2.INTER_NEAREST)
        return image

    def render_block_images(self, images, scale=30):
        return [self.render_block_image(image, scale) for image in images]

    def render_glow(self, tac):
        image = cv2.resize(
            tac, (tac.shape[1] * 20, tac.shape[0] * 20), interpolation=cv2.INTER_CUBIC)
        image[image < 0] = 0
        colors = np.zeros([image.shape[0], image.shape[1], 4])
        colors[:, :, 0] = image * 4
        colors[:, :, 1] = image * 2
        colors[:, :, 2] = image * 1
        colors[:, :, 3] = 1
        colors = np.array(colors, np.float32)
        return colors
