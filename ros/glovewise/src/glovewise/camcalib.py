import yaml
import numpy


def group(data, key):
    ret = {}
    for v in data:
        k = key(v)
        if not k in ret:
            ret[k] = []
        ret[k].append(v)
    return ret.items()


def transform_points_affine(matrix, points):
    matrix = numpy.array(matrix)
    return [
        numpy.dot(matrix, [[p[0]], [p[1]], [p[2]], [1.0]]).flatten()[:3]
        for p in points
    ]


class CameraCalibrationData:

    def load(self, data_file_names):

        target = False
        resolution = False
        camera_names = set()
        observations = []

        for data_file_name in data_file_names:
            with open(data_file_name, "r") as data_file:
                data = yaml.load(data_file, Loader=yaml.CLoader)

                data = data["calibration_data"]
                for cam in data["cameras"]:
                    w = cam["resolution"]["width"]
                    h = cam["resolution"]["height"]
                    res = (w, h)
                    if resolution is not False and res != resolution:
                        raise Exception("incompatible camera resolutions")
                    resolution = res
                    camera_names.add(cam["name"])

                observations += group(data["observations"],
                                      key=lambda x: x["time"])

                t = data["object"]
                if t != target:
                    if target is not False:
                        raise Exception("incompatible calibration objects")
                    target = t

        camera_names = sorted(list(camera_names))

        markers = {}
        for i in range(len(target["markers"])):
            m = target["markers"][i]
            p = m["position"]
            markers[m["name"]] = (p["x"], p["y"], p["z"])
        marker_names = list(markers.keys())
        marker_positions = list(markers.values())

        observations = [[frame[0], group(frame[1], key=lambda x: x["camera"])]
                        for frame in observations]

        self.target = target
        self.resolution = resolution
        self.camera_names = camera_names
        self.observations = observations
        self.marker_names = marker_names
        self.marker_positions = marker_positions
        self.markers = markers
