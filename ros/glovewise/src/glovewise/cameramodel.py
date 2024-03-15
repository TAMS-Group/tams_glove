import yaml
import tractor as tr
import tractor.types_double as tt
import numpy
import cv2
import tf.transformations


class CameraModel:

    def __init__(self, name="", projection=numpy.identity(3), distortion=numpy.zeros(5), resolution=(1, 1)):

        self.name = name

        self.width = tt.Scalar(resolution[0])
        self.height = tt.Scalar(resolution[1])

        self.pose = tt.Pose.identity

        self.fx = tt.Scalar(projection[0, 0])
        self.fy = tt.Scalar(projection[1, 1])
        self.cx = tt.Scalar(projection[0, 2])
        self.cy = tt.Scalar(projection[1, 2])

        self.k1, self.k2, self.p1, self.p2, self.k3 = [
            tt.Scalar(d) for d in distortion]

    def project_point(self, point):

        point = tr.inverse(self.pose) * point

        xc, yc, zc = tr.unpack(point)

        x1 = xc / zc
        y1 = yc / zc

        r2 = (x1 * x1) + (y1 * y1)
        r4 = r2 * r2
        r6 = r4 * r2

        f2 = (tt.Scalar(1) + self.k1 * r2 + self.k2 * r4 + self.k3 * r6)
        x2 = x1 * f2 + tt.Scalar(2) * self.p1 * x1 * \
            y1 + self.p2 * (r2 + tt.Scalar(2) * x1 * x1)
        y2 = y1 * f2 + self.p1 * \
            (r2 + tt.Scalar(2) * y1 * y1) + tt.Scalar(2) * self.p2 * x1 * y1

        return (self.fx * x2 + self.cx, self.fy * y2 + self.cy)

    def project_points_cv(self, points):

        camera_matrix = numpy.array([
            [self.fx.value, 0.0, self.cx.value],
            [0.0, self.fy.value, self.cy.value],
            [0.0, 0.0, 1.0],
        ], dtype=numpy.float64)

        dist_coeffs = numpy.array([
            self.k1.value, self.k2.value, self.p1.value, self.p2.value, self.k3.value
        ], dtype=numpy.float64)

        pose = self.pose
        pose = tr.inverse(pose)

        rquat = tr.orientation(pose).value

        rmat = tf.transformations.quaternion_matrix(rquat)[:3, :3]

        rvec, _ = cv2.Rodrigues(rmat)
        rvec = numpy.array(rvec, dtype=numpy.float64)

        tvec = tr.position(pose).value
        tvec = numpy.array(tvec, dtype=numpy.float64)

        points = numpy.array(points, dtype=numpy.float64)

        image_points, _ = cv2.projectPoints(
            points,
            rvec,
            tvec,
            camera_matrix,
            dist_coeffs
        )

        return image_points

    def compute_uv(self, points):

        camrot = tr.orientation(self.pose)
        uu = []
        vv = []
        for point in points:
            ray_direction = self.compute_ray_direction(point)
            ray_direction = camrot * tt.Vector3(ray_direction)
            side_vector = (camrot * tt.Vector3(1, 0, 0))
            up_vector = (camrot * tt.Vector3(0, 1, 0))
            normals = [
                tr.cross(side_vector, ray_direction),
                tr.cross(up_vector, ray_direction),
            ]
            normals = [tr.normalized(n).value for n in normals]
            u, v = normals
            uu.append(u)
            vv.append(v)
        return uu, vv

    def pack(cam):
        return {
            "name": cam.name,
            "resolution": {
                "width": int(round(cam.width.value)),
                "height": int(round(cam.height.value)),
            },
            "intrinsics": {
                "projection": {
                    "cx": cam.cx.value,
                    "cy": cam.cy.value,
                    "fx": cam.fx.value,
                    "fy": cam.fy.value,
                },
                "distortion": {
                    "k1": cam.k1.value,
                    "k2": cam.k2.value,
                    "k3": cam.k3.value,
                    "p1": cam.p1.value,
                    "p2": cam.p2.value,
                },
            },
            "pose": {
                "position": {
                    "x": float(tr.translation(cam.pose).value[0]),
                    "y": float(tr.translation(cam.pose).value[1]),
                    "z": float(tr.translation(cam.pose).value[2]),
                },
                "orientation": {
                    "x": float(tr.orientation(cam.pose).value[0]),
                    "y": float(tr.orientation(cam.pose).value[1]),
                    "z": float(tr.orientation(cam.pose).value[2]),
                    "w": float(tr.orientation(cam.pose).value[3]),
                },
            },
        }

    def unpack(self, data):
        self.name = data["name"]

        self.width = tt.Scalar(data["resolution"]["width"])
        self.height = tt.Scalar(data["resolution"]["height"])

        self.cx = tt.Scalar(data["intrinsics"]["projection"]["cx"])
        self.cy = tt.Scalar(data["intrinsics"]["projection"]["cy"])
        self.fx = tt.Scalar(data["intrinsics"]["projection"]["fx"])
        self.fy = tt.Scalar(data["intrinsics"]["projection"]["fy"])

        self.k1 = tt.Scalar(data["intrinsics"]["distortion"]["k1"])
        self.k2 = tt.Scalar(data["intrinsics"]["distortion"]["k2"])
        self.k3 = tt.Scalar(data["intrinsics"]["distortion"]["k3"])
        self.p1 = tt.Scalar(data["intrinsics"]["distortion"]["p1"])
        self.p2 = tt.Scalar(data["intrinsics"]["distortion"]["p2"])

        pos = data["pose"]["position"]
        quat = data["pose"]["orientation"]

        self.pose = tt.Pose()
        self.pose.value = [
            [
                pos["x"],
                pos["y"],
                pos["z"],
            ],
            [
                quat["x"],
                quat["y"],
                quat["z"],
                quat["w"],
            ],
        ]

    def compute_ray_directions(cam, points2d):
        projection = numpy.array([
            [cam.fx.value, 0.0, cam.cx.value],
            [0.0, cam.fy.value, cam.cy.value],
            [0.0, 0.0, 1.0],
        ], dtype=numpy.float64)
        distortion = numpy.array([
            cam.k1.value,
            cam.k2.value,
            cam.p1.value,
            cam.p2.value,
            cam.k3.value,
        ], dtype=numpy.float64)
        pr = cv2.undistortPoints(points2d, projection, distortion)
        pr = [numpy.array([p[0][0], p[0][1], 1]) for p in pr]
        return pr

    def compute_ray_direction(self, point2d):
        return self.compute_ray_directions(numpy.array([point2d], dtype=numpy.float64))[0]
