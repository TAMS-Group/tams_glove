import yaml
import tractor as tr
import tractor.types_double as tt
import numpy
from . import cameramodel
import math


class MultiCameraModel:

    def build_camera_map(self):
        self.camera_map = {}
        for cam in self.cameras:
            self.camera_map[cam.name] = cam

    def __init__(self, cameras=[]):
        self.cameras = cameras
        self.build_camera_map()

    def pack(self):
        return {
            "camera_calibration": [
                cam.pack()
                for cam in self.cameras
            ]
        }

    def unpack(self, multi_data):
        self.cameras = []
        for cam_data in multi_data["camera_calibration"]:

            cam = cameramodel.CameraModel()
            cam.unpack(cam_data)
            self.cameras.append(cam)

        self.build_camera_map()

    def load_calibration_file(self, filename):
        with open(filename, "r") as data_file:
            data = yaml.load(data_file, Loader=yaml.CLoader)
            self.unpack(data)

    def load(filename):
        self = MultiCameraModel()
        self.load_calibration_file(filename)
        return self

    def compute_reprojection_errors(self, observation_map, p3):
        p3 = tt.Vector3(p3)
        ee = []
        for camera_name in observation_map:
            camera = self.camera_map[camera_name]
            ox, oy = observation_map[camera_name]
            px, py = camera.project_point(p3)
            ex = px.value - ox
            ey = py.value - oy
            e = math.sqrt(ex * ex + ey * ey)

            ee.append(e)
        return numpy.array(ee)

    def triangulate(self, observation_map):
        cam_names = [
            camera_name for camera_name in observation_map if camera_name in self.camera_map]
        if len(cam_names) < 2:
            return None
        gradients = []
        residuals = []
        for camera_name in cam_names:
            camera = self.camera_map[camera_name]
            campos = tr.translation(camera.pose)
            camrot = tr.orientation(camera.pose)
            observation_point = observation_map[camera_name]
            ray_direction = camera.compute_ray_direction(observation_point)
            ray_direction = camrot * tt.Vector3(ray_direction)

            side_vector = (camrot * tt.Vector3(1, 0, 0))
            up_vector = (camrot * tt.Vector3(0, 1, 0))
            normals = [
                tr.cross(side_vector, ray_direction),
                tr.cross(up_vector, ray_direction),
            ]
            normals = [tr.normalized(n) for n in normals]
            for i in range(len(normals)):
                gradients.append(normals[i].value)
                residuals.append(tr.dot(normals[i], campos).value)

        solution, _, _, _ = numpy.linalg.lstsq(
            gradients, residuals, rcond=None)

        return solution

    def compute_dimensional_errors(self, observation_map, p3):
        cam_names = [
            camera_name for camera_name in observation_map if camera_name in self.camera_map]
        err_sum = 0.0
        err_div = 0.0
        for camera_name in cam_names:
            camera = self.camera_map[camera_name]
            campos = tr.translation(camera.pose)
            camrot = tr.orientation(camera.pose)
            observation_point = observation_map[camera_name]
            ray_direction = camera.compute_ray_direction(observation_point)
            ray_direction = camrot * tt.Vector3(ray_direction)
            side_vector = (camrot * tt.Vector3(1, 0, 0))
            up_vector = (camrot * tt.Vector3(0, 1, 0))
            normals = [
                tr.cross(side_vector, ray_direction),
                tr.cross(up_vector, ray_direction),
            ]
            normals = [tr.normalized(n) for n in normals]
            for n in normals:
                d = tr.dot(p3 - campos, n).value
                err_sum += abs(d)
                err_div += 1
        return err_sum, err_div
