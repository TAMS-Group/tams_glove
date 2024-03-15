#!/usr/bin/env python3

import time
import numpy as np
import rospy
import sdl2
import sdl2.ext
import threading
import PIL
import PIL.ImageFont
import PIL.ImageDraw
import PIL.Image
from . import net, image, mosaic, gui, profiler
from OpenGL import GL
import moderngl
import struct
import rospkg
import pymittenwire
import mittenwire.msg
import sensor_msgs.msg
import cv2


class App:

    class Frame:
        def __init__(self, message):
            self.message = message
            self.time = time.time()

    def __enter__(self):
        self.network.__enter__()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        print("app shutting down")
        self.network.__exit__(exc_type, exc_val, exc_tb)
        self.window.__exit__(exc_type, exc_val, exc_tb)
        print("app shut down")

    def preprocess(self, img):

        with profiler.Profiler("debayer", 0):
            img = cv2.cvtColor(img, cv2.COLOR_BAYER_GR2RGB)

        return img

    def postprocess(self, img):

        hconf = self.network.hub_config

        if hconf is not False and hconf.enable_filters:

            img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV_FULL)
            h, s, v = cv2.split(img)

            h += np.uint8(int(hconf.hue * 256 / 360))
            s = cv2.multiply(s, (1, 1, 1, 1), scale=hconf.saturation * 0.01)

            img = cv2.merge([h, s, v])
            img = cv2.cvtColor(img, cv2.COLOR_HSV2RGB_FULL)

            img = cv2.multiply(img, (1, 1, 1, 1),
                               scale=hconf.brightness * 0.01)

            if hconf.invert:
                img = 255 - img

            # mittenwire.tonemap(img)

            # img = np.array(img, dtype=np.uint8)
            # mittenwire.sqrt_tonemap_srgb_8(img, hconf.brightness * 0.01)

        # mittenwire.linear2srgb(img)

        if hconf is not False and hconf.enable_filters:
            if hconf.median_blur > 0:
                img = cv2.medianBlur(img, hconf.median_blur * 2 + 1)

        return img

    def process_camera_image(self, msg):

        if not self.network:
            return

        self.fps_frames += 1
        sec = int(time.time())
        if sec != self.fps_second:
            self.frames_per_second = self.fps_frames / \
                max(1e-9, len(self.get_images()))
            self.fps_second = sec
            self.fps_frames = 0

        with self.update_condition:
            if not self.update_flag:
                self.update_flag = True
                self.update_condition.notify()

        with self.frame_lock:
            self.frame_images[msg.channel] = self.Frame(msg)

    def process_tactile_matrix(self, msg):
        if not self.network:
            return
        self.pub_mat.publish(msg)

    def process_tactile_status(self, msg):
        if not self.network:
            return
        self.pub_status.publish(msg)

    def __init__(self, title):

        self.tonemap_lut = np.square(np.linspace(0.0, 1.0, 256))

        self.fps_second = 0
        self.fps_frames = 0

        self.frames_per_second = 0
        self.frame_timestamp = 0
        self.frame_time = 0
        self.frame_images = {}
        self.frame_lock = threading.Lock()

        self.pub_mat = rospy.Publisher("impedance_matrix", mittenwire.msg.ImpedanceMatrix,
                                       latch=False, queue_size=100)

        self.pub_status = rospy.Publisher("glove_status", mittenwire.msg.GloveStatus,
                                          latch=False, queue_size=100)

        self.update_condition = threading.Condition()
        self.update_flag = True

        self.network = None
        self.network = net.Network(
            camera_callback=self.process_camera_image,
            tactile_matrix_callback=self.process_tactile_matrix,
            tactile_status_callback=self.process_tactile_status)
        self.window = gui.Window(title, 1600, 900)

        self.minsize = 16

        pkg = rospkg.RosPack()

        self.gl_context = moderngl.create_context()

        self.gl_program = self.gl_context.program(
            vertex_shader=open(pkg.get_path("mittenwire") +
                               "/data/post_vs.glsl").read(),
            fragment_shader=open(pkg.get_path(
                "mittenwire") + "/data/post_fs.glsl").read()
        )

        vertices = struct.pack("8f",
                               0.0, 0.0,
                               1.0, 0.0,
                               1.0, 1.0,
                               0.0, 1.0,
                               )
        self.gl_vbo = self.gl_context.buffer(vertices)
        self.gl_vao = self.gl_context.simple_vertex_array(
            self.gl_program, self.gl_vbo, "in_vert")

        self.gl_texture = GL.glGenTextures(1)
        print("texture", self.gl_texture)

    def render_message(self, message):
        if isinstance(message, pymittenwire.ImageMessage):
            return message.data
        return None

    def image2ros(self, image):
        msg = sensor_msgs.msg.Image()
        msg.width = image.width
        msg.step = image.width
        msg.height = image.height
        msg.encoding = "bayer_grbg8"
        msg.data = bytes(image.data)
        return msg

    def get_images(self):
        images = []
        with self.frame_lock:
            images = self.frame_images.copy()
        images = [images[i] for i in range(self.network.port_count) if ((i in images) and (
            (images[i].message) and (time.time() < images[i].time + 3.0)))]
        return images

    def render_mosaic(self, width, height):

        width = max(self.minsize, width)
        height = max(self.minsize, height)

        # hconf = self.network.hub_config
        # prefix = self.network.get_mode_prefix()

        images = self.get_images()

        images = [self.render_message(img.message) for img in images]

        images = [img for img in images if img is not None]

        images = [self.preprocess(img) for img in images]
        img = mosaic.make_mosaic(
            images, (height, width),
            # (hconf[prefix+"height"], hconf[prefix+"width"])
            (300, 400), np.uint8)
        img = self.postprocess(img)

        return img

    def render(self, width, height):
        return self.render_mosaic(width, height)

    def render(self, width, height, text):

        bar = 32

        image = np.zeros([bar, width, 3], dtype=np.uint8)

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.7
        thickness = 2
        size = cv2.getTextSize(text, font, scale, thickness)[0]
        x = (image.shape[1] - size[0]) // 2
        y = (image.shape[0] + size[1]) // 2
        cv2.putText(image, text, (x, y),
                    font, scale, (255, 255, 255), thickness)

        image = np.vstack([image.astype(np.uint8) * 50,
                          self.render_mosaic(width, height - bar)])

        return image

    def handle_event(self, event):
        if event.type == sdl2.SDL_WINDOWEVENT:
            with self.update_condition:
                self.update_flag = True
                self.update_condition.notify()
        if event.type == sdl2.SDL_KEYDOWN:
            with self.update_condition:
                self.update_flag = True
                self.update_condition.notify()
            if event.key.keysym.sym == sdl2.SDLK_q:
                self.shutdown_flag = True
        if event.type == sdl2.SDL_QUIT:
            self.shutdown_flag = True

    def run(self):

        self.shutdown_flag = False

        while not self.shutdown_flag and not rospy.is_shutdown():

            do_update = False
            with self.update_condition:
                if self.update_flag:
                    do_update = True
                    self.update_flag = False
                else:
                    self.update_condition.wait(1.0)

            if do_update:

                self.window.begin()

                width = self.window.width // 4 * 4
                height = self.window.height // 4 * 4

                with profiler.Profiler("render", 0):
                    img = self.render(width, height)

                with profiler.Profiler("gl", 0):

                    GL.glViewport(0, 0, width, height)

                    GL.glBindTexture(GL.GL_TEXTURE_2D, self.gl_texture)
                    GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_RGBA8, width,
                                    height, 0, GL.GL_BGR, GL.GL_UNSIGNED_BYTE, img)
                    GL.glTexParameteri(
                        GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_NEAREST)
                    GL.glTexParameteri(
                        GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_NEAREST)

                    self.gl_vao.render(GL.GL_QUADS)

                    self.window.swap()
                    self.window.end()

            events = sdl2.ext.get_events()
            for event in events:
                self.handle_event(event)
