#!/usr/bin/env python3

import mittenwire
import rospy
import rosbag
import sdl2
import sys
import time
import sensor_msgs.msg
import threading
import numpy as np


class RecordingApp(mittenwire.App):

    def __init__(self):
        self.w = 0
        self.h = 0
        self.lock = threading.Lock()
        self.bag = None
        self.counter_images = 0
        self.counter_tactile = 0
        self.timeref = np.uint32(0)
        super().__init__("View")

    def close(self):
        print("closing bag if open")
        b = None
        with self.lock:
            if self.bag is not None:
                b = self.bag
                self.bag = None
        if b is not None:
            print("closing bag")
            b.close()
            b = None
            print("bag closed")
            return True
        else:
            return False

    def run(self):
        super().run()
        self.close()

    def time2ros(self, t):
        rtime = int(np.uint32(np.uint32(t) +
                    np.uint32(1000000) - np.uint32(self.timeref)))
        rtime = rospy.Time(0, rtime * 1000)
        # print(t, rtime, time.time() - rtime.to_sec())
        return rtime

    def process_tactile_matrix(self, msg):
        super().process_tactile_matrix(msg)
        t = self.time2ros(msg.timestamp)
        msg.header.stamp = t
        with self.lock:
            if self.bag is not None:
                self.bag.write("/impedance_matrix", msg, t=t)
        self.counter_tactile += 1

    def process_tactile_status(self, msg):
        super().process_tactile_status(msg)
        t = self.time2ros(msg.timestamp)
        msg.header.stamp = t
        with self.lock:
            if self.bag is not None:
                self.bag.write("/glove_status", msg, t=t)
        self.counter_tactile += 1

    def process_camera_image(self, msg):
        super().process_camera_image(msg)
        self.w = msg.width
        self.h = msg.height
        t = self.time2ros(msg.request_timestamp)
        with self.lock:
            if self.bag is not None:
                image_message = self.image2ros(msg)
                image_message.header.stamp = t
                info_message = sensor_msgs.msg.CameraInfo()
                info_message.header.stamp = t
                info_message.width = 2592
                info_message.height = 1944
                info_message.binning_x = msg.skip
                info_message.binning_y = msg.skip
                info_message.roi.x_offset = msg.left
                info_message.roi.y_offset = msg.top
                info_message.roi.width = msg.width * msg.skip
                info_message.roi.height = msg.height * msg.skip
                self.bag.write("/cam"+str(msg.channel) +
                               "/image_raw", image_message, t=t)
                self.bag.write("/cam"+str(msg.channel) +
                               "/camera_info", info_message, t=t)
        self.counter_images += 1

    def render(self, width, height):
        with self.lock:
            if self.bag:
                text = "recording " + self.bag.filename
            else:
                text = "press space to start recording"
                bagname = self.suggest_bag_name()
                if bagname:
                    text += " \"" + bagname + "\""
            text += " - cam " + str(self.counter_images) + " - tac " + \
                    str(self.counter_tactile)
        text += (" - fps %.3f" % self.frames_per_second)
        text += " - %i x %i" % (self.w, self.h)
        return super().render(width, height, text)

    def suggest_bag_name(self):
        if len(sys.argv) > 1:
            return sys.argv[1]
        else:
            return None

    def handle_event(self, event):
        super().handle_event(event)
        if event.type == sdl2.SDL_KEYDOWN:
            if event.key.keysym.sym == sdl2.SDLK_SPACE:
                if not self.close():
                    with self.lock:
                        if self.bag is None:
                            name = "bag-"
                            name += str(time.time_ns())
                            bagname = self.suggest_bag_name()
                            if bagname:
                                name += "-" + bagname
                            name += ".bag"
                            self.bag = rosbag.Bag(name, "w")
                            self.counter_images = 0
                            self.counter_tactile = 0
                            self.timeref = np.uint32(self.frame_timestamp)
                        else:
                            closebag = True
