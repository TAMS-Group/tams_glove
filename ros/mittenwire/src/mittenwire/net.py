#!/usr/bin/env python3

import mittenwire
import time
import dynamic_reconfigure.server
import mittenwire.cfg.SensorConfig
import mittenwire.cfg.HubConfig
import struct


class Network:

    led_color_ok = 0x000040
    led_color_off = 0x400010
    led_color_err = 0x004000
    led_color_noack = 0x004020

    def set_led_color(self, leds, i, color):
        leds[i * 3 + 0] = ((color >> 0) & 0xff)
        leds[i * 3 + 1] = ((color >> 8) & 0xff)
        leds[i * 3 + 2] = ((color >> 16) & 0xff)

    def get_region_of_interest(self, iport):
        if iport in self.attention_map:
            return self.attention_map[iport]
        return None

    def get_mode_prefix(self, iport):
        mode = "video"

        # if self.mode_override:
        #     mode = self.mode_override
        # else:

        hconf = self.hub_config
        if hconf is not False:
            mode = ["video", "photo", "zoom"][hconf.mode]

        roi = self.get_region_of_interest(iport)
        if roi:
            if roi[0] > time.time():
                mode = roi[3]
            else:
                mode = "video"

        return mode + "_"

    def make_port_config(self, iport):
        cfg = b""
        hconf = self.hub_config
        prefix = self.get_mode_prefix(iport)
        if hconf is not False:
            port_type = hconf["port_"+str(iport)]
            # print(iport, port_type)
            w = hconf[prefix+"width"]
            h = hconf[prefix+"height"]
            x = 16 + (2592 - w) * 0.5
            y = 54 + (1944 - h) * 0.5
            skip = hconf[prefix+"skip"]
            binning = hconf[prefix+"binning"]
            roi = self.get_region_of_interest(iport)
            if roi and roi[0] > time.time():
                _, roi_x, roi_y, _ = roi
                x = roi_x - w // 2
                y = roi_y - h // 2
                x = min(max(16, x), 16 + 2592 - w)
                y = min(max(54, y), 54 + 1944 - h)
            cfg += mittenwire.device_config(
                type=port_type,
                port=iport,
                left=x, top=y, width=w, height=h,
                exposure=hconf.exposure,
                analog_gain_red=(hconf.analog_gain_red - 1),
                analog_gain_green=(hconf.analog_gain_green - 1),
                analog_gain_blue=(hconf.analog_gain_blue - 1),
                binning=binning,
                skip=skip,
                blacklevel=hconf.blacklevel,
                delay=hconf[prefix+"delay"],
                frametime=max(0.00000001, 1 /
                              max(0.00000001, hconf[prefix+"framerate"])),
                double_gain=hconf.double_gain,
                digital_gain=(hconf.digital_gain - 1)
            )
        return cfg

    def writer_callback(self):
        # print("begin writer callback")

        hconf = self.hub_config

        cfg = b""
        if hconf is not False:
            for iport in range(self.port_count):
                cfg += self.make_port_config(iport)
        cfg += struct.pack("<I", 0xb1c596c8)

        if 1:
            lm = mittenwire.messages.LedMessage()
            lm.magic = 0x2C43D703
            leds = lm.leds
            for i in range(self.led_count):
                self.set_led_color(leds, i, 0x10)
            self.set_led_color(leds, 4, self.led_color_ok)
            if hconf is not False:
                for i in range(self.port_count):
                    port_type = hconf["port_"+str(i)]
                    if port_type != 0:
                        if time.time() - self.update_times[i] < 1.0:
                            if self.messages_valid[i]:
                                self.set_led_color(
                                    leds, self.port_leds[i], self.led_color_ok)
                            else:
                                self.set_led_color(
                                    leds, self.port_leds[i], self.led_color_err)
                        else:
                            self.set_led_color(
                                leds, self.port_leds[i], self.led_color_noack)
                    else:
                        self.set_led_color(
                            leds, self.port_leds[i], self.led_color_off)
            lm.leds = leds
            cfg += lm.pack()

        # print("end writer callback")
        return cfg

    def _camera_callback(self, msg):

        if not msg.valid:
            print("image corrupted")
            return

        # print("camera_callback_wrapper begin")
        self.update_times[msg.channel] = time.time()
        self.messages_valid[msg.channel] = msg.valid
        # print("check camera_callback")
        if self.camera_callback is not False:
            self.camera_callback(msg)
        # print("camera_callback_wrapper end")

    def _tactile_matrix_callback(self, msg):
        self.update_times[msg.channel] = time.time()
        self.messages_valid[msg.channel] = 1
        # print(msg)
        if self.tactile_matrix_callback_function is not False:
            # print(msg)
            # print("a")
            # print(self.tactile_matrix_callback_function)
            self.tactile_matrix_callback_function(msg)
            # print("b")

    def _tactile_status_callback(self, msg):
        self.update_times[msg.channel] = time.time()
        self.messages_valid[msg.channel] = 1
        # print(msg)
        if self.tactile_status_callback_function is not False:
            self.tactile_status_callback_function(msg)

    def remove_sensor(self, iport):
        if self.sensors[iport]:
            print("disconnect sensor", iport)
            self.hub.disconnect(iport, self.sensors[iport])
            self.sensors[iport] = None

    def update_config(self):
        hconf = self.hub_config
        if hconf is not False:
            print("keys", hconf.keys())
            print("groups", hconf.groups.keys())
            for iport in range(self.port_count):
                port_type = hconf["port_"+str(iport)]
                if port_type == 1:
                    if not isinstance(self.sensors[iport], mittenwire.Camera):
                        self.remove_sensor(iport)
                        print("creating camera at port", iport)
                        cam = mittenwire.Camera(
                            self._camera_callback)
                        self.sensors[iport] = cam
                        self.hub.connect(iport, cam)
                elif port_type == 2:
                    if not isinstance(self.sensors[iport], mittenwire.Glove):
                        self.remove_sensor(iport)
                        print("creating tactile at port", iport)
                        cam = mittenwire.Glove(
                            self._tactile_matrix_callback, self._tactile_status_callback)
                        self.sensors[iport] = cam
                        self.hub.connect(iport, cam)
                else:
                    self.remove_sensor(iport)
        self.writer.update()

    def hub_config_callback(self, config, level):
        print("hub_config_callback")
        self.hub_config = config
        self.update_config()
        return config

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        print("net del")
        for sensor in self.sensors:
            del sensor
        del self.sensors
        del self.writer
        del self.hub
        del self.master
        del self.event_loop
        del self.device
        del self.context
        print("net deleted")

    def __init__(self, camera_callback=None, tactile_matrix_callback=None, tactile_status_callback=None):

        # self.mode_override = None

        self.camera_callback = camera_callback
        self.tactile_matrix_callback_function = tactile_matrix_callback
        self.tactile_status_callback_function = tactile_status_callback

        self.attention_map = {}

        port_count = 8
        self.port_count = port_count

        self.hub_config = False

        self.update_times = [0] * port_count
        self.messages_valid = [True] * port_count

        ctx = mittenwire.Context()
        self.context = ctx

        dev = mittenwire.Device()
        dev.open_vid_pid(ctx, 0x0403, 0x601f)
        dev.reset()
        self.device = dev

        # print("power on")
        dev.set_gpio_direction(1, 1)
        dev.set_gpio_level(1, 1)

        # print("start")
        dev.start()

        buffer_mebibytes = 12

        net = mittenwire.Master(dev, buffer_mebibytes * 32, 32 * 1024)
        self.master = net

        hub = mittenwire.Hub(net)
        self.hub = hub

        self.led_count = 9
        self.port_leds = [5, 6, 7, 8, 3, 2, 1, 0]
        self.usb_led = 4

        wr = mittenwire.Writer(dev, self.writer_callback, 1.0)
        self.writer = wr

        self.sensors = [None] * port_count

        # print("create event loop")
        evl = mittenwire.EventLoop(ctx, [dev])
        self.event_loop = evl

        self.update_config()

        config_servers = []
        config_servers.append(dynamic_reconfigure.server.Server(
            mittenwire.cfg.HubConfig, lambda config, level: self.hub_config_callback(config, level)))
        self.config_servers = config_servers

        # print("net ready")
