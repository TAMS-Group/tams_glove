#!/usr/bin/env python3

import cstruct
import struct
import pymittenwire

LedMessage = cstruct.parse("""
struct LedMessage {
    uint32_t magic;
    uint8_t leds[27];
    uint8_t reserved;
};
""")


def make_camera_gain_code(analog_gain=31, analog_gain_multiplier=0, digital_gain=0):
    # digital_gain = 15  # [14:8] [0..15*8] digital gain = 1 + value / 8
    # analog_gain_multiplier = 0  # [6] [0..1] 2x analog gain multiplier
    # analog_gain = 31  # [5:0] [0..31] analog gain
    gain_code = ((digital_gain * 8) <<
                 8) | (analog_gain_multiplier << 6) | analog_gain
    return gain_code


def device_config(type, port, width, height, frametime,
                  binning=1, skip=1, blacklevel=0, left=-1, top=-1, exposure=802,
                  analog_gain_red=31,
                  analog_gain_green=31,
                  analog_gain_blue=31,
                  double_gain=0,
                  delay=0,
                  digital_gain=0):
    if left < 0:
        left = int(round(((16 + 2592) - width) / 2))
    if top < 0:
        top = int(round(((54 + 1944) - height) / 2))

    # increment = skip * binning * 2
    increment = skip * 2

    cam = pymittenwire.CameraMessage()
    cam.magic = 0xCA62
    cam.left = int(round(left / increment)) * increment
    cam.top = int(round(top / increment)) * increment
    cam.width = int(round(width / increment)) * increment
    cam.height = int(round(height / increment)) * increment
    cam.binning = binning - 1
    cam.skip = skip - 1
    cam.blacklevel = int(round(blacklevel * 4095))
    cam.digital_gain = digital_gain
    cam.analog_gain_red = analog_gain_red
    cam.analog_gain_green = analog_gain_green
    cam.analog_gain_blue = analog_gain_blue
    cam.shutter = exposure
    cam.timestamp = 123
    cam.double_gain = double_gain

    # ---

    d2 = frametime

    if 0:
        f_pixclk = 100 * 1000 * 1000

        # if cam.binning == 0:
        #     w_dc = 80
        # if cam.binning == 1:
        #     w_dc = 40
        # if cam.binning == 3:
        #     w_dc = 20
        w_dc = [80, 40, 40, 20][cam.binning]

        readout_delay = 0
        hb_min = (696 * cam.binning) + 46 + readout_delay + (w_dc / 2)

        d = (cam.shutter + 8) * (cam.width // 2 + hb_min) * (2 / f_pixclk)

        d2 -= d

    idelay = max(0, min(1024 * 1024 - 1, int(round(d2 * delay * 1000000))))

    # print("delay", frametime, d2, idelay)

    cam.delay = idelay

    # ---

    hub = pymittenwire.HubMessage()
    hub.port = port
    hub.type = type
    hub.frametime = int(round(frametime * 1000 * 1000))

    msg = struct.pack("<II",
                      0xb1c596c8,
                      0x5bdf4276,
                      ) + hub.pack() + cam.pack()

    # print("message", codecs.encode(msg, "hex"))

    return msg
