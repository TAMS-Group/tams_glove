#!/usr/bin/env python3

import rospy
import mittenwire.msg
import cstruct
import struct
import numpy as np


class GlovePacket(cstruct.MemCStruct):
    __byte_order__ = cstruct.LITTLE_ENDIAN
    __def__ = """
        struct {

        uint32_t id;

        uint32_t millis;

        uint16_t battery_voltage_mv;
        int16_t shunt_voltage_mv;

        uint8_t core_temperature;
        uint8_t flags;
        uint16_t reserved1;

        int16_t imu_temp_accel_gyro[7];
        uint16_t reserved2;
        };
    """


class GloveParser:

    def reset(self):
        self.buffer = np.zeros([self.width * self.height * 2], dtype=np.int32)
        self.valid = np.zeros([self.width * self.height * 2], dtype=np.int32)
        self.prev_index = 0

    def __init__(self):

        self.thermotable = [
            -58, -56, -54, -52, -45, -44, -43, -42, -41, -40, -39, -38, -37, -36, -30, -20, -10, -
            4, 0, 4, 10, 21, 22, 23, 24, 25, 26, 27, 28, 29, 40, 50, 60, 70, 76, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 116, 120, 124, 128, 132,
        ]

        self.width = 16
        self.height = 16

        self.count = 0

        self.timestamp = False

        self.reset()

    def flush(self):
        mat = None
        if self.timestamp is not False:
            mat = mittenwire.msg.ImpedanceMatrix()
            mat.width = self.width
            mat.height = self.height
            mat.inphase = np.copy(self.buffer[0::2])
            mat.quadrature = np.copy(self.buffer[1::2])
            mat.validity = np.copy(self.valid[0::2])
            mat.timestamp = self.timestamp
        self.reset()
        self.buffer *= 0
        self.valid *= 0
        self.count = 0
        self.timestamp = False
        return mat

    def parse(self, message_data):

        ret = None

        timestamp, magic = struct.unpack_from("<II", message_data)

        if magic != 0x79A90F58:
            return ret

        message_data = message_data[8:]

        id = struct.unpack_from("<I", message_data)[0]

        if (id & 0xffffff00) == 0x1e662200 and len(message_data) == 32:
            words = np.frombuffer(message_data, dtype=np.uint32, count=8)
            index = (words[0] & 0xff)

            if self.timestamp is False:
                self.timestamp = timestamp

            if index < self.prev_index:
                ret = self.flush()

            for k in range(7):
                if index * 14 + k * 2 + 1 < len(self.buffer):
                    i, q = mittenwire.unpack_sample(words[k + 1])
                    self.buffer[index * 14 + k * 2 + 0] = i
                    self.buffer[index * 14 + k * 2 + 1] = q
                    self.valid[index * 14 + k * 2 + 0] = 1
                    self.valid[index * 14 + k * 2 + 1] = 1
            self.prev_index = index
            self.count += 1

            if index == (256 // 7):
                ret = self.flush()

        if id == 0x8da0cdef and len(message_data) == GlovePacket.size:

            packet = GlovePacket()
            packet.unpack(message_data)

            status = mittenwire.msg.GloveStatus()

            status.timestamp = timestamp

            status.clock_milliseconds = packet.millis

            temp = self.thermotable[packet.core_temperature & 0b111111]
            status.core_temperature = temp

            status.button_pressed = ((packet.flags & 1) != 0)
            status.battery_disconnect = ((packet.flags & 2) == 0)
            status.battery_charging = ((packet.flags & 4) != 0)
            status.usb_power_present = ((packet.flags & 8) != 0)

            status.battery_voltage = packet.battery_voltage_mv * 0.001
            status.battery_current = packet.shunt_voltage_mv * 0.001 * 10

            status.imu_temperature = (
                packet.imu_temp_accel_gyro[0] / 132.48) + 25

            ascale = 16.0 / 32767.0
            status.imu_linear_acceleration.x = packet.imu_temp_accel_gyro[1] * ascale
            status.imu_linear_acceleration.y = packet.imu_temp_accel_gyro[2] * ascale
            status.imu_linear_acceleration.z = packet.imu_temp_accel_gyro[3] * ascale

            rscale = 2000.0 / 32767.0
            status.imu_angular_velocity.x = packet.imu_temp_accel_gyro[4] * rscale
            status.imu_angular_velocity.y = packet.imu_temp_accel_gyro[5] * rscale
            status.imu_angular_velocity.z = packet.imu_temp_accel_gyro[6] * rscale

            ret = status

        return ret


class Glove(mittenwire.Node):

    def __init__(self, matrix_handler, status_handler):

        self.parser = GloveParser()

        def handle_message(message):
            ros_msg = self.parser.parse(bytes(message.data))

            if isinstance(ros_msg,  mittenwire.msg.ImpedanceMatrix):
                ros_msg.channel = message.channel
                matrix_handler(ros_msg)

            if isinstance(ros_msg,  mittenwire.msg.GloveStatus):
                ros_msg.channel = message.channel
                status_handler(ros_msg)

        super().__init__(handle_message)
