import serial
import struct
import time

import rclpy
from rclpy.node import Node
from example_interfaces.msg import UInt16MultiArray


class LeArm(Node):

    def __init__(self, port = 'dev/learm',baud = 9600):
        super().__init__('learm_node')
        self.subscription = self.create_subscription(UInt16MultiArray, 'arm_control', self.cmd_angle_callback, 10)
        self.ser = serial.Serial(port, baud, timeout = 1)


    def map_angle_to_pwm(self, value):
        in_min, in_max, out_min, out_max = 0, 180, 500, 2500
        return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def forced_stop(self):
        set_servo_cmd = bytearray(b'\x55\x55\x04\x07\x00')
        self.ser.write(set_servo_cmd)

    def move_servo_angle(self, mt, s1, s2, s3, s4, s5, s6):
        s1 = self.map_angle_to_pwm(s1)
        s2 = self.map_angle_to_pwm(s2)
        s3 = self.map_angle_to_pwm(s3)
        s4 = self.map_angle_to_pwm(s4)
        s5 = self.map_angle_to_pwm(s5)
        s6 = self.map_angle_to_pwm(s6)
        set_servo_cmd = bytearray(b'\x55\x55\x17\x03\x06')
        set_servo_cmd += struct.pack('>BB', mt & 0xFF, (mt >> 8) & 0xFF)
        set_servo_cmd += bytearray(b'\x01')
        set_servo_cmd += struct.pack('>BB', s1 & 0xFF, (s1 >> 8) & 0xFF)
        set_servo_cmd += bytearray(b'\x02')
        set_servo_cmd += struct.pack('>BB', s2 & 0xFF, (s2 >> 8) & 0xFF)
        set_servo_cmd += bytearray(b'\x03')
        set_servo_cmd += struct.pack('>BB', s3 & 0xFF, (s3 >> 8) & 0xFF)
        set_servo_cmd += bytearray(b'\x04')
        set_servo_cmd += struct.pack('>BB', s4 & 0xFF, (s4 >> 8) & 0xFF)
        set_servo_cmd += bytearray(b'\x05')
        set_servo_cmd += struct.pack('>BB', s5 & 0xFF, (s5 >> 8) & 0xFF)
        set_servo_cmd += bytearray(b'\x06')
        set_servo_cmd += struct.pack('>BB', s6 & 0xFF, (s6 >> 8) & 0xFF)
        self.ser.write(set_servo_cmd)

    def move_servo_pwm(self, mt, s1, s2, s3, s4, s5, s6):
        set_servo_cmd = bytearray(b'\x55\x55\x17\x03\x06')
        set_servo_cmd += struct.pack('>BB', mt & 0xFF, (mt >> 8) & 0xFF)
        set_servo_cmd += bytearray(b'\x01')
        set_servo_cmd += struct.pack('>BB', s1 & 0xFF, (s1 >> 8) & 0xFF)
        set_servo_cmd += bytearray(b'\x02')
        set_servo_cmd += struct.pack('>BB', s2 & 0xFF, (s2 >> 8) & 0xFF)
        set_servo_cmd += bytearray(b'\x03')
        set_servo_cmd += struct.pack('>BB', s3 & 0xFF, (s3 >> 8) & 0xFF)
        set_servo_cmd += bytearray(b'\x04')
        set_servo_cmd += struct.pack('>BB', s4 & 0xFF, (s4 >> 8) & 0xFF)
        set_servo_cmd += bytearray(b'\x05')
        set_servo_cmd += struct.pack('>BB', s5 & 0xFF, (s5 >> 8) & 0xFF)
        set_servo_cmd += bytearray(b'\x06')
        set_servo_cmd += struct.pack('>BB', s6 & 0xFF, (s6 >> 8) & 0xFF)
        self.ser.write(set_servo_cmd)

    def servo_reset_home(self):
        set_servo_cmd = bytearray(b'\x55\x55\x17\x03\x06\xe8\x03\x01\xdc\x05\x02\xdc\x05\x03\xdc\x05\x04\xdc\x05\x05\xdc\x05\x06\xdc\x05')
        self.ser.write(set_servo_cmd)


    def cmd_angle_callback(self, msg):
        sd, s1, s2, s3, s4, s5, s6 = msg.data
        if s1 >=500:
            self.move_servo_pwm(sd, s1, s2, s3, s4, s5, s6)
        else:
            self.move_servo_angle(sd, s1, s2, s3, s4, s5, s6)



def main(args=None):
    rclpy.init(args=args)
    node = LeArm(port = '/dev/ttyCH341USB0',baud = 9600)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
