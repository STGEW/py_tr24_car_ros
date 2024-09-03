import ctypes
import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from py_tr24_car_ros.parser import Parser
from py_tr24_car_ros.parser import Point2D_C, Vehicle2DPosition_C

from tr24_ros_interfaces.msg import Point2D, Position2D


serial_port = '/dev/ttyUSB0'
baud_rate = 460800
timeout = 0


class CmdUart(Node):

    def __init__(self):
        super().__init__('cmd_uart')

        self._p = Parser(
            self.from_car_cmd_cb, 
            self.from_car_new_pos_cb,
            self.get_logger())

        self._ser = serial.Serial(
            serial_port, baud_rate, timeout=timeout)

        # FROM CAR
        self._pub_from_car_cmd = self.create_publisher(
            String, 'from_car_cmd', 10)
        self._pub_from_car_pos = self.create_publisher(
            Position2D, 'from_car_pos', 20)

        # TO CAR
        self._sub_to_car_new_coord = self.create_subscription(
            Point2D, 'to_car_new_coords',
            self.to_car_new_coord_cb, 10)
        self._sub_to_car_stop = self.create_subscription(
            String, 'to_car_cmd_stop',
            self.to_car_stop_cb, 10)

        # timer to read from uart
        uart_timer_period = 0.05 # 20 Hz

        self._uart_timer = self.create_timer(
            uart_timer_period,
            self.uart_read_routine)


    def uart_read_routine(self):
        # self.get_logger().info(
        #     f"UART ROUTINE")
        data = self._ser.read(256)
        if data:
            # self.get_logger().info(
            #     f"UART raw read: '{data}'")
            self._p.write(data)


    def from_car_new_pos_cb(self, veh_pos):
        msg = Position2D()
        msg.x = veh_pos.p.x
        msg.y = veh_pos.p.y
        msg.phi = veh_pos.phi
        self.get_logger().info(
            f'From car new pos will be published; '
            f'x: {msg.x} y: {msg.y} phi: {msg.phi}')
        self._pub_from_car_pos.publish(msg)


    def from_car_cmd_cb(self, cmd):
        msg = String()
        msg.data = cmd
        self.get_logger().info(
            f'From car cmd be published; '
            f'cmd: {msg.data}')
        self._pub_from_car_cmd.publish(msg)


    def to_car_new_coord_cb(self, msg):
        self.get_logger().info(
            f'To car new coord; '
            f'x: {msg.x} y: {msg.y}')

        point = Point2D_C(
            x=float(msg.x),
            y=float(msg.y))

        size_of_point = ctypes.sizeof(point)

        # Convert the struct to bytes
        point_bytes = ctypes.string_at(
            ctypes.addressof(point),
            size_of_point)

        self._ser.write("LF".encode())
        self._ser.write(
            size_of_point.to_bytes(
                1, byteorder='little'))
        self._ser.write(point_bytes)


    def to_car_stop_cb(self, msg):
        self.get_logger().info(
            f'To car new cmd: stop')
        s = ctypes.c_ubyte(4).value.to_bytes(
            1, byteorder='little')
        self._ser.write('TR'.encode())
        self._ser.write(s)
        self._ser.write('stop'.encode())


    def _pub_text(self, text):
        self.get_logger().info(
            f"Publishing the text: '{text}'")
        msg = String()
        msg.data = text
        self._pub_resp_from_car.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    cmd_uart = CmdUart()

    rclpy.spin(cmd_uart)

    cmd_uart.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
