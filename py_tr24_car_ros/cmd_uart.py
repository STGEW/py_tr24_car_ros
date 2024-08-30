import ctypes
import serial

import rclpy
from rclpy.node import Node

from tr24_ros_interfaces.msg import CarEnginesState
from std_msgs.msg import String


serial_port = '/dev/ttyUSB0'
baud_rate = 460800
timeout = 0
uart_read_buff_size = 256
text_done = 'done'
text_abort = 'abort'
text_ok = 'ok'

resps = [text_done, text_ok, text_abort]


class Point2D(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float),
                ("y", ctypes.c_float)]
    _pack_ = 1  # Ensure the struct is packed (no padding between fields)


class CmdUart(Node):

    def __init__(self):
        super().__init__('cmd_uart')

        self._uart_buff = [''] * 6

        self._ser = serial.Serial(
            serial_port, baud_rate, timeout=timeout)

        # a response from car (done/abort/ok)
        self._pub_resp_from_car = self.create_publisher(
            String,
            'resp_from_car',
            10)

        # new coords for car
        self._sub_car_new_coords = self.create_subscription(
            CarEnginesState,
            'car_new_coords',
            self.car_new_coords_cb,
            10)

        # stop cmd
        self._sub_car_stop = self.create_subscription(
            String,
            'car_stop_cmd',
            self.car_stop_cb,
            10)

        # timer to read from uart
        uart_timer_period = 0.05 # 20 Hz

        self._uart_timer = self.create_timer(
            uart_timer_period,
            self.uart_read_routine)
        self.i = 0

    def uart_read_routine(self):
        # self.get_logger().info(
        #     f"UART ROUTINE")
        data = self._ser.read(uart_read_buff_size)
        if data:
            data = data.decode('utf-8', errors='ignore')
            self.get_logger().info(
                f"Received: '{data}'")
            for s in data:
                self._uart_buff = self._uart_buff[1:] + [s]
                buff = ''.join(self._uart_buff)
                # self.get_logger().info(
                #     f"buff: '{buff}'")
                for r in resps:
                    if buff[-len(r):] == r:
                        self._pub_text(r)
                        break

    def car_new_coords_cb(self, msg):
        self.get_logger().info(
            f'New coordinates for car. '
            f'Left: {msg.left} right: {msg.right}')

        point = Point2D(
            x=float(msg.left),
            y=float(msg.right))

        # Convert the struct to bytes
        point_bytes = ctypes.string_at(
            ctypes.byref(point), ctypes.sizeof(point))
        self._ser.write("LF".encode())
        size_of_point = ctypes.sizeof(point)
        self._ser.write(ctypes.c_ubyte(
            size_of_point).value.to_bytes(1, byteorder='little'))
        self._ser.write(point_bytes)

    def car_stop_cb(self, msg):
        self.get_logger().info(f'New stop cmd for car')
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
