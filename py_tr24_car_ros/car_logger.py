import serial
import logging
from logging.handlers import RotatingFileHandler
import os

import rclpy
from rclpy.node import Node


serial_port = '/dev/ttyUSB1'
baud_rate = 460800
timeout = 0

log_directory = '/home/afomin/logs'
if not os.path.exists(log_directory):
    os.makedirs(log_directory)

# Configure the logging system
logging.basicConfig(
    level=logging.INFO,  # Log everything from DEBUG level and above
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        RotatingFileHandler(
            os.path.join(
                log_directory, 'application.log'),
            maxBytes=30 * 1024 * 1024,  # Max size 30 MB per file
            backupCount=10  # Keep up to 10 log files
        )
    ]
)


class CarLogger(Node):

    def __init__(self):
        super().__init__('car_logger')

        self._ser = serial.Serial(
            serial_port, baud_rate, timeout=timeout)

        # timer to read from uart
        uart_logger_timer_period = 0.05 # 20 Hz

        self._uart_timer = self.create_timer(
            uart_logger_timer_period,
            self.uart_read_routine)
        self.buffer = ""


    def uart_read_routine(self):
        data = self._ser.read(256)
        if data:
            try:
                # Decode the incoming bytes to a string
                decoded_data = data.decode('utf-8', errors='replace')

                # Append the decoded data to the buffer
                self.buffer += decoded_data

                # Check if the buffer contains a newline character
                if '\r' in self.buffer:
                    # Split the buffer into lines
                    lines = self.buffer.split('\r')

                    # Log all complete lines
                    for line in lines[:-1]:
                        logging.info(line.strip())  # Strip removes any extra whitespace

                    # Keep the remaining partial line in the buffer
                    self.buffer = lines[-1]
                elif len(self.buffer) > 256:
                    logging.info(self.buffer)
                    self.buffer = ""

            except UnicodeDecodeError as e:
                # If decoding fails, log the raw byte data and the error
                logging.error(f"Decoding failed: {e}")
                logging.info(data)

def main(args=None):
    rclpy.init(args=args)

    car_logger = CarLogger()

    rclpy.spin(car_logger)

    car_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
