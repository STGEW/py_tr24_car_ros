import serial.tools.list_ports


def get_uart_path(PID, VID):
    all_ports = serial.tools.list_ports.comports()
    for port in all_ports:
        if port.pid == PID and port.vid == VID:
            return port.device
    else:
        raise ValueError(f"Error! Can't find port with PID: {PID} and VID: {VID}")
