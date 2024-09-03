from enum import Enum
import ctypes


class Point2D_C(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float),
                ("y", ctypes.c_float)]
    _pack_ = 1  # Ensure the struct is packed (no padding between fields)


class Vehicle2DPosition_C(ctypes.Structure):
    _fields_ = [("p", Point2D_C),
                ("phi", ctypes.c_float)]
    _pack_ = 1  # Ensure the struct is packed (no padding between fields)


class State(Enum):
    EXP_HEADER = 1
    CMD = 2
    VEH_POS = 3


class Parser:

    TEXT_DONE = 'done'
    TEXT_ABORT = 'abort'
    TEXT_OK = 'ok'
    TEXTS = [TEXT_DONE, TEXT_ABORT, TEXT_OK]
    CMD_HEADER = "CM"
    VEH_POS_HEADER = "PV"
    CMD_MAX_SIZE = max(
        len(TEXT_DONE), len(TEXT_ABORT), len(TEXT_OK))
    PV_SIZE = ctypes.sizeof(Vehicle2DPosition_C)


    def __init__(self, cmd_cb, veh_pos_cb, log):
        self._cmd_cb = cmd_cb
        self._veh_pos_cb = veh_pos_cb
        self._log = log
        self._state = State.EXP_HEADER
        self._pref_buff = ['X', 'X']
        self._cmd_buff = [''] * self.CMD_MAX_SIZE
        self._veh_pos_buff = bytearray(self.PV_SIZE)
        self._size = 0  # To keep track of the number of bytes read


    def write(self, data):
        for s in data:
            s = chr(s)
            # self._log.info(f"Parser write process sym: '{s}'")
            if self._state == State.EXP_HEADER:
                # s = s.decode('utf-8', errors='ignore')
                self._pref_buff[0] = self._pref_buff[1]
                self._pref_buff[1] = s
                pref = ''.join(self._pref_buff)
                if pref == self.CMD_HEADER:
                    self._state = State.CMD
                elif pref == self.VEH_POS_HEADER:
                    self._state = State.VEH_POS
                    self._size = 0

            elif self._state == State.CMD:
                # s = s.decode('utf-8', errors='ignore')
                self._cmd_buff.pop(0)  # Remove the oldest character
                self._cmd_buff.append(s)
                buff = ''.join(self._cmd_buff)
                for t in self.TEXTS:
                    if buff.endswith(t):
                        self._cmd_cb(t)
                        self._state = State.EXP_HEADER

            elif self._state == State.VEH_POS:
                if self._size < self.PV_SIZE:
                    # Append byte to buffer
                    self._veh_pos_buff[self._size] = ord(s) 
                    self._size += 1

                if self._size == self.PV_SIZE:
                    # Parse Vehicle2DPosition from buffer
                    veh_pos = Vehicle2DPosition_C.from_buffer_copy(
                        self._veh_pos_buff)
                    self._veh_pos_cb(veh_pos)
                    self._state = State.EXP_HEADER  # Reset state
