from arm_control.robot import Robot
from ctypes import *
import time

usleep = lambda x: time.sleep(x / 1000000.0)
ZCAN_DEVICE_TYPE = c_uint32
ZCAN_DEVICE_INDEX = c_uint32
ZCAN_Reserved = c_uint32
ZCAN_CHANNEL = c_uint32
LEN = c_uint32

USBCAN2 = ZCAN_DEVICE_TYPE(4)
DEVICE_INDEX = ZCAN_DEVICE_INDEX(0)
Reserved = ZCAN_Reserved(0)
CHANNEL = ZCAN_CHANNEL(0)


class ZCAN_CAN_INIT_CONFIG(Structure):
    _fields_ = [
        ("AccCode", c_int),
        ("AccMask", c_int),
        ("Reserved", c_int),
        ("Filter", c_ubyte),
        ("Timing0", c_ubyte),
        ("Timing1", c_ubyte),
        ("Mode", c_ubyte),
    ]


class ZCAN_CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint32),
        ("TimeStamp", c_uint32),
        ("TimeFlag", c_uint8),
        ("SendType", c_byte),
        ("RemoteFlag", c_byte),
        ("ExternFlag", c_byte),
        ("DataLen", c_byte),
        ("Data", c_ubyte * 8),
        ("Reserved", c_ubyte * 3),
    ]


class dbUR10(Robot):
    def __init__(self):
        super(dbUR10, self).__init__()
        self.robot_name = "dbUR10"
        self.robot_type = "UR10"
        self.zlg = cdll.LoadLibrary("./libusbcan.so")
        self.init_can()
        self.init_motor()
        self.sleep_duration = 1000  # 1ms
        self.motor_speed = 10000  # 600rpm

        self.CAN_data_index = 0
        self.CAN_data_limit = 100
        self.CAN_data_list = [ZCAN_CAN_OBJ() for i in range(self.CAN_data_limit)]

    def init_can(self):
        # 1. open the can port
        ret = self.zlg.VCI_OpenDevice(USBCAN2, DEVICE_INDEX, Reserved)
        if ret == 0:
            print("Opendevice fail!")
            return -1
        else:
            print("Opendevice success!")
        # 2. init the can port
        init_config = ZCAN_CAN_INIT_CONFIG()
        init_config.AccCode = 0
        init_config.AccMask = 0xFFFFFFFF
        init_config.Reserved = 0
        init_config.Filter = 1
        init_config.Timing0 = 0x00
        init_config.Timing1 = 0x1C
        init_config.Mode = 0
        ret = self.zlg.VCI_InitCAN(USBCAN2, DEVICE_INDEX, 0, byref(init_config))
        if ret == 0:
            print("InitCAN fail!")
            return -1
        else:
            print("InitCAN success!")
        # 3. start the can comm
        ret = self.zlg.VCI_StartCAN(USBCAN2, DEVICE_INDEX, CHANNEL)
        if ret == 0:
            print("StartCAN fail!")
            return -1
        else:
            print("StartCAN success!")

        return 0

    def set_command(self, id: c_int, length: c_int, data: c_long):
        if self.CAN_data_index > self.CAN_data_limit:
            self.CAN_data_index = 0
            print("CAN data num is too large! Reset to 0!")
        self.CAN_data_list[self.CAN_data_index].ID = id
        self.CAN_data_list[self.CAN_data_index].SendType = 1
        self.CAN_data_list[self.CAN_data_index].RemoteFlag = 0
        self.CAN_data_list[self.CAN_data_index].ExternFlag = 0
        self.CAN_data_list[self.CAN_data_index].DataLen = length
        for i in range(length - 1, -1, -1):
            self.CAN_data_list[self.CAN_data_index].Data[i] = data & 0xFF
            data = data >> 8
        self.CAN_data_index += 1

    def send_command(self):
        if self.CAN_data_index == 0:
            print("No data to send!")
            return -1
        ret = self.zlg.VCI_Transmit(
            USBCAN2,
            DEVICE_INDEX,
            CHANNEL,
            byref(self.CAN_data_list),
            self.CAN_data_index,
        )
        if ret == 0:
            print("Transmit fail!")
            return -1
        else:
            print("Transmit success!")
        usleep(self.sleep_duration)
        self.CAN_data_index = 0

    def init_motor(self):
        # 1. reset the motor
        # 2. set the motor mode
        pass
