# Base driver for STorM32 Gimbal controller for McGill Robotics
# Implement serial communication
#
# Author: Bei Chen Liu (bei.liu@mail.mcgill.ca)
# Version 0.1

import serial
from time import time, sleep
import struct
from threading import Lock


def _BV(b):
    """Quick function to get bit value.

    Args:
        b: Bit to shift to.

    Returns: Shifted value.
    """
    return 1 << b


# Exception Definitions
class ACKChecksumError(Exception):

    def __init__(self, crc_cal, crc_rec):
        self.crc_cal = crc_cal
        self.crc_rec = crc_rec

    def __str__(self):
        return "CRC calculated: {0:s}, CRC received: {1:s}".format(
            self.crc_cal, self.crc_rec)


class ACKFailError(Exception):

    def __str__(self):
        return "Received ACK but action failed!"


class ACKAccessDeniedError(Exception):

    def __str__(self):
        return "Received ACK but access denied!"


class ACKNotSupported(Exception):

    def __str__(self):
        return "Received ACK but action not supported!"


class ACKTimeout(Exception):

    def __str__(self):
        return "Received ACK but action timed out!"


class ACKPayloadLen(Exception):

    def __str__(self):
        return "Received ACK but payload length is wrong!"


class Storm32(object):

    # Header and Command definitions
    _MSG_HEADER_IN = 0xFB
    _MSG_HEADER_OUT = 0xFA
    _CMD_GET_VERSION = 0x01
    _CMD_GET_VR_STR = 0x02
    _CMD_GET_PARAM = 0X03
    _CMD_SET_PARAM = 0X04
    _CMD_GET_DATA = 0X05
    _CMD_GET_DATAFIELDS = 0x06
    _CMD_SET_PITCH = 0x0A
    _CMD_SET_ROLL = 0x0B
    _CMD_SET_YAW = 0x0C
    _CMD_SET_PAN_MODE = 0x0D
    _CMD_SET_STANDBY = 0x0E
    _CMD_DO_CAMERA = 0x0F
    _CMD_SET_SCRIPT_CONTROL = 0x10
    _CMD_SET_ANGLES = 0x11
    _CMD_SET_PITCH_ROLL_YAW = 0x12
    _CMD_SET_PWM_OUT = 0x13
    _CMD_RESTORE_PARAM = 0x14
    _CMD_RESTORE_ALL_PARAM = 0x15
    _CMD_ACTIVE_PAN_MODE_SETTING = 0x64

    _CMD_ACK = 0x96
    _CMD_ACK_OK = 0
    _CMD_ACK_ERR_FAILD = 1
    _CMD_ACK_ERR_ACCESS_DENIED = 2
    _CMD_ACK_ERR_NOT_SUPPORTED = 3
    _CMD_ACK_ERR_TIMEOUT = 150
    _CMD_ACK_ERR_CRC = 151
    _CMD_ACK_ERR_PAYLOAD_LEN = 152

    _CMD_HEADER_IN = 0xFB
    _CMD_HEADER_OUT = 0xFA

    _MSG_READ_TIMEOUT = 0.1

    # Get Status response dictionary
    _STATE_STR = {
        0: "Startup Motor",
        1: "Startup Settle",
        2: "Startup Calibrate",
        3: "Startup Level",
        4: "Startup MotorDir",
        5: "Startup Relevel",
        6: "Normal",
        7: "Standby"
    }

    _STATUS2_STR = {
        _BV(8): "Recenter Pitch",
        _BV(9): "Recenter Roll",
        _BV(10): "Recenter Yaw",
        _BV(11): "IR Camera",
        _BV(12): "Standby",
        _BV(13): "Pan Pitch",
        _BV(14): "Pan Roll",
        _BV(15): "Pan Yaw",
        _BV(3): "Motor Pitch Active",
        _BV(4): "Motor Roll Active",
        _BV(5): "Motor Yaw Active"
    }

    _STATUS_STR = {
        _BV(8): "Storm32Link Present",
        _BV(9): "NT Bus in Use",
        _BV(10): "IMU2 NT Bus",
        _BV(11): "IMU2 High ADR",
        _BV(12): "IMU2 Present",
        _BV(13): "MAG Present",
        _BV(14): "IMU High ADR",
        _BV(15): "IMU Present",
        _BV(0): "Storm32Link in Use",
        _BV(1): "Storm32Link OK",
        _BV(2): "Level Failed",
        _BV(3): "Battery Connected",
        _BV(4): "Bat Voltage Low",
        _BV(5): "IMU OK",
        _BV(6): "IMU2 OK",
        _BV(7): "MAG OK"
    }

    _CAPABILITIES_STR = {
        _BV(15): "Has Battery Voltage",
        _BV(14): "Has Onboard MPU",
        _BV(13): "Has I2C2",
        _BV(12): "Has RC2",
        _BV(11): "Has SPEKTRUM",
        _BV(10): "Has SBUS",
        _BV(9): "Has IR",
        _BV(8): "Has 3AUX",
    }

    _BOARD_STR = {1: "F103CB", 2: "F103RB", 3: "F103RCDE", 4: "F405RG"}

    def __init__(self, baud=115200, port="/dev/ttyACM0"):
        """Object initializer for the gimbal object.
        Args:
            baud: Baudrate for serial port.
            port: Path for the serial port
        """
        # Start the serial object
        self.serial = serial.Serial(port, baud, timeout=0)
        self.serial.flush()
        self.buff = []
        self.lock = Lock()

    def _send_msg(self, cmd, data=[]):
        """Send a message to the gimbal controller.

        Args:
            msg_id: Message command.
            data: Data list, optional.
        returns response of the gimbal controller as an "int" list.
        """
        # Constuct the message
        msg_len = len(data)
        msg = [msg_len, cmd]
        msg.extend(data)

        # Calculating CRC, currently not working, but the gimbal does not check
        # it anyway, so it is OK
        # TODO: Fix CRC calculation
        crc = [0, 0]

        # Insert header and CRC
        msg.insert(0, self._CMD_HEADER_OUT)
        msg.extend(crc)

        # Send the message and get the response
        with self.lock:
            self.serial.write(msg)
            response = self.__get_msg(cmd)
        return response

    def __get_msg(self, cmd):
        """Get the response from the controller.

        Args:
            cmd: Command expected.

        return: response from the gimbal controller as an "int" list, empty
        list of timeout.

        Raise: All ACK Exceptions.
        """
        # Read the serial port and record start time
        self.buff.extend(self.serial.read(100))
        start_time = time()

        # While not timeout yet, try to recover the message
        while (time() - start_time) < self._MSG_READ_TIMEOUT:

            # Try to find a message header
            for i in range(0, len(self.buff) - 3):

                # if header found and command is right, or command is ACK
                if ord(self.buff[i]) == self._MSG_HEADER_IN and (
                    (ord(self.buff[i + 2]) == cmd or
                     (ord(self.buff[i + 2]) == self._CMD_ACK))):

                    # Check if all bytes of the message is here
                    end_of_msg = i + ord(self.buff[i + 1]) + 5
                    if end_of_msg <= len(self.buff):
                        msg = [ord(i) for i in self.buff[i:end_of_msg]]

                        # TODO: Add CRC calculation.

                        # Clean the buffer
                        self.buff = self.buff[end_of_msg:]

                        # Remove header byte and crc bytes
                        msg = msg[1:-2]

                        # If the message is a ACK, check the value and raise
                        # Exception if needed.
                        if msg[1] == self._CMD_ACK:
                            if msg[2] == self._CMD_ACK_OK:
                                return msg
                            if msg[2] == self._CMD_ACK_ERR_ACCESS_DENIED:
                                raise ACKAccessDeniedError()
                            if msg[2] == self._CMD_ACK_ERR_FAILD:
                                raise ACKFailError()
                            if msg[2] == self._CMD_ACK_ERR_PAYLOAD_LEN:
                                raise ACKPayloadLen()
                            if msg[2] == self._CMD_ACK_ERR_TIMEOUT:
                                raise ACKTimeout()
                            if msg[2] == self._CMD_ACK_ERR_NOT_SUPPORTED:
                                raise ACKNotSupported()
                        return msg
            # Try to keep reading
            self.buff.extend(self.serial.read(100))

        # Flush serial and internal buffer on timeout to try to recover.
        self.serial.flush()
        self.buff = []
        return []

    def _float_to_bytes(self, value):
        """Convert a float into a little-endian "int" list of 4 "int".

        Args:
            value: the float to be converted.

        returns: Little-endian "int" list of 4 "int".
        """

        a = list(struct.pack("!f", value))
        return list(map(ord, reversed(a)))

    def _int_to_bytes(self, value):
        """Convert a signed or unsigned 16-bit "int" into a little-endian "int"
        list of 2 "int".

        Args:
            value: 16-bit "int" to be converted.

        returns: Little-endian "int" list of 2 "int"
        """
        return [value & 0xFF, (value >> 8) & 0xFF]

    def _bytes_to_uint(self, byte_low, byte_high):
        """Convert 2 bytes into a unsigned 16-bit int.

        Args:
            byte_low: the lower byte of the int
            byte_hight: the higher byte of the int
        returns: 16-bit unsiged "int"
        """
        return byte_high << 8 | byte_low

    def _bytes_to_int(self, byte_low, byte_high):
        """Convert 2 bytes into a 16-bit int.

        Args:
            byte_low: the lower byte of the int
            byte_hight: the higher byte of the int
        returns: 16-bit signed "int"
        """
        value = self._bytes_to_uint(byte_low, byte_high)
        if value > _BV(15):
            value -= _BV(16)
        return value

    def get_version(self):
        """Get the version information form the gimbal controller.

        returns: Dictionary with all the informations, return empty dictionary
        on timeout.
        """
        msg = self._send_msg(self._CMD_GET_VERSION)
        if msg:
            info = {}
            firm_v = self._bytes_to_uint(msg[2], msg[3]) / 100.0
            layout_v = self._bytes_to_uint(msg[4], msg[5]) / 100.0
            capabilities = self._bytes_to_uint(msg[6], msg[7])
            info["Firmware Version"] = firm_v
            info["Layout Version"] = layout_v
            for k, v in self._CAPABILITIES_STR.items():
                info[v] = bool(k & capabilities)
            for k, v in self._BOARD_STR.items():
                if capabilities & 0xF == k:
                    info["Board"] = v
            return info
        return {}

    def restart_controller(self):
        """Send "xx" to restart the gimbal controller.

        returns: "True" for success, "False" for error.
        """
        with self.lock:
            self.serial.flush()
            self.serial.write("xx")

            # Wait a bit to make sure the response is in
            sleep(0.1)
            s = self.serial.read()
        if s == "o":
            return True
        return False

    def get_data(self):
        """Get all data from the controller.

        returns: data bytes list in "int", refer to Wiki for data format, empty
        list on timeout.
        """
        msg = self._send_msg(self._CMD_GET_DATA, [0])
        return msg

    def get_datafields(self, mask_low, mask_high):
        """Get data from specfic field or fields from the controller.

        Args:
            mask_low: lower bitmask. Refer
            mast_high: higher bitmask.

        Returns: data bytes list in "int", refer to Wiki for data format, empty
        on timeout
        Raises: All ACK Exceptions
        """
        msg = self._send_msg(self._CMD_GET_DATAFIELDS, [mask_low, mask_high])
        return msg

    def get_time(self):
        """ Get time data from gimbal controller.n

        returns: List of 2 "int" in [systicks, cycle_time], empty list on
        timeout.
        """
        msg = self.get_datafields(0x02, 0x00)
        if msg:
            return [msg[4] | msg[5] << 8, msg[6] | msg[7] << 8]
        return [0, 0]

    def set_pitch_roll_yaw(self, pitch=1500, roll=1500, yaw=1500):
        """Set the pitch, roll, yaw of the gimbal controller as RC command.
        Mapped by the RC input settings, Value "1500" is center and value "0"
        is reset. This function will set the angles relative to set_angles
        function, but the reset value will reset the set_angles as well.
        Args:
            pitch: "int" value from "700" to "2300", or 0, optional,
            default "1500".
            roll: "int" value from "700" to "2300", or 0, optional,
            default "1500".
            yaw: "int" value from "700" to "2300", or 0, optional,
            default "1500".

        Returns: ACK_OK message on success, empty list on timeout.

        Raise: ValueError if value is out of bound.
        """
        # Check bounds
        rc_cmd = [pitch, roll, yaw]
        data = []
        for i in rc_cmd:
            if (i > 2300 or i < 700) and i != 0:
                raise ValueError("Value out of bounds!")
            else:
                data.extend(self._int_to_bytes(i))
        return self._send_msg(self._CMD_SET_PITCH_ROLL_YAW, data=data)

    def set_angles(self, pitch=0.0, roll=0.0, yaw=0.0, unlimited=False):
        """Set relative angles of the gimbal from the starting leveled
        position. This function works independently from set_pitch_roll_yaw.

        Args:
            pitch: Pitch angle in float degree, optional, default is "0.0".
            roll: Roll angle in float degree, optional, default is "0.0".
            yaw: Yaw angle in float degree, optional, default is "0.0".
            unlimited: If unlimited is set to "False" the angles will be
            limited by the minimum and maximum settings in RC settings. If
            "True" the angles will not be limited, optional, default "False"

        Returns: ACK_OK message on success, empty list on timeout
        """
        data = self._float_to_bytes(pitch)
        data.extend(self._float_to_bytes(roll))
        data.extend(self._float_to_bytes(yaw))
        data.extend([0x00, 0x00] if unlimited else [0x07, 0x00])
        return self._send_msg(self._CMD_SET_ANGLES, data)

    def get_imu1_angles(self):
        """Get the IMU1 angles from the gimbal controller.

        Returns: [pitch, roll, yaw] as float in degree, returns empty list on
        timeout.
        """
        data = self.get_datafields(0x20, 0x00)
        if data:
            if data[0] == 8 and data[1] == self._CMD_GET_DATAFIELDS:
                angles = [
                    self._bytes_to_int(data[4], data[5]) / 100.0,
                    self._bytes_to_int(data[6], data[7]) / 100.0,
                    self._bytes_to_int(data[8], data[9]) / 100.0
                ]
                return angles
        return []

    def get_imu2_angles(self):
        """Get the IMU2 angles from the gimbal controller.

        Returns: [pitch, roll, yaw] as float in degree, returns empty list on
        timeout.
        """
        data = self.get_datafields(0x00, 0x01)
        if data:
            if data[0] == 8 and data[1] == self._CMD_GET_DATAFIELDS:
                angles = [
                    self._bytes_to_int(data[4], data[5]) / 100.0,
                    self._bytes_to_int(data[6], data[7]) / 100.0,
                    self._bytes_to_int(data[8], data[9]) / 100.0
                ]
                return angles
        return []

    def get_status(self):
        """Get status value from the gimbal controller.

        returns: A dictionary of all parsed values, empty dictionary on
        timeout.
        """
        data = self.get_datafields(0x01, 0x00)
        if data:
            if data[0] == 12 and data[1] == self._CMD_GET_DATAFIELDS:
                result = {}

                state = self._bytes_to_uint(data[4], data[5])
                for k, v in self._STATE_STR.items():
                    if state == k:
                        result["State"] = v

                status = self._bytes_to_uint(data[6], data[7])
                for k, v in self._STATUS_STR.items():
                    result[v] = bool(k & status)

                status2 = self._bytes_to_uint(data[8], data[9])
                for k, v in self._STATUS2_STR.items():
                    result[v] = bool(k & status2)

                i2c_errors = self._bytes_to_uint(data[10], data[11])
                result["I2C Errors"] = i2c_errors

                v_bat = self._bytes_to_uint(data[12], data[13])
                result["VBAT"] = v_bat / 1000.0
                return result

        return {}

    def set_standby(self, state=True):
        """Set the gimbal in or out of standby mode.

        Args:
            state: "True" to set the gimbal to standby mode, "False" to exit
            standby mode, optional, default is "True"

        Returns: ACK_OK on success, empty list on timeout.
        """
        return self._send_msg(self._CMD_SET_STANDBY, [state])
