from dynamixel_sdk import *

from teraffe_drive_control_agent.config.dynamixel_config import *


class DynamixelMotor:
    def __init__(
        self,
        name,
        dxl_id,
        port_handler,
        packet_handler,
        pull_current_A=0.15,
        push_current_A=-0.05,
    ):
        self.name = name
        self.id = dxl_id
        self.port = port_handler
        self.packet = packet_handler

        self.pull_current_A = pull_current_A
        self.push_current_A = push_current_A

    def check_result(self, comm_result, dxl_error):
        if comm_result != COMM_SUCCESS:
            return False

        if dxl_error != 0:
            return False

        return True

    def signed_to_unsigned_2byte(self, value):
        if value < 0:
            value += 2 ** 16

        return value

    def signed_to_unsigned_4byte(self, value):
        if value < 0:
            value += 2 ** 32

        return value

    def unsigned_to_signed_2byte(self, value):
        if value >= 2 ** 15:
            value -= 2 ** 16

        return value

    def unsigned_to_signed_4byte(self, value):
        if value >= 2 ** 31:
            value -= 2 ** 32

        return value

    def write_1byte(self, address, value):
        comm_result, dxl_error = self.packet.write1ByteTxRx(
            self.port,
            self.id,
            address,
            value,
        )

        return self.check_result(comm_result, dxl_error)

    def write_2byte(self, address, value):
        comm_result, dxl_error = self.packet.write2ByteTxRx(
            self.port,
            self.id,
            address,
            value,
        )

        return self.check_result(comm_result, dxl_error)

    def write_4byte(self, address, value):
        comm_result, dxl_error = self.packet.write4ByteTxRx(
            self.port,
            self.id,
            address,
            value,
        )

        return self.check_result(comm_result, dxl_error)

    def read_1byte(self, address):
        value, comm_result, dxl_error = self.packet.read1ByteTxRx(
            self.port,
            self.id,
            address,
        )

        if not self.check_result(comm_result, dxl_error):
            return None

        return value

    def read_2byte(self, address):
        value, comm_result, dxl_error = self.packet.read2ByteTxRx(
            self.port,
            self.id,
            address,
        )

        if not self.check_result(comm_result, dxl_error):
            return None

        return value

    def read_4byte(self, address):
        value, comm_result, dxl_error = self.packet.read4ByteTxRx(
            self.port,
            self.id,
            address,
        )

        if not self.check_result(comm_result, dxl_error):
            return None

        return value

    def torque_on(self):
        return self.write_1byte(
            ADDR_TORQUE_ENABLE,
            TORQUE_ENABLE,
        )

    def torque_off(self):
        return self.write_1byte(
            ADDR_TORQUE_ENABLE,
            TORQUE_DISABLE,
        )

    def set_operating_mode(self, mode):
        self.torque_off()

        result = self.write_1byte(
            ADDR_OPERATING_MODE,
            mode,
        )

        self.torque_on()

        return result

    def set_current_control_mode(self):
        return self.set_operating_mode(CURRENT_CONTROL_MODE)

    def set_velocity_control_mode(self):
        return self.set_operating_mode(VELOCITY_CONTROL_MODE)

    def set_position_control_mode(self):
        return self.set_operating_mode(POSITION_CONTROL_MODE)

    def current_A_to_raw(self, current_A):
        return int(current_A * 1000.0 / 2.69)

    def current_raw_to_A(self, current_raw):
        return current_raw * 2.69 / 1000.0

    def velocity_raw_to_rad_s(self, velocity_raw):
        return velocity_raw * PRESENT_VELOCITY_UNIT

    def set_goal_current_A(self, current_A):
        current_raw = self.current_A_to_raw(current_A)
        current_raw = self.signed_to_unsigned_2byte(current_raw)

        return self.write_2byte(
            ADDR_GOAL_CURRENT,
            current_raw,
        )

    def set_goal_velocity_raw(self, velocity_raw):
        velocity_raw = self.signed_to_unsigned_4byte(velocity_raw)

        return self.write_4byte(
            ADDR_GOAL_VELOCITY,
            velocity_raw,
        )

    def pull(self, direction=1):
        return self.set_goal_current_A(
            self.pull_current_A * direction
        )

    def push(self, direction=1):
        return self.set_goal_current_A(
            self.push_current_A * direction
        )

    def assist(self, current_A):
        return self.set_goal_current_A(current_A)

    def stop_assist(self):
        return self.set_goal_current_A(0.0)

    def stop(self):
        self.set_goal_velocity_raw(0)
        self.set_goal_current_A(0.0)
        self.torque_off()

    def read_present_current_raw(self):
        value = self.read_2byte(ADDR_PRESENT_CURRENT)

        if value is None:
            return None

        return self.unsigned_to_signed_2byte(value)

    def read_present_current_A(self):
        current_raw = self.read_present_current_raw()

        if current_raw is None:
            return None

        return self.current_raw_to_A(current_raw)

    def read_present_velocity_raw(self):
        value = self.read_4byte(ADDR_PRESENT_VELOCITY)

        if value is None:
            return None

        return self.unsigned_to_signed_4byte(value)

    def read_present_velocity_rad_s(self):
        velocity_raw = self.read_present_velocity_raw()

        if velocity_raw is None:
            return None

        return self.velocity_raw_to_rad_s(velocity_raw)

    def read_present_voltage(self):
        value = self.read_2byte(ADDR_PRESENT_INPUT_VOLTAGE)

        if value is None:
            return None

        return value * 0.1

    def read_present_temperature(self):
        return self.read_1byte(ADDR_PRESENT_TEMPERATURE)

    def read_hardware_error(self):
        return self.read_1byte(ADDR_HARDWARE_ERROR_STATUS)