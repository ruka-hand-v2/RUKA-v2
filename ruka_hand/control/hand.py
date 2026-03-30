import os
from copy import deepcopy as copy

import numpy as np

from ruka_hand.utils.constants import (
    FINGER_NAMES_TO_MOTOR_IDS,
    MOTOR_RANGES_LEFT,
    MOTOR_RANGES_RIGHT,
    USB_PORTS,
)
from ruka_hand.utils.dynamixel_util import *
from ruka_hand.utils.file_ops import get_repo_root


# PID Gains
MCP_D_GAIN = 1000
MCP_I_GAIN = 500
MCP_P_GAIN = 2500

DIP_PIP_D_GAIN = 960
DIP_PIP_I_GAIN = 300
DIP_PIP_P_GAIN = 700


class Hand:
    """Robot Hand class.
    Initializes dynamixel client, sets motor ids and initial motor settings.
    """

    def __init__(self, hand_type="right"):
        if hand_type == "v1_right":
            self.motors = motors = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
            self.DIP_PIP_motors = [1, 2, 3, 5, 8, 10]
            self.MCP_motors = [0, 4, 6, 7, 9]
        else:
            self.motors = motors = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
            self.DIP_PIP_motors = [14, 12, 7, 10, 6, 3]
            self.MCP_motors = [8, 9, 11, 4, 5, 2, 1, 15, 16, 13]
        self.port = USB_PORTS[hand_type]
        print(self.port, hand_type)
        self.dxl_client = DynamixelClient(motors, self.port)
        self.dxl_client.connect()

        self.fingers_dict = FINGER_NAMES_TO_MOTOR_IDS

        # Paramaters for initalization
        self.hand_type = hand_type
        self.curr_lim = 1300
        self.temp_lim = 60
        self.goal_velocity = 400
        self.operating_mode = 5  # 5: current-based position control
        # self.operating_mode = 3  # 5: current-based position control

        repo_root = get_repo_root()
        curled_path = os.path.join(repo_root, "curl_limits", f"{hand_type}_curl_limits.npy")
        tension_path = os.path.join(repo_root, "curl_limits", f"{hand_type}_tension_limits.npy")
        print(curled_path)
        if os.path.exists(curled_path) and os.path.exists(tension_path):
            print(f"Loading saved calibration for {hand_type}...")
            self.curled_bound = np.load(curled_path)
            self.tensioned_pos = np.load(tension_path)
        else:
            if hand_type == "right":
                self.curled_bound = np.array([2609, 2852, 1375, 1811, 2605, 2026, 1511, 1187, 354, 2184, 1969, 506, 2241, 1795, 1617, 1850])
                self.tensioned_pos = np.array([3500,3283, 2511, 1550, 3461, 897, 2590, 1371, 1047, 1400, 962, 930, 2980, 2340, 2417, 1400])
            elif hand_type == "left":
                self.curled_bound = np.array([1572, 2919, 1616, 3013, 3157, 2340, 1034, 1757, 3502, 1159, 511, 1952, 2170, 3380, 3204, 1338])
                self.tensioned_pos = np.array([1040, 2581, 660, 3947, 2973, 2814, 746, 1070, 3364, 1582, 1502, 1800, 1864, 2496, 2601, 1927])
            elif hand_type == "v1_right":
                self.curled_bound = np.array([2416, 3327, 2435, 986, 1248, 2939, 1950, 2808, 2077, 1893, 2230])
                self.tensioned_pos = np.array([3225, 3794, 2682, 1818, 1932, 3815, 2608, 3390, 3059, 2708, 3208])

        self.min_lim = np.minimum(self.tensioned_pos, self.curled_bound)
        self.max_lim = np.maximum(self.tensioned_pos, self.curled_bound) 

        self.init_pos = copy(self.tensioned_pos)
        self._commanded_pos = copy(self.tensioned_pos)

        # Initialization settings of dxl_client
        self.dxl_client.sync_write(
            motors,
            np.ones(len(motors)) * self.operating_mode,
            ADDR_OPERATING_MODE,
            LEN_OPERATING_MODE,
        )  # Set all motors to current-based position control mode
        self.dxl_client.sync_write(
            motors,
            np.ones(len(motors)) * self.temp_lim,
            ADDR_TEMP_LIMIT,
            LEN_TEMP_LIMIT,
        )  # Set Temp limit
        self.dxl_client.sync_write(
            motors,
            np.ones(len(motors)) * self.curr_lim,
            ADDR_CURRENT_LIMIT,
            LEN_CURRENT_LIMIT,
        )  # Set Current limit
        self.dxl_client.sync_write(
            FINGER_NAMES_TO_MOTOR_IDS["Thumb"],
            np.ones(len(motors)) * 700,
            ADDR_CURRENT_LIMIT,
            LEN_CURRENT_LIMIT,
        )  # Set thumb specific current limit
        self.dxl_client.sync_write(
            motors,
            np.ones(len(motors)) * self.goal_velocity,
            ADDR_GOAL_VELOCITY,
            LEN_GOAL_VELOCITY,
        )  # Set Goal Velocity

        # PID Gains for DIP + PIP motors
        self.dxl_client.sync_write(
            self.DIP_PIP_motors,
            np.ones(len(motors)) * DIP_PIP_P_GAIN,
            ADDR_POSITION_P_GAIN,
            LEN_POSITION_P_GAIN,
        )  # Set P gain for DIP and PIP motors
        self.dxl_client.sync_write(
            self.DIP_PIP_motors,
            np.ones(len(motors)) * DIP_PIP_I_GAIN,
            ADDR_POSITION_I_GAIN,
            LEN_POSITION_I_GAIN,
        )  # Set I gain for DIP and PIP motors
        self.dxl_client.sync_write(
            self.DIP_PIP_motors,
            np.ones(len(motors)) * DIP_PIP_D_GAIN,
            ADDR_POSITION_D_GAIN,
            LEN_POSITION_D_GAIN,
        )  # Set D gain for DIP and PIP motors

        # PID Gains for MCP motors
        self.dxl_client.sync_write(
            self.MCP_motors,
            np.ones(len(motors)) * MCP_P_GAIN,
            ADDR_POSITION_P_GAIN,
            LEN_POSITION_P_GAIN,
        )  # Set P gain for MCP motors

        # PID Gains for MCP motors
        self.dxl_client.sync_write(
            [9],
            [2000],
            ADDR_POSITION_P_GAIN,
            LEN_POSITION_P_GAIN,
        )  

        self.dxl_client.sync_write(
            self.MCP_motors,
            np.ones(len(motors)) * MCP_I_GAIN,
            ADDR_POSITION_I_GAIN,
            LEN_POSITION_I_GAIN,
        )  # Set I gain for MCP motors
        self.dxl_client.sync_write(
            self.MCP_motors,
            np.ones(len(motors)) * MCP_D_GAIN,
            ADDR_POSITION_D_GAIN,
            LEN_POSITION_D_GAIN,
        )  # Set D gain for MCP motors

        # Enable Torque
        self.dxl_client.set_torque_enabled(True, -1, 0.05)

        # self.dxl_client.set_pos(self.init_pos)

        self.data_recording_functions = {
            "actual_hand_state": self.get_hand_state,
            "commanded_hand_state": self.get_commanded_hand_state,
        }

    def close(self):
        self.dxl_client.disconnect()

    # read any given address for the given motors
    def read_any(self, addr: int, size: int):
        return self.dxl_client.sync_read(self.motors, addr, size)

    # read position
    def read_pos(self):
        # print(f"in read_pos")
        curr_pos = self.dxl_client.read_pos()
        while curr_pos is None:
            curr_pos = self.dxl_client.read_pos()
            time.sleep(0.001)

        return curr_pos

    # read velocity
    def read_vel(self):
        # print(f"in read_vel")
        return self.dxl_client.read_vel()

    # read current
    def read_cur(self):
        return self.dxl_client.read_cur()

    # set pose
    def set_pos(self, pos):
        self._commanded_pos = pos
        self.dxl_client.set_pos(pos)
        return

    def read_temp(self):
        self.dxl_client.sync_read(self.motors, 146, 1)

    @property
    def commanded_pos(self):
        return self._commanded_pos

    @property
    def actual_pos(self):
        pos = self.read_pos()
        while any(item is None for item in pos):
            pos = self.read_pos()
            time.sleep(0.0001)
        return pos

    def get_hand_state(self):

        motor_state = dict(
            position=np.array(self.actual_pos, dtype=np.float32),
            commanded_position=np.array(self.commanded_pos, dtype=np.float32),
            velocity=np.array(self.read_vel(), dtype=np.float32),
            timestamp=time.time(),
        )
        return motor_state

    def get_commanded_hand_state(self):

        motor_state = dict(
            position=np.array(self.commanded_pos, dtype=np.float32),
            timestamp=time.time(),
        )
        return motor_state

    def read_single_cur(self, motor_id):
        cur = self.dxl_client.read_single_cur(motor_id)
        return cur

if __name__ == "__main__":
    hand= Hand("left")