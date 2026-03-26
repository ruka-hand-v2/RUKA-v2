import numpy as np
import pybullet as p
import pybullet_data
from ruka_hand.control.hand import Hand
from ruka_hand.utils.trajectory import move_to_pos
from dex_retargeting.retargeting_config import RetargetingConfig

min_deg = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -25, 0], dtype=float)
max_deg = np.array([90, 40, 85, 15, 90, 85, 70, 20, 90, 80, 90, 90, 145, 90, 25, 60], dtype=float)

def angle_between(v1, v2):
    v1 = np.array(v1)
    v2 = np.array(v2)
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
    return np.arccos(dot)

def apply_offset(pos, orn, offset):
    rot_matrix = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    world_offset = rot_matrix.dot(offset)
    return pos + world_offset

class DexRukav2Handler:
    def __init__(self, urdf_path="/Users/sissi/Downloads/RUKA/assets/robot.urdf", hand_type="left"):
        self.hand_type = hand_type
        self.hand = Hand(hand_type)
        # self.reset()
        self.initial_wrist_axis = None
        self.initial_palm_normal = None
        self.initial_horiz = None
        self.set_up_dex(urdf_path)

    def set_up_dex(self, urdf_path):
        config_path = "/Users/sissi/Downloads/RUKA/assets/dex_retarget.yml"
        self.retargeting = RetargetingConfig.load_from_file(config_path).build()
        self.retargeting.optimizer.opt.set_maxeval(15)
        self.retargeting.optimizer.opt.set_xtol_rel(0)
        self.target_joint_names = self.retargeting.optimizer.robot.dof_joint_names

        client_id = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        robot_id = p.loadURDF(urdf_path, useFixedBase=True)
        mcp_offsets = {
            "wrist": [0, 0, -0.015],
            "index": [0, 0, -0.008],
            "pinky": [0, 0, 0.006]
        }
        
        state = p.getLinkState(robot_id, 1)
        link_pos = state[4]
        link_orn = state[5]
        self.wrist_pos = apply_offset(link_pos, link_orn, mcp_offsets["wrist"])
        state = p.getLinkState(robot_id, 2)
        link_pos = state[4]
        link_orn = state[5]
        index_mcp_pos = apply_offset(link_pos, link_orn, mcp_offsets["index"])
        state = p.getLinkState(robot_id, 13)
        link_pos = state[4]
        link_orn = state[5]
        pinky_mcp_pos = apply_offset(link_pos, link_orn, mcp_offsets["pinky"])
        self.hand_width = (0 * np.linalg.norm(pinky_mcp_pos - self.wrist_pos) + 2.5 * np.linalg.norm(index_mcp_pos - self.wrist_pos)) / 3
        # self.hand_width = np.linalg.norm(pinky_mcp_pos - self.wrist_pos)

        init_x_axis = index_mcp_pos - self.wrist_pos 
        init_z_axis = np.cross(index_mcp_pos - self.wrist_pos, pinky_mcp_pos - self.wrist_pos) 
        init_y_axis = np.cross(init_z_axis, init_x_axis)
        init_x_axis = init_x_axis / np.linalg.norm(init_x_axis)
        init_y_axis = init_y_axis / np.linalg.norm(init_y_axis)
        init_z_axis = init_z_axis / np.linalg.norm(init_z_axis)
        self.R_robot = np.stack([init_x_axis, init_y_axis, init_z_axis], axis=1)

    def get_wrist_angles(self, horiz, wrist_axis, palm_normal):
        if self.initial_wrist_axis is None:
            self.initial_wrist_axis = wrist_axis
            self.initial_palm_normal = palm_normal
            self.initial_horiz = horiz

        v1 = wrist_axis
        v2 = self.initial_wrist_axis
        v1_proj = v1 - np.dot(v1, self.initial_palm_normal) * self.initial_palm_normal
        v2_proj = v2 - np.dot(v2, self.initial_palm_normal) * self.initial_palm_normal
        yaw = angle_between(v1_proj, v2_proj)
        if np.dot(np.cross(v2_proj, v1_proj), self.initial_palm_normal) > 0: 
            yaw = -yaw
        
        v1_proj = v1 - np.dot(v1, self.initial_horiz) * self.initial_horiz
        v2_proj = v2 - np.dot(v2, self.initial_horiz) * self.initial_horiz
        pitch = angle_between(v1_proj, v2_proj)
        if np.dot(np.cross(v2_proj, v1_proj), self.initial_horiz) < 0:
            pitch = 0
        return yaw, pitch
    
    def to_robot_frame(self, positions, x_axis, y_axis, palm_normal, norm_len):
        R_hand = np.stack([x_axis, y_axis, palm_normal], axis=1)  # columns = hand axes
        scale_factor = self.hand_width / norm_len
        rel_hand_frame = scale_factor * (positions @ R_hand  @ self.R_robot.T) + self.wrist_pos
        return rel_hand_frame

    def get_finger_angles(self, target_points):
        indices = self.retargeting.optimizer.target_link_human_indices
        origin_indices = indices[0, :]
        task_indices = indices[1, :]
        ref_value = target_points[task_indices, :] - target_points[origin_indices, :]

        lower_limits = self.retargeting.optimizer.robot.model.lowerPositionLimit
        robot_q = self.retargeting.retarget(ref_value) - lower_limits
        joint_names = self.retargeting.optimizer.robot.dof_joint_names
        return dict(zip(joint_names, robot_q))

    def points_to_joint_angles(self, points):
        angles = np.zeros(16)
        # wrist = points[0][0]
        # index_mcp = points[1][1]
        # middle_mcp = points[2][1]
        # pinky_mcp = points[4][1]
        # thumb_tip = points[0][4]
        # index_tip = points[1][4]
        # middle_tip = points[2][4]
        # ring_tip = points[3][4]
        # pinky_tip = points[4][4]
        points = points - points[0]
        wrist = points[0]
        index_mcp = points[5]
        middle_mcp = points[9]
        pinky_mcp = points[17]
        horiz = index_mcp - pinky_mcp
        horiz = horiz / np.linalg.norm(horiz)
        wrist_axis = middle_mcp - wrist
        wrist_axis = wrist_axis / np.linalg.norm(wrist_axis)
        palm_normal = np.cross(index_mcp - wrist, pinky_mcp - wrist)
        palm_normal = palm_normal / np.linalg.norm(palm_normal)
        angles[14], angles[15] = self.get_wrist_angles(horiz, wrist_axis, palm_normal)

        x_axis = index_mcp - wrist
        y_axis = np.cross(palm_normal, x_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        norm_len = (2.5 * np.linalg.norm(index_mcp - wrist) + 0 * np.linalg.norm(pinky_mcp - wrist)) / 3
        # norm_len = np.linalg.norm(pinky_mcp - wrist)
        transformed_fingertips = self.to_robot_frame(points, x_axis, y_axis, palm_normal, norm_len)
        finger_deg = self.get_finger_angles(transformed_fingertips)
        
        angles[8] = finger_deg['index_splay']
        angles[6] = finger_deg['index_mcp'] * 1.7
        # angles[6] = finger_deg['index_mcp'] * 1.7
        angles[7] = (finger_deg['index_pip'] + finger_deg['index_dip']) * 2 / 3
        # angles[6] = 0
        angles[9] = finger_deg['mid_mcp'] * 1.37
        angles[10] = (finger_deg['mid_pip'] + finger_deg['mid_dip']) * 2 / 3
        # angles[9] = 0
        angles[4] = finger_deg['ring_splay']
        angles[5] = finger_deg['ring_mcp'] * 1.3
        angles[3] = (finger_deg['ring_pip'] + finger_deg['ring_dip']) * 1 / 3
        angles[1] = finger_deg['pinky_splay']
        angles[0] = finger_deg['pinky_mcp'] * 1.2
        angles[2] = (finger_deg['pinky_pip'] + finger_deg['pinky_dip']) * 1.5 / 3
        angles[13] = finger_deg['thumb_cmc'] * 1.2
        angles[12] = finger_deg['thumb_mcp'] * 1.8
        angles[11] = finger_deg['thumb_ip'] * 1.3
        return np.degrees(angles)

    def compute_motor_pos(self, test_pos):
        test_pos = np.array(test_pos, dtype=float)
        test_pos[13] = test_pos[13] * 1.17 + 20
        print(test_pos[13])
        test_pos[8] = test_pos[8] * 2 - 20
        test_pos[1] = test_pos[1] * 2 - 20
        test_pos[4] = test_pos[4] * 2 - 20
        # test_pos[3] -= 20
        # print(test_pos[8], test_pos[10], test_pos[4])
        clamped = np.clip(test_pos, min_deg, max_deg)
        normed = clamped / (max_deg - min_deg)
        positions = normed * (self.hand.curled_bound - self.hand.tensioned_pos) + self.hand.tensioned_pos
        positions[1] = 2600 + normed[1] * abs(self.hand.curled_bound[1] - self.hand.tensioned_pos[1])
        positions[4] = 3070 - normed[4] * abs(self.hand.curled_bound[4] - self.hand.tensioned_pos[4])
        positions[8] = 1370 + normed[8] * abs(self.hand.curled_bound[8] - self.hand.tensioned_pos[8])
        positions[14] = 2948 - normed[14] * abs(self.hand.curled_bound[14] - self.hand.tensioned_pos[14])
        return positions

    def get_command(self, points_24):
        joint_angles = self.points_to_joint_angles(points_24)
        motor_positions = self.compute_motor_pos(joint_angles)
        motor_positions = np.clip(motor_positions, np.minimum(self.hand.curled_bound, self.hand.tensioned_pos), np.maximum(self.hand.curled_bound, self.hand.tensioned_pos))
        return motor_positions

    def reset(self):
        motor_positions = self.compute_motor_pos(np.zeros(16))
        curr_pos = self.hand.read_pos()
        move_to_pos(curr_pos=curr_pos, des_pos=motor_positions, hand=self.hand, traj_len=35)

    def close(self):
        self.hand.close()

"""
    FINGER_NAMES_TO_MOTOR_IDS = {
        "Thumb": [12, 13, 11],
        "Index": [7, 8, 6],
        "Middle": [10, 9],
        "Ring": [3, 4, 5],
        "Pinky": [1, 0, 2],
        "Wrist": [14, 15]
    }
"""