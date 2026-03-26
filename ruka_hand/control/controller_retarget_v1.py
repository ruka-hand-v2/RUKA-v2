import numpy as np
import pybullet as p
import pybullet_data
from ruka_hand.control.hand import Hand
from ruka_hand.utils.trajectory import move_to_pos
from dex_retargeting.retargeting_config import RetargetingConfig
import yaml

min_deg = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=float)
max_deg = np.array([145, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90], dtype=float)

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
    def __init__(self, urdf_path="/Users/sissi/Downloads/RUKA/assets_v1/Hand_Assembly_URDF.urdf", config_path="/Users/sissi/Downloads/RUKA/assets/dex_retarget_v1.yml", hand_type="v1_right"):
        with open(config_path, "r") as f:
            self.config = yaml.safe_load(f)
        self.hand_type = hand_type
        self.hand = Hand(hand_type)
        self.reset()
        self.initial_wrist_axis = None
        self.initial_palm_normal = None
        self.initial_horiz = None
        self.set_up_dex(urdf_path, config_path)

    def set_up_dex(self, urdf_path, config_path):
        self.retargeting = RetargetingConfig.load_from_file(config_path).build()
        self.retargeting.optimizer.opt.set_maxeval(15)
        self.retargeting.optimizer.opt.set_xtol_rel(0)
        self.target_joint_names = self.retargeting.optimizer.robot.dof_joint_names

        client_id = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        robot_id = p.loadURDF(urdf_path, useFixedBase=True)
        mcp_offsets = {
            "wrist": [0, 0, 0],
            "index": [0, 0.008, 0],
            "pinky": [0, 0, 0]
        }
        
        self.wrist_pos = [0, 0, 0]
        state = p.getLinkState(robot_id, 0)
        link_pos = state[4]
        link_orn = state[5]
        index_mcp_pos = apply_offset(link_pos, link_orn, mcp_offsets["index"])
        state = p.getLinkState(robot_id, 12)
        link_pos = state[4]
        link_orn = state[5]
        pinky_mcp_pos = apply_offset(link_pos, link_orn, mcp_offsets["pinky"])
        self.hand_width = (0 * np.linalg.norm(pinky_mcp_pos - self.wrist_pos) + 2.5 * np.linalg.norm(index_mcp_pos - self.wrist_pos)) / 3

        init_x_axis = index_mcp_pos - self.wrist_pos 
        init_z_axis = np.cross(index_mcp_pos - self.wrist_pos, pinky_mcp_pos - self.wrist_pos) 
        init_y_axis = np.cross(init_z_axis, init_x_axis)
        init_x_axis = init_x_axis / np.linalg.norm(init_x_axis)
        init_y_axis = init_y_axis / np.linalg.norm(init_y_axis)
        init_z_axis = init_z_axis / np.linalg.norm(init_z_axis)
        self.R_robot = np.stack([init_x_axis, init_y_axis, init_z_axis], axis=1)
    
    def to_robot_frame(self, positions, x_axis, y_axis, palm_normal, norm_len):
        R_hand = np.stack([x_axis, y_axis, palm_normal], axis=1)  # columns = hand axes
        scale_factor = self.hand_width / norm_len
        rel_hand_frame = scale_factor * (positions @ R_hand @ self.R_robot.T) + self.wrist_pos
        return rel_hand_frame

    def get_finger_angles(self, target_points):
        indices = self.config['oculus_indices']
        origin_indices = indices[0] 
        task_indices = indices[1]
        ref_vectors = []
        for i in range(len(origin_indices)):
            f_start, j_start = origin_indices[i]
            f_end, j_end = task_indices[i]
            p_start = np.array(target_points[f_start][j_start])
            p_end = np.array(target_points[f_end][j_end])
            ref_vectors.append(p_end - p_start)
        ref_value = np.array(ref_vectors)

        lower_limits = self.retargeting.optimizer.robot.model.lowerPositionLimit
        robot_q = self.retargeting.retarget(ref_value) - lower_limits
        joint_names = self.retargeting.optimizer.robot.dof_joint_names
        return dict(zip(joint_names, robot_q))

    def points_to_joint_angles(self, points):
        angles = np.zeros(11)
        points[:, :, 1] *= -1
        points = points - points[0][0]
        wrist = points[0][0]
        index_mcp = points[1][1]
        middle_mcp = points[2][1]
        pinky_mcp = points[4][1]
        horiz = index_mcp - pinky_mcp
        horiz = horiz / np.linalg.norm(horiz)
        wrist_axis = middle_mcp - wrist
        wrist_axis = wrist_axis / np.linalg.norm(wrist_axis)
        palm_normal = np.cross(index_mcp - wrist, pinky_mcp - wrist)
        palm_normal = palm_normal / np.linalg.norm(palm_normal)

        x_axis = index_mcp - wrist
        y_axis = np.cross(palm_normal, x_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        norm_len = (1 * np.linalg.norm(index_mcp - wrist) + 0 * np.linalg.norm(pinky_mcp - wrist)) / 3
        transformed_fingertips = self.to_robot_frame(points, x_axis, y_axis, palm_normal, norm_len)
        finger_deg = self.get_finger_angles(transformed_fingertips)
        
        angles[4] = finger_deg['Index_MCP_Joint']
        angles[3] = (finger_deg['Index_PIP_Joint'] + finger_deg['Index_DIP_Joint']) * 1.7 / 3
        angles[6] = finger_deg['Middle_MCP_Joint']
        angles[5] = (finger_deg['Middle_PIP_Joint'] + finger_deg['Middle_DIP_Joint']) * 1.7 / 3
        angles[7] = finger_deg['Ring_MCP_Joint']
        angles[8] = (finger_deg['Ring_PIP_Joint'] + finger_deg['Ring_DIP_Joint']) * 1.7 / 3
        angles[9] = finger_deg['Pinky_MCP_Joint']
        angles[10] = (finger_deg['Pinky_PIP_Joint'] + finger_deg['Pinky_DIP_Joint']) * 1.7 / 3
        angles[0] = finger_deg['Thumb_CMC_Joint'] * 1.2
        angles[1] = finger_deg['Thumb_MCP_Joint'] * 1.2
        angles[2] = finger_deg['Thumb_IP_Joint'] * 1.2
        return np.degrees(angles)

    def compute_motor_pos(self, test_pos):
        test_pos = np.array(test_pos, dtype=float)
        clamped = np.clip(test_pos, min_deg, max_deg)
        normed = clamped / (max_deg - min_deg)
        positions = normed * (self.hand.curled_bound - self.hand.tensioned_pos) + self.hand.tensioned_pos
        return positions

    def get_command(self, points_24):
        joint_angles = self.points_to_joint_angles(points_24)
        motor_positions = self.compute_motor_pos(joint_angles)
        motor_positions = np.clip(motor_positions, np.minimum(self.hand.curled_bound, self.hand.tensioned_pos), np.maximum(self.hand.curled_bound, self.hand.tensioned_pos))
        return motor_positions

    def reset(self):
        motor_positions = self.compute_motor_pos(np.zeros(11))
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