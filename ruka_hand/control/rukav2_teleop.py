import numpy as np
from ruka_hand.control.hand import Hand
from ruka_hand.utils.trajectory import move_to_pos

def angle_between(v1, v2):
    v1 = np.array(v1)
    v2 = np.array(v2)
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
    return np.arccos(dot)


min_deg = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -25, 0], dtype=float)
max_deg = np.array([90, 40, 85, 15, 90, 85, 70, 20, 90, 80, 90, 90, 145, 90, 25, 60], dtype=float)

class RUKAv2Handler:
    def __init__(self, hand_type="right"):
        self.hand_type = hand_type
        self.hand = Hand(hand_type)
        self.initial_wrist_axis = None
        self.initial_palm_normal = None
        self.initial_horiz = None

    def compute_joint_angles(self, points):
        """
        points: np.array shape (24,3)
        Returns: joint angles array (16,)
        """

        wrist = points[0][0]
        index_mcp = points[1][1]
        pinky_mcp = points[4][1]
        middle_mcp = points[2][1]
        ring_mcp = points[3][1]
        thumb_cmc = points[0][1]
        thumb_mcp = points[0][2]
        thumb_ip  = points[0][3]
        thumb_tip = points[0][4]
        horiz = index_mcp - ring_mcp 
        horiz = horiz / np.linalg.norm(horiz)
        palm_normal = np.cross(index_mcp - wrist, pinky_mcp - wrist)
        palm_normal = palm_normal / np.linalg.norm(palm_normal)

        wrist_axis = (ring_mcp + middle_mcp) / 2 - wrist
        wrist_axis = wrist_axis / np.linalg.norm(wrist_axis)
        v1 = wrist_axis
        v2 = thumb_ip - thumb_mcp
        v1_proj = v1 - np.dot(v1, palm_normal) * palm_normal
        v2_proj = v2 - np.dot(v2, palm_normal) * palm_normal
        thumb_ip_flex = 90 - np.degrees(angle_between(v1_proj, v2_proj))

        v1 = index_mcp - pinky_mcp
        v1_proj = v1 - np.dot(v1, wrist_axis) * wrist_axis
        v2_proj = v2 - np.dot(v2, wrist_axis) * wrist_axis
        thumb_mcp_flex = np.degrees(angle_between(v1_proj, v2_proj))

        v1 = thumb_ip - thumb_mcp
        v2 = thumb_tip - thumb_ip 
        thumb_dip_flex = np.degrees(angle_between(v1, v2))

        angles = {
            "thumb": {
                "mcp": thumb_mcp_flex,
                "ip": thumb_ip_flex,
                "dip": thumb_dip_flex
            }
        }
        finger_joints = {
            "index": 1,
            "middle": 2,
            "ring": 3,
            "pinky": 4,
        }

        for finger, id in finger_joints.items():
            finger_flexion = []
            mcp = points[id][1]
            pip = points[id][2]
            v1 = points[2][2] - points[2][1]
            v2 = pip - mcp
            v1_proj = v1 - np.dot(v1, palm_normal) * palm_normal
            v2_proj = v2 - np.dot(v2, palm_normal) * palm_normal
            sideways_angle = np.degrees(angle_between(v1_proj, v2_proj))

            v1 = mcp - wrist
            finger_flexion.append(np.degrees(angle_between(v1, v2)))

            v1 = pip - mcp
            v2 = points[id][3] - pip
            finger_flexion.append(np.degrees(angle_between(v1, v2)))

            angles[finger] = {"flexion": finger_flexion, "sideways_mcp": sideways_angle}

        # Save initial baseline if not set yet
        if self.initial_wrist_axis is None:
            self.initial_wrist_axis = wrist_axis
            self.initial_palm_normal = palm_normal
            self.initial_horiz = horiz
            
        # Wrist angles
        v1 = wrist_axis
        v2 = self.initial_wrist_axis
        v1_proj = v1 - np.dot(v1, self.initial_palm_normal) * self.initial_palm_normal
        v2_proj = v2 - np.dot(v2, self.initial_palm_normal) * self.initial_palm_normal
        yaw = np.degrees(angle_between(v1_proj, v2_proj))
        if np.dot(np.cross(v2_proj, v1_proj), self.initial_palm_normal) > 0: # wrist axis -> pinky, angle going left
            yaw = -yaw
        
        v1_proj = v1 - np.dot(v1, self.initial_horiz) * self.initial_horiz
        v2_proj = v2 - np.dot(v2, self.initial_horiz) * self.initial_horiz
        pitch = np.degrees(angle_between(v1_proj, v2_proj))
        if np.dot(np.cross(v2_proj, v1_proj), self.initial_horiz) > 0:
            pitch = 0
        angles["wrist"] = {"yaw": yaw, "pitch": pitch}

        motor_positions = np.zeros(16)
        motor_positions[12] = angles["thumb"]["mcp"]
        motor_positions[13] = angles["thumb"]["ip"]
        motor_positions[11] = angles["thumb"]["dip"]
        motor_positions[7] = angles["index"]["sideways_mcp"]
        motor_positions[8] = angles["index"]["flexion"][0]
        motor_positions[6] = angles["index"]["flexion"][1]
        motor_positions[10] = angles["middle"]["flexion"][0]
        motor_positions[9]  = angles["middle"]["flexion"][1]
        motor_positions[3] = angles["ring"]["sideways_mcp"]
        motor_positions[4] = angles["ring"]["flexion"][0] 
        motor_positions[5] = angles["ring"]["flexion"][1]
        motor_positions[1] = angles["pinky"]["sideways_mcp"]
        motor_positions[0] = angles["pinky"]["flexion"][0]
        motor_positions[2] = angles["pinky"]["flexion"][1]
        motor_positions[14] = angles["wrist"]["yaw"]
        motor_positions[15] = angles["wrist"]["pitch"]

        return motor_positions
    
    def compute_motor_pos(self, test_pos):
        test_pos = np.array(test_pos, dtype=float)
        test_pos[8] = test_pos[8] * (1 + 0.02 * test_pos[8])
        test_pos[10] = test_pos[10] * (1 + 0.02 * test_pos[10])
        test_pos[4] = test_pos[4] * (1 + 0.02 * test_pos[4])
        test_pos[0] = test_pos[0] * (1 + 0.02 * test_pos[0])

        test_pos[6] = test_pos[6] - 0.07 * test_pos[8]
        test_pos[9] = test_pos[9] - 0.07 * test_pos[10]
        test_pos[5] = test_pos[5] - 0.07 * test_pos[4]
        test_pos[2] = test_pos[2] - 0.07 * test_pos[0]

        test_pos[13] = (test_pos[13] - 40) * 1.5 # change thumb underestimation
        test_pos[12] = test_pos[12] * 1.8 # change thumb angle
        
        test_pos[1] = test_pos[1] * 1.7
        test_pos[7] = test_pos[7] * 1.7
        test_pos[3] = test_pos[3] * 1.7
        print(test_pos[13])

        clamped = np.clip(test_pos, min_deg, max_deg)
        normed = clamped / (max_deg - min_deg)
        positions = normed * (self.hand.curled_bound - self.hand.tensioned_pos) + self.hand.tensioned_pos
        positions[1] = 2285 + normed[1] * abs(self.hand.curled_bound[1] - self.hand.tensioned_pos[1])
        positions[3] = 2070 - normed[3] * abs(self.hand.curled_bound[3] - self.hand.tensioned_pos[3])
        positions[7] = 2125 + normed[7] * abs(self.hand.curled_bound[7] - self.hand.tensioned_pos[7])
        positions[14] = 1990 + normed[14] * abs(self.hand.curled_bound[14] - self.hand.tensioned_pos[14])
        return positions

    def get_command(self, points_24):
        joint_angles = self.compute_joint_angles(points_24)
        motor_positions = self.compute_motor_pos(joint_angles)
        return motor_positions

    def reset(self):
        motor_positions = self.compute_motor_pos(np.zeros(16))
        curr_pos = self.hand.read_pos()
        move_to_pos(curr_pos=curr_pos, des_pos=motor_positions, hand=self.hand, traj_len=35)

    def close(self):
        self.hand.close()
