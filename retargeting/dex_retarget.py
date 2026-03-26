# import cv2
# import mediapipe as mp
# import numpy as np
# import pybullet as p
# import pybullet_data
# import time
# from dex_retargeting.retargeting_config import RetargetingConfig
# from retargeting.dex_retarget_controller_mp import DexRukav2Handler
# from ruka_hand.control.hand import Hand
# from ruka_hand.utils.trajectory import move_to_pos

# controller = DexRukav2Handler()

# def main():
#     # 3. SETUP MEDIAPIPE
#     mp_hands = mp.solutions.hands
#     mp_draw = mp.solutions.drawing_utils
#     cap = cv2.VideoCapture(1)
#     first_pos = True

#     with mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7) as hands:
#         while cap.isOpened():
#             # p.stepSimulation()
#             ret, frame = cap.read()
#             if not ret: break
            
#             frame = cv2.flip(frame, 1)
#             rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#             results = hands.process(rgb)

#             if results.multi_hand_world_landmarks:
#                 hand_landmarks = results.multi_hand_world_landmarks[0]
                
#                 # Raw Points (21, 3)
#                 points = np.array([[lm.x, lm.y, lm.z] for lm in hand_landmarks.landmark])
#                 command = controller.get_command(points)
#                 command = np.array([
#                     2815.        , 2952.39456761, 1713.66489977, 1756.        ,
#                     2857.        , 1776.45128655, 2195.        , 1776.        ,
#                     1100.81154701, 1343.81727454, 1207.        , 1402.96441607,
#                     3080.25904964, 2357.        , 1650.08004243, 1654.45926902
#                 ])

#                 print(np.array(command))
#                 curr_pos = controller.hand.read_pos()
#                 if first_pos == True:
#                     move_to_pos(curr_pos, command, controller.hand, traj_len=35)
#                     first_pos = False
#                 else: move_to_pos(curr_pos, command, controller.hand, traj_len=10)

#             cv2.imshow("Webcam Feed", frame)
#             if cv2.waitKey(1) & 0xFF == 27: break
#             time.sleep(1./60.)

#     p.disconnect()
#     cap.release()
#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()


import cv2
import mediapipe as mp
import numpy as np
import pybullet as p
import pybullet_data
import time
from dex_retargeting.retargeting_config import RetargetingConfig

def apply_offset(pos, orn, offset):
    rot_matrix = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    world_offset = rot_matrix.dot(offset)
    return pos + world_offset

def main():
    # 1. SETUP PYBULLET
    client_id = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

    # This is the line that actually changes the background color
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, rgbBackground=[1, 1, 1])
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0) 
    robot_id = p.loadURDF("/Users/sissi/Downloads/RUKA/assets/robot.urdf", useFixedBase=True)

    # Map Joints
    joint_map = {}
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        if info[2] != p.JOINT_FIXED:
            joint_map[info[1].decode("utf-8")] = i

    mcp_offsets = {
        "wrist": [0, 0, -0.015],
        "index": [0, 0, -0.008],
        "pinky": [0, 0, 0.006]
    }
    
    # Calculate Initial Frames and Hand Scaling
    state = p.getLinkState(robot_id, 1)
    wrist_pos = apply_offset(state[4], state[5], mcp_offsets["wrist"])
    
    state = p.getLinkState(robot_id, 2)
    index_mcp_pos = apply_offset(state[4], state[5], mcp_offsets["index"])
    
    state = p.getLinkState(robot_id, 13)
    pinky_mcp_pos = apply_offset(state[4], state[5], mcp_offsets["pinky"])
    
    state = p.getLinkState(robot_id, 7)
    mid_mcp_pos = apply_offset(state[4], state[5], [0, 0, 0])
    
    hand_width = np.linalg.norm(mid_mcp_pos - wrist_pos)

    init_x_axis = index_mcp_pos - wrist_pos 
    init_z_axis = np.cross(index_mcp_pos - wrist_pos, pinky_mcp_pos - wrist_pos) 
    init_y_axis = np.cross(init_z_axis, init_x_axis)
    init_x_axis /= np.linalg.norm(init_x_axis)
    init_y_axis /= np.linalg.norm(init_y_axis)
    init_z_axis /= np.linalg.norm(init_z_axis)
    R_robot = np.stack([init_x_axis, init_y_axis, init_z_axis], axis=1)

    # 2. SETUP RETARGETING
    config_path = "/Users/sissi/Downloads/RUKA/assets/dex_retarget.yml"
    retargeting = RetargetingConfig.load_from_file(config_path).build()
    target_joint_names = retargeting.optimizer.robot.dof_joint_names
    
    # PERFORMANCE TUNING: Zero-tolerance for snappiness
    retargeting.optimizer.opt.set_maxeval(50)
    retargeting.optimizer.opt.set_xtol_rel(0)

    # 3. SETUP MEDIAPIPE
    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils
    cap = cv2.VideoCapture(1)

    saved_points = []

    print("--- RECORDING STARTED ---")
    print("Press ESC in the window or Ctrl+C in terminal to stop and save.")

    try:
        with mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7) as hands:
            while cap.isOpened():
                p.stepSimulation()
                ret, frame = cap.read()
                if not ret: break
                
                frame = cv2.flip(frame, 1)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = hands.process(rgb)

                if results.multi_hand_world_landmarks:
                    hand_landmarks = results.multi_hand_world_landmarks[0]
                    
                    # Raw Points for saving
                    points = np.array([[lm.x, lm.y, lm.z] for lm in hand_landmarks.landmark])
                    saved_points.append(points)
                    
                    # Transformation Logic
                    points_zeroed = points - points[0]
                    wrist = points_zeroed[0]
                    index_mcp = points_zeroed[5]
                    pinky_mcp = points_zeroed[17]

                    palm_normal = np.cross(index_mcp - wrist, pinky_mcp - wrist)
                    palm_normal /= np.linalg.norm(palm_normal)
                    x_axis = index_mcp - wrist
                    y_axis = np.cross(palm_normal, x_axis)
                    x_axis /= np.linalg.norm(x_axis)
                    y_axis /= np.linalg.norm(y_axis)
                    R_hand = np.stack([x_axis, y_axis, palm_normal], axis=1)  
                    
                    width = (2 * np.linalg.norm(points_zeroed[9] - wrist) + np.linalg.norm(pinky_mcp - wrist)) / 3
                    joint_pos = (hand_width / width) * (points_zeroed @ R_hand @ R_robot.T) + wrist_pos

                    # Solver Logic
                    indices = retargeting.optimizer.target_link_human_indices
                    origin_indices = indices[0, :]
                    task_indices = indices[1, :]
                    ref_value = joint_pos[task_indices, :] - joint_pos[origin_indices, :]

                    robot_q = retargeting.retarget(ref_value) 

                    # Apply to PyBullet
                    for i, joint_name in enumerate(target_joint_names):
                        if joint_name in joint_map:
                            p.resetJointState(robot_id, joint_map[joint_name], robot_q[i])
                    
                    mp_draw.draw_landmarks(frame, results.multi_hand_landmarks[0], mp_hands.HAND_CONNECTIONS)

                cv2.imshow("Webcam Feed", frame)
                # ESC key to exit
                if cv2.waitKey(1) & 0xFF == 27: 
                    raise KeyboardInterrupt 

    except KeyboardInterrupt:
        print("\nStopping recording...")

    # 4. CLEANUP AND SAVE
    p.disconnect()
    cap.release()
    cv2.destroyAllWindows()

    if len(saved_points) > 0:
        trajectory_array = np.array(saved_points, dtype=np.float32)
        # np.save("mp_trajectory.npy", trajectory_array)
        # print(f"Successfully saved {trajectory_array.shape[0]} frames to mp_trajectory.npy")
    else:
        print("No frames recorded. File not saved.")

if __name__ == "__main__":
    main()
