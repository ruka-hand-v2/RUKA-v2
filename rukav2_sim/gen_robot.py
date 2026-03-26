import pybullet as p
import pybullet_data
import time
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)

robot_id = p.loadURDF("assets/robot.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)

for i in range(-1, 17):
    for j in range(-1, 17):
        p.setCollisionFilterPair(robot_id, robot_id, i, j, enableCollision=0)
    p.setCollisionFilterPair(robot_id, robot_id, i, 20, enableCollision=0)

joint_name_to_index = {}
link_name_to_index = {}
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    joint_name_to_index[info[1].decode()] = i
    link_name_to_index[info[12].decode()] = i
thumb_joints = [
    joint_name_to_index["thumb_cmc"],
    joint_name_to_index["thumb_mcp"],
    joint_name_to_index["thumb_ip"],
]

thumb_tip_link = 19
tip_offset = np.array([-0.017, 0.0025, -0.005])
index_tip_link = 5
index_tip_offset = np.array([-0.027, 0, -0.008])
wrist_link = 1
wrist_offset = np.array([0, 0, -0.015])
index_mcp_link = 2
index_mcp_offset = np.array([0, 0, -0.008])
pinky_mcp_link = 13
pinky_mcp_offset = np.array([0, 0, 0.006])

sphere_radius = 0.007
visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=[1, 0, 0, 1]
)
marker_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape_id)

def apply_offset(pos, orn, offset):
    rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    return pos + rot.dot(offset)

point_cloud = []
num_samples = 2000

for _ in range(num_samples):
    # Random thumb joint angles
    for joint_index in [17, 18, 19]: # 17 18 19, 2 3 4 5
        info = p.getJointInfo(robot_id, joint_index)
        lower, upper = info[8], info[9]
        p.resetJointState(robot_id, joint_index, np.random.uniform(lower, upper))

    p.stepSimulation()

    contacts = p.getContactPoints(bodyA=robot_id, bodyB=robot_id)
    if len(contacts) > 0:
        # for c in contacts:
        #     pos = c[5]
        #     end = [pos[0], pos[1], pos[2] + 0.2]  
        #     line_id = p.addUserDebugLine(pos, end, [1, 0, 0], lineWidth=5, lifeTime=1)
        #     print(f" Link {c[3]} (vs) Link {c[4]}")
        continue  # skip this configuration if there is any collision


    wrist_state = p.getLinkState(robot_id, wrist_link)
    wrist_pos = apply_offset(np.array(wrist_state[4]), wrist_state[5], wrist_offset)

    index_state = p.getLinkState(robot_id, index_mcp_link)
    index_pos = apply_offset(np.array(index_state[4]), index_state[5], index_mcp_offset)

    pinky_state = p.getLinkState(robot_id, pinky_mcp_link)
    pinky_pos = apply_offset(np.array(pinky_state[4]), pinky_state[5], pinky_mcp_offset)

    # Hand-centric frame
    x_axis = index_pos - wrist_pos
    x_axis /= np.linalg.norm(x_axis)
    y_axis = pinky_pos - wrist_pos
    y_axis /= np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)
    z_axis /= np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, x_axis)
    hand_frame = np.vstack([x_axis, y_axis, z_axis]).T  # 3x3 rotation

    # Thumb tip in world frame
    thumb_state = p.getLinkState(robot_id, thumb_tip_link)
    thumb_pos = apply_offset(np.array(thumb_state[4]), thumb_state[5], tip_offset)

    # Transform to hand frame
    thumb_rel = thumb_pos - wrist_pos
    thumb_hand = hand_frame.T @ thumb_rel  # 3D coords in hand frame

    point_cloud.append(thumb_hand)

    # Update visualization
    p.resetBasePositionAndOrientation(marker_id, thumb_pos, [0, 0, 0, 1])

# Normalize by median palm width 
point_cloud = np.array(point_cloud)
norm_length = np.linalg.norm(pinky_pos - index_pos)
thumb_cloud_norm = point_cloud / norm_length

print("Collected points:", point_cloud.shape)
print("Normalized points:", thumb_cloud_norm.shape)
np.save("thumb_cloud_normalized_robot.npy", thumb_cloud_norm)
print("Saved as thumb_cloud_normalized_robot.npy")

"""
thumb tip: link 19, offset [-0.017, 0.0025, -0.005]
wrist: link, 1, offset [0, 0, -0.015]
index mcp: link 2, offset [0, 0, -0.008]
pinky mcp: link 13, offset [0, 0, 0.006]
index tip: link 5, offset [-0.027, 0, -0.004]
middle tip: link 8, offset [-0.025, 0.002, -0.004]
ring tip: link 12, [-0.025, 0.002, -0.004]
pinky tip: link 16, [-0.022, 0.002, -0.004]
"""