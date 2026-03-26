import numpy as np
import smplx
import torch
import matplotlib.pyplot as plt

num_samples = 1000 
thumb_vectors = []

# Load MANO
mano = smplx.create(
    model_path="models",
    model_type="mano",
    is_rhand=True,
    use_pca=False,
    flat_hand_mean=False
)

# Indices (MANO right hand)
wrist_idx = 0
thumb_joints = [3, 4, 5]  # 12 13 14 thumb joints, 012 index
thumb_tip_idx = 6
index_mcp_idx = 1
pinky_mcp_idx = 7

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

for i in range(num_samples):
    # Randomly move thumb joints only
    hand_pose = torch.zeros(1, 45)
    for j in thumb_joints:
        hand_pose[0, j*3 : j*3 + 3] = torch.randn(3) * 0.5

    out_sample = mano(
        global_orient=torch.zeros(1, 3),
        hand_pose=hand_pose,
        betas=torch.zeros(1, 10)
    )

    joints = out_sample.joints[0].detach().numpy()

    # hand frame calculation 
    wrist = joints[wrist_idx]
    index_mcp = joints[index_mcp_idx]
    pinky_mcp = joints[pinky_mcp_idx]

    x_axis = index_mcp - wrist
    x_axis /= np.linalg.norm(x_axis)

    y_axis = pinky_mcp - wrist
    y_axis -= y_axis.dot(x_axis) * x_axis
    y_axis /= np.linalg.norm(y_axis)

    z_axis = np.cross(x_axis, y_axis)
    hand_frame = np.vstack([x_axis, y_axis, z_axis]).T

    # Thumb tip in hand frame 
    thumb_tip = joints[thumb_tip_idx]
    vec = thumb_tip - wrist
    vec_hand = hand_frame.T @ vec
    hand_width = np.linalg.norm(index_mcp - pinky_mcp)
    vec_normalized = vec_hand / hand_width
    thumb_vectors.append(vec_normalized)

thumb_vectors = np.array(thumb_vectors)
np.save("thumb_cloud_normalized_mano_middle.npy", thumb_vectors)
print("Saved as thumb_cloud_normalized_mano.npy")
