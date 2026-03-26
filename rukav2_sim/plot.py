import numpy as np
import matplotlib.pyplot as plt

# Load normalized thumb clouds
human_cloud = np.load("thumb_cloud_normalized_mp.npy")        # MediaPipe / human
robot_cloud = np.load("thumb_cloud_normalized_robot_index.npy")    # PyBullet / robot
mano_cloud = np.load("thumb_cloud_normalized_mano_index.npy")    # MANO / simulated hand

# Create 3D plot
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# # Plot human thumb cloud in blue
ax.scatter(human_cloud[:, 0], human_cloud[:, 1], human_cloud[:, 2],
           s=5, alpha=0.6, color='blue', label='Human')

# Plot robot thumb cloud in red
ax.scatter(robot_cloud[:, 0], robot_cloud[:, 1], robot_cloud[:, 2],
           s=5, alpha=0.6, color='red', label='Robot')

# Plot MANO thumb cloud in green
ax.scatter(mano_cloud[:, 0], mano_cloud[:, 1], mano_cloud[:, 2],
           s=5, alpha=0.6, color='green', label='MANO')

axis_len = 1.0
ax.plot([0, axis_len], [0, 0], [0, 0], color='r')  # X
ax.plot([0, 0], [0, axis_len], [0, 0], color='g')  # Y
ax.plot([0, 0], [0, 0], [0, 0], color='b')  # Z

ax.set_xlabel("X (index direction)")
ax.set_ylabel("Y (pinky direction)")
ax.set_zlabel("Z (palm normal)")

ax.set_title("Normalized Thumb Tip Point Cloud: Human vs Robot vs MANO")
ax.legend()

all_points = np.vstack([human_cloud, robot_cloud, mano_cloud])
max_range = np.max(np.ptp(all_points, axis=0))
mid = np.mean(all_points, axis=0)

ax.set_xlim(mid[0] - max_range/2, mid[0] + max_range/2)
ax.set_ylim(mid[1] - max_range/2, mid[1] + max_range/2)
ax.set_zlim(mid[2] - max_range/2, mid[2] + max_range/2)

plt.show()
