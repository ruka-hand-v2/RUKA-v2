import cv2
import mediapipe as mp
import numpy as np

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(1)

def draw_axis(frame, origin, axis, color, scale=0.1):
    h, w, _ = frame.shape
    p0 = (int(origin[0] * w), int(origin[1] * h))
    p1 = (
        int((origin[0] + axis[0] * scale) * w),
        int((origin[1] + axis[1] * scale) * h)
    )
    cv2.arrowedLine(frame, p0, p1, color, 2, tipLength=0.2)

thumb_cloud = []
norm_lengths = []

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb)

        if result.multi_hand_landmarks:
            hand_landmarks = result.multi_hand_landmarks[0]
            frame_points = np.array([
                [lm.x, lm.y, lm.z] for lm in hand_landmarks.landmark
            ])

            wrist = frame_points[0]
            index_mcp = frame_points[5]
            pinky_mcp = frame_points[17]
            thumb_tip = frame_points[4] # 4, 8

            norm_length = np.linalg.norm(pinky_mcp - index_mcp)
            norm_lengths.append(norm_length)

            # Build hand-centric frame 
            x_axis = index_mcp - wrist
            x_axis /= np.linalg.norm(x_axis)

            y_axis = pinky_mcp - wrist
            y_axis /= np.linalg.norm(y_axis)

            z_axis = np.cross(x_axis, y_axis)
            z_axis /= np.linalg.norm(z_axis)

            y_axis = np.cross(z_axis, x_axis)

            # Draw axes 
            draw_axis(frame, wrist, x_axis, (0, 0, 255))
            draw_axis(frame, wrist, y_axis, (0, 255, 0))
            draw_axis(frame, wrist, z_axis, (255, 0, 0))

            # Thumb point in hand frame 
            thumb_rel = thumb_tip - wrist
            thumb_hand = np.array([x_axis, y_axis, z_axis]) @ thumb_rel
            thumb_cloud.append(thumb_hand)

            mp_draw.draw_landmarks(
                frame,
                hand_landmarks,
                mp_hands.HAND_CONNECTIONS
            )

        cv2.imshow("MediaPipe Hands", frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    print("\nCtrl+C detected — stopping capture.")

finally:
    cap.release()
    cv2.destroyAllWindows()

    if len(thumb_cloud) == 0:
        print("No data collected — nothing to save.")
    else:
        thumb_cloud = np.array(thumb_cloud)
        norm_lengths = np.array(norm_lengths)

        print("Collected thumb points:", thumb_cloud.shape)

        palm_scale = np.median(norm_lengths)
        thumb_cloud_norm = thumb_cloud / palm_scale

        print("Palm width scale:", palm_scale)
        print("Normalized cloud shape:", thumb_cloud_norm.shape)

        np.save("thumb_cloud_normalized_mp.npy", thumb_cloud_norm)
        print("Saved to thumb_cloud_normalized_mp.npy")
