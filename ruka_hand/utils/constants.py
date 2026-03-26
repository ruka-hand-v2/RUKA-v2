# RUKA constants
# Finger to Motor_IDs
# Actuators indexed from palm outwards, i.e. actuator closest to palm is self.finger[0]
# side is self.finger[0] in index, ring, pinky

# BELOW IS FOR RUKA V2 RIGHT
FINGER_NAMES_TO_MOTOR_IDS = {
    "Thumb": [12, 13, 11],
    "Index": [7, 8, 6],
    "Middle": [9, 10],
    "Ring": [3, 4, 5],
    "Pinky": [1, 0, 2],
    "Wrist": [14, 15]
}

##FOR RUKA V2 LEFT
# FINGER_NAMES_TO_MOTOR_IDS = {
#     "Thumb": [13, 12, 11],
#     "Index": [8, 6, 7],
#     "Middle": [9, 10],
#     "Ring": [4, 5, 3],
#     "Pinky": [1, 0, 2],
#     "Wrist": [14, 15]
# }

# # BELOW IS FOR RUKA V1 LEFT
# FINGER_NAMES_TO_MOTOR_IDS = {
#     "Thumb": [0, 1, 2],
#     "Index": [4, 3],
#     "Middle": [6, 5],
#     "Ring": [7, 8],
#     "Pinky": [9, 10]
# }
FINGER_NAMES_TO_MANUS_IDS = {"Thumb": 0, "Index": 1, "Middle": 2, "Ring": 3, "Pinky": 4}
MOTOR_RANGES_LEFT = [724, 600, 563, 1230, 930, 1240, 930, 1000, 1270, 1100, 1100]
MOTOR_RANGES_RIGHT = [900, 600, 563, 1430, 930, 1340, 1058, 1000, 1270, 1200, 1300]
USB_PORTS = {"left": "/dev/tty.usbserial-FTAU5737", "right": "/dev/tty.usbserial-FTAA0B7K", "v1_right": "/dev/ttyUSB0"} #/dev/tty.usbserial-FTAA0B7K /dev/tty.usbserial-FTAU5737 # /dev/ttyUSB0 or 1

# Controller constants
HOST = "127.0.0.1"
CHECKPOINT_DIR = "ruka_data/osfstorage/checkpoints"

# Manus constants
RIGHT_STREAM_PORT = 5050
LEFT_STREAM_PORT = 5051
LEFT_GLOVE_ID = "<input glove id>"
RIGHT_GLOVE_ID = "<input glove id>"

# Oculus constants
OCULUS_NUM_KEYPOINTS = 24
OCULUS_JOINTS = {
    "metacarpals": [2, 6, 9, 12, 15],
    "knuckles": [6, 9, 12, 16],
    "thumb": [2, 3, 4, 5, 19],
    "index": [6, 7, 8, 20],
    "middle": [9, 10, 11, 21],
    "ring": [12, 13, 14, 22],
    "pinky": [15, 16, 17, 18, 23],
}
HAND_JOINTS = {
    "thumb": [0, 3, 4, 5, 19],
    "index": [0, 6, 7, 8, 20],
    "middle": [0, 9, 10, 11, 21],
    "ring": [0, 12, 13, 14, 22],
    "pinky": [0, 16, 17, 18, 23],
}
OCULUS_LEFT_PORT = 8110
OCULUS_RIGHT_PORT = 8087
