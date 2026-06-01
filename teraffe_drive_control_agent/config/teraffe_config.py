# Teraffe Robot Specific Configuration

# --- Dynamixel Configuration ---
DYNAMIXEL_PORT = "/dev/ttyUSB0"
DYNAMIXEL_BAUDRATE = 57600

DYNAMIXEL_IDS = {
    "front_left": 1,
    "front_right": 2,
    "rear_left": 3,
    "rear_right": 4,
}

# --- Encoder Configuration ---
AMT212_PORT = "/dev/ttyUSB1"
AMT212_BAUDRATE = 115200

# --- AK80-64 Configuration ---
AK80_64_IDS = {
    "front_left_drive": 101,
    "front_right_drive": 102,
    "rear_left_drive": 103,
    "rear_right_drive": 104,
    "front_left_lift": 1,
    "front_right_lift": 2,
    "rear_left_lift": 3,
    "rear_right_lift": 4,
}
