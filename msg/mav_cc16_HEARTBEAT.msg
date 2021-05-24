# Automatically Generated in 2017-06-12 22:33:47.452693
# MESSAGE: HEARTBEAT
# Description:Heartbeat status message
std_msgs/Header header

uint8 ID = 1
uint8 sysid
uint8 compid

uint16 battery_voltage      # Battery voltage, in millivolts (1 = 1 millivolt)
uint8 remote_control        # Remote Control status (see REMOTE_CONTROL_STATUS ENUM)
uint8 drive_mode            # Selected drive mode (see DRIVE_MODE ENUM)
float32 rc_velocity         # Velocity set by remote control [m/s]
float32 rc_steering_front   # Front steering angle set by remote control [rad]
float32 rc_steering_rear    # Rear steering angle set by remote control [rad]

uint8 REMOTE_CONTROL_STATUS_DISCONNECTED = 0     # Remote Control is not connected to the system
uint8 REMOTE_CONTROL_STATUS_CONNECTED = 1        # Remote Control connection is established
uint8 REMOTE_CONTROL_STATUS_AUTONOMOUS = 2       # Remote Control is connected and under active control
uint8 REMOTE_CONTROL_STATUS_SEMI_AUTONOMOUS = 3  # Remote Control is connected and under active control
uint8 REMOTE_CONTROL_STATUS_MANUAL = 4           # Remote Control is connected and under active control


uint8 DRIVE_MODE_TRACK = 0              # Track (excluding obstacles)
uint8 DRIVE_MODE_TRACK_OBSTACLES = 1    # Track including obstacles
uint8 DRIVE_MODE_PARKING = 2            # Parking mode
uint8 DRIVE_MODE_IDLE = 3               # Idle mode
