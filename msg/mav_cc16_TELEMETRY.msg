# Automatically Generated in 2017-06-12 22:33:47.452753
# MESSAGE: TELEMETRY
# Description:Telemetry
Header header

uint8 ID = 3
uint8 sysid
uint8 compid

float32 xacc
float32 yacc
float32 zacc
float32 xgyro
float32 ygyro
float32 zgyro
float32 dist_front
float32 dist_rear
float32 dist_side
float32 odom
float32 odom_accumulated
float32 xmotion_front
float32 ymotion_front
float32 xmotion_rear
float32 ymotion_rear
uint8 motion_front_quality
uint8 motion_rear_quality
int32 current_motor
uint16 current_servo_front
uint16 current_servo_rear
uint16 current_total
uint16 pwm_servo_front
uint16 pwm_servo_rear
uint16 battery_voltage
uint8 remote_control
uint8 drive_mode


uint8 REMOTE_CONTROL_STATUS_DISCONNECTED = 0     # Remote Control is not connected to the system
uint8 REMOTE_CONTROL_STATUS_CONNECTED = 1        # Remote Control connection is established
uint8 REMOTE_CONTROL_STATUS_AUTONOMOUS = 2       # Remote Control is connected and under active control
uint8 REMOTE_CONTROL_STATUS_SEMI_AUTONOMOUS = 3  # Remote Control is connected and under active control
uint8 REMOTE_CONTROL_STATUS_MANUAL = 4           # Remote Control is connected and under active control


uint8 DRIVE_MODE_TRACK = 0              # Track (excluding obstacles)
uint8 DRIVE_MODE_TRACK_OBSTACLES = 1    # Track including obstacles
uint8 DRIVE_MODE_PARKING = 2            # Parking mode
uint8 DRIVE_MODE_IDLE = 3               # Idle mode
