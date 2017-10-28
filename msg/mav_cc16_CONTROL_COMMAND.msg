# Automatically Generated in 2017-06-12 22:33:47.452827
# MESSAGE: CONTROL_COMMAND
# Description: Control command data
uint8 ID = 64
uint8 sysid
uint8 compid

float32 velocity            # Desired velocity [m/s]
float32 steering_front      # Front axle steering angle [rad]
float32 steering_rear       # Rear axle steering angle [rad]
uint8 indicator_left        # Left turn signal indicator
uint8 indicator_right       # Right turn signal indicator

uint8 INDICATOR_OFF = 0 # turn indicator off
uint8 INDICATOR_ON  = 1 # turn indicator on
