# Automatically Generated in 2017-06-12 22:33:47.453254
# MESSAGE: COMMAND
# Description:Command to perform
uint8 ID = 210
uint8 sysid
uint8 compid

uint8 command   # Command to perform (see COMMAND_TYPE ENUM)

uint8 COMMAND_TYPE_IMU_CALI = 0           # Command: Perform IMU calibration
uint8 COMMAND_TYPE_SELF_TEST = 1          # Command: Perform self-test of sensor and actor
