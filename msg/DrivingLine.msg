Header header

# x range of the current detected lane
float32 detectionRange

# The polynom describing the driving lane
uint8 polynom_order
float32[] polynom_params # has polynom_order + 1 elements
