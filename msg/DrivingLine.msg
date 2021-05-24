std_msgs/Header header

# x range of the current detected lane
float32 detection_range

# The polynom describing the driving lane
uint8 polynom_order
float32[] polynom_params # has polynom_order + 1 elements

# The polynom describing the left line (lane marking) of the street
bool left_line_found # True, if the poly for left line is set
uint8 left_poly_order # Only reasonable if left_line_found == True
float32[] left_poly_params # Only reasonable if left_line_found == True

# The polynom describing the right line (lane marking) of the street
bool right_line_found # True, if the poly for right line is set
uint8 right_poly_order # Only reasonable if right_line_found == True
float32[] right_poly_params # Only reasonable if right_line_found == True
