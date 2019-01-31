Header header

###############################################################
# Usually we should only detect one intersection, but we still send all, if we detect more.
# They should be somehow fused with detected signs.
# For every intersection there is one entry in each of the following three arrays.
###############################################################

# The position in rear_axis_middle_ground (z is always 0)
geometry_msgs/PointStamped[] positions

# The confidence which is greater 0.0, but has no upper limit
float32[] confidences

# True, if we detected some line(s) on the ego lane. This could be a stop or yield line.
# We should trust the signs more than this.
bool[] stop_line_detected
