std_msgs/Header header
int32 left_count            # cumulative encoder ticks, wraps at int32 max
int32 right_count           # cumulative encoder ticks, wraps at int32 max
float32 left_speed_mps      # derived wheel speed, m/s
float32 right_speed_mps     # derived wheel speed, m/s
