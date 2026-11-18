std_msgs/Header header
float32 voltage_1        # AGM battery 1, Volts
float32 voltage_2        # AGM battery 2, Volts
float32 voltage_total    # combined pack voltage, Volts
float32 current_a        # pack current, Amps; 0 if not sensed
bool estop_clear         # true when e-stop is not pressed
