std_msgs/Header header
bool connected            # true if Stadia controller is paired and active
float32 battery_pct       # controller battery, percent 0-100; -1 if unknown
bool last_input_active    # any stick/button activity in the last 500ms
