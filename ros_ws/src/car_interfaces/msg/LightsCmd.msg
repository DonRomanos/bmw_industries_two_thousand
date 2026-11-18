uint8 MODE_SOLID=0
uint8 MODE_INDICATOR_LEFT=1
uint8 MODE_INDICATOR_RIGHT=2
uint8 MODE_HAZARD=3

uint8 FLAG_BRAKE=1
uint8 FLAG_REVERSE=2

std_msgs/Header header
uint8[12] front_rgb         # 4 lamps x [r,g,b], flattened; lamp i at indices 3*i..3*i+2; 0=leftmost..3=rightmost
uint8[96] rear_rgb          # 4 bars x 8 WS2812 pixels x [r,g,b], flattened; bar i pixel p at indices 24*i+3*p..24*i+3*p+2; bar 0=left..3=right
uint8 mode                  # one of MODE_* above
uint16 blink_period_ms      # milliseconds, default 500
uint8 flags                 # OR of FLAG_*; forces brake/reverse light, firmware ORs in its own detection
