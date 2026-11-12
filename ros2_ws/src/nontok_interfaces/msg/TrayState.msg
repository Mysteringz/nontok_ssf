std_msgs/Header header
uint8 EMPTY=0
uint8 OCCUPIED=1
uint8 MOVING=2
uint8 state
float32 occupied_fraction   # share of tray pixels that differ from the empty reference
bool stable                 # unchanged for the configured time
bool has_reference
