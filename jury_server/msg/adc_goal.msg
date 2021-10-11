#type definitions
uint8 PASSTHROUGH_GOAL=0
uint8 STOP_GOAL=1
uint8 PARKING_GOAL=2

string id       # goal identifier
float32 x       # x coordinates in meters [m]
float32 y       # y coordinate in meters [m]
float32 yaw     # yaw orientation in radians [rad]
int32 type      # see type definitions above
