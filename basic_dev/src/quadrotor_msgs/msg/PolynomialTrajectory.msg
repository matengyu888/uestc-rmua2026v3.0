Header header

# the trajectory id, starts from "1".
uint32 trajectory_id

# The type of the trajectory
uint32 type
uint32 HEART_BEAT = 1
uint32 POSITION_TRAJ = 2
uint32 YAW_TRAJ = 4
uint32 YAW_COMMAND = 8
uint32 EMER_STOP = 16

uint32 piece_num_yaw
uint32 piece_num_pos
uint32 order_pos
uint32 order_yaw

time start_WT_yaw
float64 yaw
float64 yaw_rate
float64[] coef_yaw
float64[] time_yaw

time start_WT_pos
float64[] coef_pos_x
float64[] coef_pos_y
float64[] coef_pos_z
float64[] time_pos

string debug_info
