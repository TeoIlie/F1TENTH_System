std_msgs/Header header

# Obs-vector inputs (raw, pre-normalization), in FEATURE_ORDER
float64 linear_vel_x
float64 linear_vel_y
float64 frenet_u
float64 frenet_n
float64 ang_vel_z
float64 beta
float64 curr_avg_wheel_omega
float64[5] lookahead_curvatures
float64[2] lookahead_widths

# Diagnostics (not part of the obs vector)
float64 yaw
float64 delta

# Recovery-success flag (latches true once the recovery condition is met)
bool recovered
