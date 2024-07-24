Header header
# ATTENTION：控制信号的定义，加速度和转向角
float64 acc
float64 steer_rate

# if open_loop is set, use the state as desired state 
# and omit the control loop
# 这里还有期望的状态，和是否是开环控制
std_msgs/Bool is_openloop
State state





