std_msgs/Header header
string robotname
float32 joint_num
geometry_msgs/Quaternion quat
geometry_msgs/Pose pose

#control message
float64[12] kp_joint
float64[12] kd_joint
float64[12] tau_ff
float64[12] q_des
float64[12] q_data
float64[12] dq_des
float64[12] dq_data
