# Longitudinal control command – mirrors autoware_control_msgs/msg/Longitudinal.
# Published by longitudinal controllers (PID, Fuzzy PID) to the vehicle plant.
#
# stamp        – time of this command (simulation clock)
# control_time – time when the command was computed
# velocity     [m/s]   – target velocity from planning (passed through for logging)
# acceleration [m/s^2] – final target acceleration after feed-forward + feedback + limits
# jerk         [m/s^3] – rate of acceleration change after jerk limiting
# is_defined_acceleration – false when acceleration field is not valid
# is_defined_jerk         – false when jerk field is not valid

builtin_interfaces/Time stamp
builtin_interfaces/Time control_time
float32 velocity
float32 acceleration
float32 jerk
bool is_defined_acceleration
bool is_defined_jerk
