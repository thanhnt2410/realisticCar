# Planning reference published by the trajectory planner.
# Consumers: longitudinal controllers (PID, Fuzzy PID).
#
# velocity     [m/s]   – reference longitudinal speed
# acceleration [m/s^2] – reference longitudinal acceleration (analytic, not finite-difference)
# jerk         [m/s^3] – reference jerk (0 for piecewise-linear profiles)
# is_defined_acceleration – false when acceleration is not meaningful (e.g. hold segment edge)
# is_defined_jerk         – false when jerk is not provided

builtin_interfaces/Time stamp
float32 velocity
float32 acceleration
float32 jerk
bool is_defined_acceleration
bool is_defined_jerk
