# Follower progress for the plan being followed. Published at 5 Hz and whenever a field changes.
std_msgs/Header header
uint32 plan_id    # plan_id of the plan being followed; 0 when idle
uint32 cursor     # index of the trajectory pose being tracked, in [0, size)
uint32 size       # number of poses in the plan; 0 when idle
bool arrived      # final pose reached within the goal tolerances
