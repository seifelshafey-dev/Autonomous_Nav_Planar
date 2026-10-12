# A tracked plan: a nav_msgs/Path plus index-aligned per-pose motion intent.
#
# The controller consumes this to follow the plan in closed loop. The wrapped
# nav_msgs/Path is what rviz/nav2 read directly; mode[] and speed[] carry the
# per-pose motion intent the planner knows exactly (sign of v and the pivot
# flag) but which a bare nav_msgs/Path discards.
#
# Index-alignment invariant (producer- and consumer-enforced):
#   mode.size() == speed.size() == path.poses.size()
# The i-th entry of mode/speed describes the i-th pose in path.poses.

std_msgs/Header  header
nav_msgs/Path    path        # flattened sample poses, ordered start->goal (rviz/nav2 read this)
int8[]           mode        # per-pose motion mode (see constants below), index-aligned with path.poses
float64[]        speed       # per-pose signed reference speed v (m/s), index-aligned with path.poses

uint32           plan_id     # identity of this plan, assigned by its publisher, increasing; 0 for the empty stop plan
float64          speed_scale # in (0, 1]; multiplies the follower's cruise speed and pivot rate for this plan

# Motion mode values for mode[].
int8 FWD=0
int8 REV=1
int8 PIVOT=2
