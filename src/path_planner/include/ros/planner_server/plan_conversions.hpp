/**
 * @file plan_conversions.hpp
 * @brief Conversion of a planned `Path` into the `PlanTrajectory` message the follower consumes.
 */

#pragma once

#include "std_msgs/msg/header.hpp"
#include "path_planner_msgs/msg/plan_trajectory.hpp"

#include "core/planner/path.hpp"

/**
 * @brief Convert a planned path to a framed `PlanTrajectory` (dense sample poses plus the per-pose
 *        motion intent the follower needs).
 *
 * Every segment's dense sample poses are flattened, in order, into `path.poses`. For each pose,
 * `mode` is `FWD`, `REV` or `PIVOT` according to the segment's control (a pivot has zero linear
 * speed and non-zero turn rate; negative linear speed is reverse) and `speed` is the segment's
 * signed linear speed (m/s). `mode`, `speed` and `path.poses` have equal length. A path with no
 * segments gives an empty trajectory.
 *
 * @param path   Planned path; sample poses are in the planning frame (m, m, rad).
 * @param header Frame and time stamp given to the trajectory, its `path` and each pose.
 * @return Trajectory with `plan_id` 0 and `speed_scale` 1.0; the publisher assigns the id.
 */
path_planner_msgs::msg::PlanTrajectory
pathToTrajectory(const Path & path, const std_msgs::msg::Header & header);
