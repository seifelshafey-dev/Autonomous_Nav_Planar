#include "ros/planner_server/plan_conversions.hpp"

#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "core/diagnostics/check.hpp"

path_planner_msgs::msg::PlanTrajectory
pathToTrajectory(const Path & path, const std_msgs::msg::Header & header) {
    using PlanTrajectory = path_planner_msgs::msg::PlanTrajectory;

    // Numeric guards for classifying a segment's control into a motion mode. The planner's pivot
    // primitive has exactly v=0, so a small epsilon robustly separates pivots from drive segments.
    constexpr double v_eps = 1e-6;
    constexpr double w_eps = 1e-6;

    PlanTrajectory out;
    out.plan_id     = 0;    // the publisher of /plan assigns the id
    out.speed_scale = 1.0;
    out.header      = header;
    out.path.header = header;

    for (const auto & segment : path.segments) {
        // Derive the per-segment motion intent (FWD / REV / PIVOT) from the segment's control.
        int8_t mode = PlanTrajectory::FWD;
        if (std::abs(segment.u.v) < v_eps && std::abs(segment.u.omega) > w_eps) {
            mode = PlanTrajectory::PIVOT;
        } else if (segment.u.v < 0.0) {
            mode = PlanTrajectory::REV;
        }

        for (const auto & s : segment.samples_poses) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = header;
            ps.pose.position.x    = s.x;
            ps.pose.position.y    = s.y;
            ps.pose.orientation.z = std::sin(s.theta * 0.5);
            ps.pose.orientation.w = std::cos(s.theta * 0.5);
            out.path.poses.push_back(ps);
            out.mode.push_back(mode);
            out.speed.push_back(segment.u.v);
        }
    }

    // Index-alignment invariant enforced at production (mirrored on the consumer side).
    CHECK(out.mode.size() == out.speed.size() && out.mode.size() == out.path.poses.size(),
          "pathToTrajectory: mode/speed/poses must be index-aligned");
    return out;
}
