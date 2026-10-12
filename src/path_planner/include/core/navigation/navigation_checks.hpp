/**
 * @file navigation_checks.hpp
 * @brief ROS-free checks the navigator runs on a driven path: remaining-path collision scan and rest test.
 */

#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

#include "core/collision_checker/collision_checker.hpp"
#include "core/diagnostics/check.hpp"
#include "core/types/math/wrap_angle.hpp"
#include "core/types/state.hpp"

/**
 * @brief Result of `checkRemainingPath`.
 */
struct RemainingPathCheck {
    bool        valid;  ///< True when every checked sample is collision-free (also true for an empty path).
    double      blocked_distance;  ///< Distance along the path (m) from the cursor sample to the first blocked sample; 0 when `valid` is true or when the cursor sample itself is blocked.
    std::size_t cursor;  ///< Index of the sample nearest to the robot; never less than the `cursor_prev` passed in. 0 for an empty path.
};

/**
 * @brief Find the robot's position on a path and check the rest of the path for collisions.
 *
 * The cursor is the sample at or after `cursor_prev` whose position is nearest to the robot (lowest
 * index on a tie; heading is ignored). Samples from the cursor to the end are checked in order with
 * `CollisionChecker::checkState`; samples before the cursor are not checked.
 *
 * @param samples      Path poses in the map frame (m, m, rad), in driving order.
 * @param robot_x      Robot x in the map frame (m).
 * @param robot_y      Robot y in the map frame (m).
 * @param cursor_prev  Cursor returned by the previous call (0 on the first). Ignored when `samples` is empty;
 *                     otherwise must be < `samples.size()`, else a CHECK throws std::invalid_argument.
 * @param checker      Collision checker over the current map.
 * @return `{true, 0, 0}` for an empty path; otherwise the cursor, whether the remaining path is clear,
 *         and the distance along the path to the first blocked sample.
 */
inline RemainingPathCheck checkRemainingPath(const std::vector<SE2State>& samples, double robot_x, double robot_y,
                                             std::size_t cursor_prev, const CollisionChecker& checker) {
    if (samples.empty()) return {true, 0.0, 0};
    CHECK(cursor_prev < samples.size(), "checkRemainingPath: cursor_prev must be < samples.size()");

    std::size_t cursor = cursor_prev;
    double best = std::hypot(samples[cursor].x - robot_x, samples[cursor].y - robot_y);
    for (std::size_t i = cursor_prev + 1; i < samples.size(); ++i) {
        const double d = std::hypot(samples[i].x - robot_x, samples[i].y - robot_y);
        if (d < best) { best = d; cursor = i; }
    }

    double along = 0.0;
    for (std::size_t i = cursor; i < samples.size(); ++i) {
        if (!checker.checkState(samples[i])) return {false, along, cursor};
        if (i + 1 < samples.size()) along += std::hypot(samples[i + 1].x - samples[i].x, samples[i + 1].y - samples[i].y);
    }
    return {true, 0.0, cursor};
}

/**
 * @brief Whether two poses are the same within a position and a heading tolerance.
 *
 * @param a                   First pose (m, m, rad).
 * @param b                   Second pose (m, m, rad), same frame as `a`.
 * @param position_tolerance  Maximum Euclidean distance (m); finite and >= 0, else a CHECK throws std::invalid_argument.
 * @param heading_tolerance   Maximum absolute heading difference (rad, wrapped to [-pi, pi]); finite and >= 0,
 *                            else a CHECK throws std::invalid_argument.
 * @return true iff both differences are within their tolerance (bounds inclusive).
 */
inline bool atRest(const SE2State& a, const SE2State& b, double position_tolerance, double heading_tolerance) {
    CHECK(std::isfinite(position_tolerance) && position_tolerance >= 0.0, "atRest: position_tolerance must be finite and >= 0");
    CHECK(std::isfinite(heading_tolerance) && heading_tolerance >= 0.0, "atRest: heading_tolerance must be finite and >= 0");
    return std::hypot(b.x - a.x, b.y - a.y) <= position_tolerance &&
           std::fabs(wrapToPi(b.theta - a.theta)) <= heading_tolerance;
}
