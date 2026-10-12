/**
 * @file wrap_angle.hpp
 * @author Seif ElShafey
 * @brief Helper function to wrap any euler angle into the canonical SE(2) range [-pi, pi]. Used throughout the codebase.
 * @version 0.3
 * @date 2026-05-30
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <cmath>
#include <numbers>

/**
 * @brief Function that wraps euler angles into their corresponding signed value in [-pi, pi].
 *
 * @note The range is closed at both ends: an input of exactly +pi returns +pi and exactly -pi returns -pi,
 *       so +pi is a possible result and the range is not half-open. A small clockwise rotation of -0.1 rad
 *       stays -0.1. A non-finite input returns NaN.
 * @param angle Input euler angle. Usually used for robot heading.
 * @return double Wrapped angle representation in [-pi, pi].
 */
inline double wrapToPi(double angle) {
    // std::remainder uses round-half-to-even on the quotient, yielding a result in
    // [-pi, pi]. The tie-to-even rule preserves the input sign at the boundary, so
    // wrapToPi(pi) == pi and wrapToPi(-pi) == -pi (both valid since they are the same heading).
    return std::remainder(angle, 2 * std::numbers::pi);
}