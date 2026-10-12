#include <gtest/gtest.h>
#include <numbers>
#include "core/types/math/wrap_angle.hpp"

/*
Wrap to Pi contract:
    Invariant: Finite number.
    Input: Takes in any number of type double (both negative and positive). Edge cases - -pi, pi, 0, >pi, <-pi.
    Contract: Wraps the angle to a value between -pi and pi.
    Output: [-pi,pi].
*/

TEST(WrappedAngle, correctWrapping){
    double angle1 = (7 * std::numbers::pi)/3;
    double angle2 = (std::numbers::pi)/2;
    double angle3 = -3*(std::numbers::pi)/2;
    ASSERT_NEAR(wrapToPi(angle1), (std::numbers::pi / 3), 1e-9);
    ASSERT_NEAR(wrapToPi(angle2), (std::numbers::pi)/2, 1e-9);
    ASSERT_NEAR(wrapToPi(angle3), (std::numbers::pi)/2, 1e-9);
}

TEST(WrappedAngle, zeroWrapping){
    double angle1 = 0.0;
    ASSERT_EQ(0.0, wrapToPi(angle1));
}

TEST(WrappedAngle, edgeWrapping){
    double angle1 = std::numbers::pi;
    double angle2 = -std::numbers::pi;
    ASSERT_NEAR(wrapToPi(angle1), std::numbers::pi, 1e-9);
    ASSERT_NEAR(wrapToPi(angle2), -std::numbers::pi, 1e-9);
}

// Odd multiples of pi sit exactly on the wrap seam. Round-half-to-even on the quotient
// sends +3*pi to -pi and -3*pi to +pi (quotient 1.5/-1.5 both round to the even 2/-2).
TEST(WrappedAngle, oddMultiplesOfPi){
    constexpr double pi = std::numbers::pi;
    ASSERT_NEAR(wrapToPi(3 * pi), -pi, 1e-9);
    ASSERT_NEAR(wrapToPi(-3 * pi), pi, 1e-9);
}

// Even multiples of pi (full revolutions) must collapse to 0, not to the +/-pi seam.
TEST(WrappedAngle, evenMultiplesOfTwoPi){
    constexpr double pi = std::numbers::pi;
    ASSERT_NEAR(wrapToPi(2 * pi), 0.0, 1e-9);
    ASSERT_NEAR(wrapToPi(4 * pi), 0.0, 1e-9);
    ASSERT_NEAR(wrapToPi(-2 * pi), 0.0, 1e-9);
}

// Continuity check just past the +/-pi boundary: pi + d wraps to -pi + d.
TEST(WrappedAngle, justAcrossTheSeam){
    constexpr double pi = std::numbers::pi;
    double delta = 0.01;
    ASSERT_NEAR(wrapToPi(pi + delta), -pi + delta, 1e-9);
    ASSERT_NEAR(wrapToPi(-pi - delta), pi - delta, 1e-9);
}
