#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <cmath>
#include <numbers>
#include "core/types/math/wrap_angle.hpp"

/*
Property-based tests for wrapToPi (RapidCheck).

RapidCheck's arbitrary<double> generator deliberately produces edge values
(inf, nan, huge, zero). Finiteness is a precondition of wrapToPi, so each
property discards non-finite inputs with RC_PRE before asserting.
*/

namespace {
constexpr double pi = std::numbers::pi;
}

// Range invariant: every wrapped angle lies within the canonical [-pi, pi].
RC_GTEST_PROP(WrappedAngleProp, outputInRange, (double angle)) {
    RC_PRE(std::isfinite(angle));
    double wrapped = wrapToPi(angle);
    RC_ASSERT(wrapped >= -pi - 1e-12);
    RC_ASSERT(wrapped <= pi + 1e-12);
}

// Idempotency: an already-wrapped angle is a fixed point of wrapToPi.
RC_GTEST_PROP(WrappedAngleProp, idempotent, (double angle)) {
    RC_PRE(std::isfinite(angle));
    double once = wrapToPi(angle);
    double twice = wrapToPi(once);
    RC_ASSERT(once == twice);
}
