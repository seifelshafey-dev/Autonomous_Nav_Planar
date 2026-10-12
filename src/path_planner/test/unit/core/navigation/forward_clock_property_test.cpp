#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "core/navigation/forward_clock.hpp"

/*
For any non-empty sequence of finite readings: outputs never decrease, the first is 0, and the last
equals the sum of the positive consecutive differences.
*/

RC_GTEST_PROP(ForwardClockProp, nonDecreasingStartsAtZeroAndSumsForwardSteps, ()) {
    const auto n = *rc::gen::inRange(1, 40);
    std::vector<double> r;
    for (int k = 0; k < n; ++k) {
        r.push_back(static_cast<double>(*rc::gen::inRange(-1000000000, 1000000001)) / 1000.0);  // [-1e6, 1e6] s
    }

    ForwardClock c;
    std::vector<double> out;
    for (double x : r) out.push_back(c.advance(x));

    RC_ASSERT(out.front() == 0.0);
    for (std::size_t k = 1; k < out.size(); ++k) RC_ASSERT(out[k] >= out[k - 1]);

    double expected = 0.0;
    for (std::size_t k = 1; k < r.size(); ++k) expected += std::max(0.0, r[k] - r[k - 1]);
    RC_ASSERT(std::abs(out.back() - expected) <= 1e-9 * std::max(1.0, std::abs(expected)));
}
