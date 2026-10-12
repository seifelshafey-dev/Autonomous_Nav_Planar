/**
 * @file forward_clock.hpp
 * @brief ROS-free adapter that turns a clock that can jump backward into event times that never decrease.
 */

#pragma once

#include <algorithm>

/**
 * @brief Converts readings of a clock that may step backward into times that never decrease.
 *
 * Use it to timestamp events for `NavigationMode::step`, which requires non-decreasing times, when the
 * source clock can jump backward (for example wall time under time synchronisation, or simulated time
 * after a reset). The returned time is the sum of the forward differences between consecutive readings.
 *
 * Example: readings 0, 1, 2, -298, -297 return 0, 1, 2, 2, 3.
 *
 * @note A forward jump of the source clock is passed through at its full size.
 */
class ForwardClock {
public:
    /**
     * @brief Feed the next reading of the source clock.
     *
     * @param raw Current reading of the source clock (s). Any finite value; need not be monotonic.
     * @return Elapsed forward time since the first reading (s). 0 on the first call; never decreases.
     *         A backward step of `raw` adds zero; a forward step adds its full size.
     */
    double advance(double raw) {
        if (!started_) {
            started_ = true;
            last_raw_ = raw;
            return 0.0;
        }
        elapsed_ += std::max(0.0, raw - last_raw_);
        last_raw_ = raw;
        return elapsed_;
    }

private:
    bool started_ = false;   // false until the first reading arrives
    double last_raw_ = 0.0;  // last raw reading (s)
    double elapsed_ = 0.0;   // accumulated forward time (s)
};
