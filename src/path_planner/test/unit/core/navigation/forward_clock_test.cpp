#include <gtest/gtest.h>

#include "core/navigation/forward_clock.hpp"

/*
ForwardClock contract:
    advance(raw): 0.0 on the first call; afterwards the running sum of max(0, raw - previous raw).
    A backward step adds zero; a forward step passes through at full size.
*/

TEST(ForwardClock, backwardStepAddsZero) {
    ForwardClock c;
    EXPECT_EQ(c.advance(0.0), 0.0);
    EXPECT_EQ(c.advance(1.0), 1.0);
    EXPECT_EQ(c.advance(2.0), 2.0);
    EXPECT_EQ(c.advance(-298.0), 2.0);
    EXPECT_EQ(c.advance(-297.0), 3.0);
}

TEST(ForwardClock, firstReadingIsZeroWhateverItsValue) {
    ForwardClock c;
    EXPECT_EQ(c.advance(1.7e9), 0.0);
}

TEST(ForwardClock, forwardJumpPassesThrough) {
    ForwardClock c;
    EXPECT_EQ(c.advance(10.0), 0.0);
    EXPECT_EQ(c.advance(10.5), 0.5);
    EXPECT_EQ(c.advance(20.5), 10.5);
}
