#include <gtest/gtest.h>

#include "core/planner/hybrid_astar/open_set.hpp"

/*
OpenSet is a std::priority_queue ordered by OpenEntryGreater, which must make it a MIN-heap on f
(smallest f on top). The only logic here is the comparator; a flipped comparison would turn the
queue into a max-heap and break A*'s "expand lowest f first" contract.
*/

TEST(OpenEntryGreater, ordersByAscendingF){
    OpenEntryGreater cmp;
    EXPECT_TRUE(cmp(OpenEntry{0, 3.0}, OpenEntry{0, 1.0}));   // 3 > 1 -> 3 sinks below 1
    EXPECT_FALSE(cmp(OpenEntry{0, 1.0}, OpenEntry{0, 3.0}));
    EXPECT_FALSE(cmp(OpenEntry{0, 2.0}, OpenEntry{0, 2.0}));  // equal: not greater
}

TEST(OpenSet, popsSmallestFFirst){
    OpenSet open;
    open.push(OpenEntry{10, 3.0});
    open.push(OpenEntry{11, 1.0});
    open.push(OpenEntry{12, 2.0});
    open.push(OpenEntry{13, 5.0});
    open.push(OpenEntry{14, 4.0});

    EXPECT_DOUBLE_EQ(open.top().f, 1.0); open.pop();
    EXPECT_DOUBLE_EQ(open.top().f, 2.0); open.pop();
    EXPECT_DOUBLE_EQ(open.top().f, 3.0); open.pop();
    EXPECT_DOUBLE_EQ(open.top().f, 4.0); open.pop();
    EXPECT_DOUBLE_EQ(open.top().f, 5.0); open.pop();
    EXPECT_TRUE(open.empty());
}

TEST(OpenSet, carriesNodeIndexWithPriority){
    // The entry's idx must travel with its f through the heap (top of {f=1} is idx 99).
    OpenSet open;
    open.push(OpenEntry{42, 7.0});
    open.push(OpenEntry{99, 1.0});
    EXPECT_EQ(open.top().idx, 99);
}

TEST(OpenSet, equalFAndHPopByAscendingIdx){
    OpenSet open;
    for (int idx : {5, 1, 9, 3, 7}) open.push(OpenEntry{idx, 2.0, 1.0});
    std::vector<int> got;
    while (!open.empty()){ got.push_back(open.top().idx); open.pop(); }
    EXPECT_EQ(got, (std::vector<int>{1, 3, 5, 7, 9}));
}

TEST(OpenSet, popOrderIndependentOfInsertionOrder){
    // f asc, then h asc, then idx asc.
    const std::vector<OpenEntry> base = {
        {4, 1.0, 0.5}, {2, 3.0, 2.0}, {8, 2.0, 1.5}, {1, 2.0, 0.5},
        {6, 2.0, 1.5}, {3, 2.0, 0.5}, {9, 3.0, 1.0}, {0, 5.0, 4.0}, {7, 2.0, 1.0}};
    const std::vector<int> expected = {4, 1, 3, 7, 6, 8, 9, 2, 0};
    std::vector<std::vector<int>> orders = {{0,1,2,3,4,5,6,7,8}, {8,7,6,5,4,3,2,1,0}, {3,8,0,5,2,7,4,1,6}};
    for (const auto & ord : orders){
        OpenSet open;
        for (int k : ord) open.push(base[k]);
        std::vector<int> got;
        while (!open.empty()){ got.push_back(open.top().idx); open.pop(); }
        EXPECT_EQ(got, expected);
    }
}

TEST(OpenEntryGreater, fDominatesHAndIdx){
    OpenEntryGreater cmp;
    EXPECT_TRUE(cmp(OpenEntry{0, 2.0, 0.0}, OpenEntry{9, 1.0, 9.0}));
    EXPECT_FALSE(cmp(OpenEntry{9, 1.0, 9.0}, OpenEntry{0, 2.0, 0.0}));
}
