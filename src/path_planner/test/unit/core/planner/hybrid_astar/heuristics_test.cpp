#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "core/planner/hybrid_astar/heuristics.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/types/state.hpp"

/*
DistanceHeuristic's own logic is thin: prepare(goal) caches the goal, operator()(current) returns
weight * dist(current, goal). The injected SE2Distance (PointsWeightedEuclideanSE2) is already tested,
so here we check the wrapper: weight application, goal binding/rebinding, and zero-at-goal.
Distance values are hand-computed for the chosen metric weight.
*/

namespace {

constexpr double METRIC_W = 0.25;  // PointsWeightedEuclideanSE2 rotational weight (e.g. rmin^2)

std::shared_ptr<const SE2Distance> metric(){
    return std::make_shared<const PointsWeightedEuclideanSE2>(METRIC_W);
}

}  // namespace

TEST(DistanceHeuristic, equalsRawDistanceAtWeightOne){
    DistanceHeuristic h(metric(), 1.0);
    h.prepare(SE2State{3.0, 4.0, 0.0});
    // dist from origin to (3,4,0): sqrt(9 + 16 + 0) = 5.
    EXPECT_DOUBLE_EQ(h(SE2State{0.0, 0.0, 0.0}), 5.0);
}

TEST(DistanceHeuristic, multipliesRawDistanceByWeight){
    DistanceHeuristic h(metric(), 2.5);
    h.prepare(SE2State{3.0, 4.0, 0.0});
    EXPECT_DOUBLE_EQ(h(SE2State{0.0, 0.0, 0.0}), 2.5 * 5.0);
}

TEST(DistanceHeuristic, accountsForWeightedHeading){
    DistanceHeuristic h(metric(), 1.0);
    h.prepare(SE2State{0.0, 0.0, 1.0});
    // dist: sqrt(0 + 0 + 0.25 * wrapToPi(1)^2) = 0.5 * 1 = 0.5.
    EXPECT_DOUBLE_EQ(h(SE2State{0.0, 0.0, 0.0}), 0.5);
}

TEST(DistanceHeuristic, isZeroAtTheGoal){
    DistanceHeuristic h(metric(), 3.0);
    SE2State goal{1.0, 2.0, 0.5};
    h.prepare(goal);
    EXPECT_DOUBLE_EQ(h(goal), 0.0);
}

TEST(DistanceHeuristic, prepareRebindsTheGoal){
    DistanceHeuristic h(metric(), 1.0);
    h.prepare(SE2State{10.0, 0.0, 0.0});
    EXPECT_DOUBLE_EQ(h(SE2State{0.0, 0.0, 0.0}), 10.0);
    h.prepare(SE2State{0.0, 6.0, 0.0});  // rebind
    EXPECT_DOUBLE_EQ(h(SE2State{0.0, 0.0, 0.0}), 6.0);
}

/*
MaxHeuristic combines children by taking their per-call maximum and forwards prepare() to all of them.
Controlled real children (DistanceHeuristic) with known outputs serve as the oracle.
*/

TEST(MaxHeuristic, returnsTheLargerChild){
    auto d = metric();
    MaxHeuristic mh({std::make_shared<const DistanceHeuristic>(d, 1.0),
                     std::make_shared<const DistanceHeuristic>(d, 3.0)});
    mh.prepare(SE2State{3.0, 4.0, 0.0});       // raw dist from origin = 5
    EXPECT_DOUBLE_EQ(mh(SE2State{0.0, 0.0, 0.0}), 3.0 * 5.0);  // max(5, 15)
}

TEST(MaxHeuristic, prepareForwardsToAllChildren){
    auto h1 = std::make_shared<const DistanceHeuristic>(metric(), 1.0);
    auto h2 = std::make_shared<const DistanceHeuristic>(metric(), 2.0);
    MaxHeuristic mh({h1, h2});
    SE2State goal{1.0, 2.0, 0.5};
    mh.prepare(goal);                          // must bind the goal on every child
    EXPECT_DOUBLE_EQ((*h1)(goal), 0.0);
    EXPECT_DOUBLE_EQ((*h2)(goal), 0.0);
}

