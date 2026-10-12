#include <gtest/gtest.h>
#include <vector>
#include "core/map/map_signature.hpp"

TEST(MapSignature, identicalDataHashesEqual){
    std::vector<int8_t> a{0, 100, -1, 50, 0};
    std::vector<int8_t> b{0, 100, -1, 50, 0};
    EXPECT_EQ(hashOccupancy(a), hashOccupancy(b));
}

TEST(MapSignature, oneCellDifferenceChangesHash){
    std::vector<int8_t> a{0, 0, 0, 0, 0};
    std::vector<int8_t> b{0, 0, 1, 0, 0};
    EXPECT_NE(hashOccupancy(a), hashOccupancy(b));
}

TEST(MapSignature, identicalMapsCompareEqual){
    std::vector<int8_t> d{0, 100, -1};
    auto s1 = makeSignature(3, 1, 0.05f, 0.0f, 0.0f, d);
    auto s2 = makeSignature(3, 1, 0.05f, 0.0f, 0.0f, d);
    EXPECT_TRUE(s1 == s2);
}

TEST(MapSignature, originShiftCompareUnequal){
    std::vector<int8_t> d{0, 100, -1};
    auto s1 = makeSignature(3, 1, 0.05f, 0.0f, 0.0f, d);
    auto s2 = makeSignature(3, 1, 0.05f, 1.0f, 0.0f, d);  // same cells, shifted origin
    EXPECT_TRUE(s1 != s2);
}

TEST(MapSignature, emptyDataHashIsStable){
    EXPECT_EQ(hashOccupancy({}), hashOccupancy({}));
}
