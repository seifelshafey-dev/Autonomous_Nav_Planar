#include <gtest/gtest.h>
#include <cstdint>

#include "core/planner/hybrid_astar/closed_set.hpp"

/*
packClosedKey packs (i, j, k) into disjoint bit-fields: i -> bits[40,64), j -> [16,40), k -> [0,16).
Because the fields do not overlap, the encoding is a perfect bijection over the supported domain
(i, j in [-OFFSET, OFFSET), k in [0, K_MAX)) -- there are NO key collisions for distinct valid triples
(hash-bucket collisions are absl's concern, not tested here). We verify that bijection with a test-local
inverse (round-trip recovers the triple AND pins the exact bit layout), and verify the debug-only range
asserts fire at the boundary of the supported range (death tests; asserts are enabled in this build).
*/

namespace {

struct Unpacked { int32_t i, j, k; };

// Inverse of packClosedKey, derived directly from the documented bit layout.
Unpacked unpackClosedKey(ClosedKey key){
    const uint32_t k  = static_cast<uint32_t>(key & 0xFFFFu);                       // low 16 bits
    const uint32_t uj = static_cast<uint32_t>((key >> K_BITS) & 0xFFFFFFu);         // next 24 bits
    const uint32_t ui = static_cast<uint32_t>((key >> (J_BITS + K_BITS)) & 0xFFFFFFu); // top 24 bits
    return Unpacked{static_cast<int32_t>(ui) - I_OFFSET,
                    static_cast<int32_t>(uj) - J_OFFSET,
                    static_cast<int32_t>(k)};
}

void expectRoundTrip(int32_t i, int32_t j, int32_t k){
    Unpacked u = unpackClosedKey(packClosedKey(i, j, k));
    EXPECT_EQ(u.i, i);
    EXPECT_EQ(u.j, j);
    EXPECT_EQ(u.k, k);
}

}  // namespace

TEST(PackClosedKey, roundTripsOriginAndSmallValues){
    expectRoundTrip(0, 0, 0);
    expectRoundTrip(1, 2, 3);
    expectRoundTrip(123, 456, 7);
}

TEST(PackClosedKey, roundTripsNegativeIndices){
    // i, j may be negative (SLAM frontier expansion); the offset must preserve sign.
    expectRoundTrip(-1, -1, 0);
    expectRoundTrip(-5, 7, 9);
    expectRoundTrip(100, -200, 15);
}

TEST(PackClosedKey, roundTripsDomainBoundaries){
    expectRoundTrip(I_OFFSET - 1, J_OFFSET - 1, K_MAX - 1);  // max corner
    expectRoundTrip(-I_OFFSET, -J_OFFSET, 0);                // min corner
}

TEST(PackClosedKey, distinctTriplesYieldDistinctKeys){
    // A change in any single field must change the key (no field aliases another).
    EXPECT_NE(packClosedKey(0, 0, 0), packClosedKey(1, 0, 0));
    EXPECT_NE(packClosedKey(0, 0, 0), packClosedKey(0, 1, 0));
    EXPECT_NE(packClosedKey(0, 0, 0), packClosedKey(0, 0, 1));
    EXPECT_NE(packClosedKey(5, 9, 2), packClosedKey(9, 5, 2));  // i/j not swappable
}

// --- Range asserts (debug-only; boundary of the documented supported range) ----------------
//
// packClosedKey guards its range with assert() (not CHECK): the out-of-range triple is an internal
// invariant violation in a hot loop, and the supported signed range is practically unreachable
// (~±400 km at 0.05 m). assert() is compiled OUT under NDEBUG, so under a release build the packer
// does not abort and these death tests would "fail to die". They are therefore guarded: under NDEBUG
// each one GTEST_SKIPs (reported skipped, not failed) so the suite stays green in BOTH a default
// (asserts-live) build and a -DNDEBUG (release) build, without weakening the debug-build coverage.
// Do NOT convert these to EXPECT_THROW — assert aborts, it does not throw.
#ifdef NDEBUG
#  define EXPECT_RANGE_ASSERT_ABORTS(stmt, msg) GTEST_SKIP() << "assert() compiled out under NDEBUG"
#else
#  define EXPECT_RANGE_ASSERT_ABORTS(stmt, msg) EXPECT_DEATH(stmt, msg)
#endif

TEST(PackClosedKeyDeath, gridIAboveRangeAborts){
    EXPECT_RANGE_ASSERT_ABORTS(
        { volatile ClosedKey x = packClosedKey(I_OFFSET, 0, 0); (void)x; },
        "grid i out of packing range");
}

TEST(PackClosedKeyDeath, gridIBelowRangeAborts){
    EXPECT_RANGE_ASSERT_ABORTS(
        { volatile ClosedKey x = packClosedKey(-I_OFFSET - 1, 0, 0); (void)x; },
        "grid i out of packing range");
}

TEST(PackClosedKeyDeath, gridJAboveRangeAborts){
    EXPECT_RANGE_ASSERT_ABORTS(
        { volatile ClosedKey x = packClosedKey(0, J_OFFSET, 0); (void)x; },
        "grid j out of packing range");
}

TEST(PackClosedKeyDeath, headingBinAboveRangeAborts){
    EXPECT_RANGE_ASSERT_ABORTS(
        { volatile ClosedKey x = packClosedKey(0, 0, K_MAX); (void)x; },
        "heading bin out of packing range");
}

TEST(PackClosedKeyDeath, headingBinNegativeAborts){
    EXPECT_RANGE_ASSERT_ABORTS(
        { volatile ClosedKey x = packClosedKey(0, 0, -1); (void)x; },
        "heading bin out of packing range");
}
