#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <cstdint>

#include "core/planner/hybrid_astar/closed_set.hpp"

/*
Property: packClosedKey is a bijection over its supported domain. Round-trip through a test-local
inverse recovers (i, j, k) for any valid triple, which simultaneously proves injectivity (no two
distinct valid triples collide) and the exact bit layout. Generators are bounded to the supported
range so we stay clear of the documented out-of-range asserts (covered by death tests instead).
*/

namespace {

struct Unpacked { int32_t i, j, k; };

Unpacked unpackClosedKey(ClosedKey key){
    const uint32_t k  = static_cast<uint32_t>(key & 0xFFFFu);
    const uint32_t uj = static_cast<uint32_t>((key >> K_BITS) & 0xFFFFFFu);
    const uint32_t ui = static_cast<uint32_t>((key >> (J_BITS + K_BITS)) & 0xFFFFFFu);
    return Unpacked{static_cast<int32_t>(ui) - I_OFFSET,
                    static_cast<int32_t>(uj) - J_OFFSET,
                    static_cast<int32_t>(k)};
}

}  // namespace

RC_GTEST_PROP(PackClosedKeyProp, roundTripsOverSupportedDomain, ()){
    const int32_t i = *rc::gen::inRange<int32_t>(-I_OFFSET, I_OFFSET);  // [-2^23, 2^23)
    const int32_t j = *rc::gen::inRange<int32_t>(-J_OFFSET, J_OFFSET);
    const int32_t k = *rc::gen::inRange<int32_t>(0, K_MAX);             // [0, 2^16)

    Unpacked u = unpackClosedKey(packClosedKey(i, j, k));
    RC_ASSERT(u.i == i);
    RC_ASSERT(u.j == j);
    RC_ASSERT(u.k == k);
}
