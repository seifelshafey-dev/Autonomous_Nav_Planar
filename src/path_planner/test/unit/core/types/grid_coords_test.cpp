#include <gtest/gtest.h>
#include "core/types/grid_coords.hpp"

/*
GridCoords operator contract:
    Invariant: i (column / x) and j (row / y) are integers (may be negative). OOB handling
               relative to any grid is NOT this layer's concern.
    ==        : true iff both i and j match (same cell in space).
    <=> / </> : lexicographic, COLUMN-FIRST. Compare i; on a tie compare j.
    +         : componentwise integer addition.
*/

// ---- Equality ----

TEST(GridCoords, equalWhenBothMatch){
    ASSERT_TRUE((GridCoords{3, 4} == GridCoords{3, 4}));
}

TEST(GridCoords, notEqualWhenColumnDiffers){
    ASSERT_FALSE((GridCoords{3, 4} == GridCoords{5, 4}));
    ASSERT_TRUE((GridCoords{3, 4} != GridCoords{5, 4}));
}

TEST(GridCoords, notEqualWhenRowDiffers){
    ASSERT_FALSE((GridCoords{3, 4} == GridCoords{3, 9}));
    ASSERT_TRUE((GridCoords{3, 4} != GridCoords{3, 9}));
}

// ---- Ordering: column (i) is the primary key ----

TEST(GridCoords, columnDominatesOrdering){
    // Larger column wins regardless of row.
    ASSERT_TRUE((GridCoords{1, -1000} > GridCoords{0, 1000}));
    ASSERT_TRUE((GridCoords{-1, 50} < GridCoords{0, -50}));
}

TEST(GridCoords, rowBreaksTieWhenColumnsEqual){
    ASSERT_TRUE((GridCoords{5, 2} > GridCoords{5, 1}));
    ASSERT_TRUE((GridCoords{5, 1} < GridCoords{5, 2}));
}

// ---- Addition ----

TEST(GridCoords, additionWithNegatives){
    ASSERT_TRUE((GridCoords{-1, -2} + GridCoords{1, 2} == GridCoords{0, 0}));
}

// Addition is commutative, associative, and {0,0} is its identity (asymmetric operands so a
// swapped-component bug is visible).
TEST(GridCoords, additionCommutesAssociatesAndHasZeroIdentity){
    const GridCoords a{1, -2}, b{30, 4}, c{-500, 6};
    ASSERT_TRUE((a + b == b + a));
    ASSERT_TRUE(((a + b) + c == a + (b + c)));
    ASSERT_TRUE((a + GridCoords{0, 0} == a));
}
