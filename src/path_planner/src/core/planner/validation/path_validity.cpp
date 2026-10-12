/**
 * @file path_validity.cpp
 * @author Seif ElShafey
 * @brief Implementation of pathValid — re-checks each sampled pose of a path against a collision checker.
 * @version 0.1
 * @date 2026-06-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "core/planner/validation/path_validity.hpp"

bool pathValid(const Path & path, const CollisionChecker & checker) {
    for (const auto & segment : path.segments) {
        for (const auto & pose : segment.samples_poses) {
            if (!checker.checkState(pose)) return false;
        }
    }
    return true;
}
