/**
 * @file path_validity.hpp
 * @author Seif ElShafey
 * @brief Collision re-check of a planned path against a collision checker.
 * @version 0.1
 * @date 2026-06-02
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include "core/planner/path.hpp"
#include "core/collision_checker/collision_checker.hpp"

/**
 * @brief Test whether every sampled pose along a path is collision-free.
 * @param path A planned path; its segments' densely-sampled poses are checked in order.
 * @param checker Collision oracle to validate each pose against.
 * @return true if no sampled pose collides; false at the first colliding pose.
 */
bool pathValid(const Path & path, const CollisionChecker & checker);
