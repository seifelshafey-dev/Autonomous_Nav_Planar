/**
 * @file map_signature.hpp
 * @author Seif ElShafey
 * @brief Cheap content+geometry fingerprint of an occupancy grid, used to detect whether a freshly
 *        received map differs from the last one before doing expensive downstream work.
 * @version 0.1
 * @date 2026-05-24
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <cstdint>
#include <vector>

/**
 * @brief Geometry + content fingerprint of an occupancy grid. Two grids with equal signatures are
 *        treated as the same map.
 */
struct MapSignature {
    int cols = 0;          ///< Grid width in cells.
    int rows = 0;          ///< Grid height in cells.
    float resolution = 0;  ///< Cell size in meters.
    float origin_x = 0;    ///< World x of the grid origin in meters.
    float origin_y = 0;    ///< World y of the grid origin in meters.
    uint64_t hash = 0;     ///< Content hash of the occupancy cells.

    /**
     * @brief Equal if geometry fields and content hash all compare equal.
     * @param o Signature to compare with.
     * @return true if all fields are equal.
     */
    bool operator==(const MapSignature & o) const {
        return cols == o.cols && rows == o.rows && resolution == o.resolution
            && origin_x == o.origin_x && origin_y == o.origin_y && hash == o.hash;
    }
    /**
     * @brief Negation of `operator==`.
     * @param o Signature to compare with.
     * @return true if any field differs.
     */
    bool operator!=(const MapSignature & o) const { return !(*this == o); }
};

/**
 * @brief Content hash of an occupancy-cell buffer.
 * @param data Row-major occupancy cells.
 * @return A 64-bit hash; equal buffers hash equal, and differing buffers hash differently with
 *         overwhelming probability.
 */
uint64_t hashOccupancy(const std::vector<int8_t> & data);

/**
 * @brief Build a signature from a grid's geometry and cells.
 * @param cols Grid width in cells.
 * @param rows Grid height in cells.
 * @param resolution Cell size in meters.
 * @param origin_x World x of the grid origin in meters.
 * @param origin_y World y of the grid origin in meters.
 * @param data Row-major occupancy cells.
 * @return The combined geometry+content signature.
 */
MapSignature makeSignature(int cols, int rows, float resolution,
                           float origin_x, float origin_y,
                           const std::vector<int8_t> & data);
