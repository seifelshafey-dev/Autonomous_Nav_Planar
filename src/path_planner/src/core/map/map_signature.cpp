/**
 * @file map_signature.cpp
 * @author Seif ElShafey
 * @brief Implements the occupancy-grid fingerprint helpers declared in map_signature.hpp.
 * @version 0.1
 * @date 2026-05-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "core/map/map_signature.hpp"

uint64_t hashOccupancy(const std::vector<int8_t> & data) {
    // FNV-1a 64-bit over the raw cell bytes.
    uint64_t h = 1469598103934665603ULL;       // offset basis
    for (int8_t c : data) {
        h ^= static_cast<uint64_t>(static_cast<uint8_t>(c));
        h *= 1099511628211ULL;                  // FNV prime
    }
    return h;
}

MapSignature makeSignature(int cols, int rows, float resolution,
                           float origin_x, float origin_y,
                           const std::vector<int8_t> & data) {
    return MapSignature{cols, rows, resolution, origin_x, origin_y, hashOccupancy(data)};
}
