/**
 * @file map_provider.hpp
 * @author Seif ElShafey
 * @brief Defines an interface for the Map snapshot (EDT + Occupancy) required by the planners, along with the abstract interface of the provider of said Map.
 * @version 0.1
 * @date 2026-05-24
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <memory>

#include "core/map/grid.hpp"
#include "core/map/edt.hpp"

/**
 * @brief A planner needs both the occupancy grid propagation and the EDT map for collision checking,
 clearance-based cost policies, and trajectory optimization. This structure bundles them for easy atomic access.
 * 
 */

struct MapSnapshot{
    OccupancyGrid grid_; ///< Occupancy grid for cell-level free / occupied / unknown queries.
    EDT edt_;            ///< Euclidean distance transform matching the grid's extent; values in meters.
};

/**
 * @brief Abstract interface defining the structure of any MapProvider. In real-world implementation,
 the map provider will be defined as a ROS node subscribed to SLAM. However, this interface provides flexibility.
 to choose any provider.
 */

class IMapProvider{
    public:
        virtual ~IMapProvider() = default;

        /**
         * @brief Return the most recent map snapshot from the provider.
         *
         * @return std::shared_ptr<const MapSnapshot> Shared pointer to an immutable bundle of
         *         occupancy grid + EDT; may be replaced atomically by the provider as new SLAM
         *         updates arrive.
         */
        virtual std::shared_ptr<const MapSnapshot> snapshot() const = 0;
};