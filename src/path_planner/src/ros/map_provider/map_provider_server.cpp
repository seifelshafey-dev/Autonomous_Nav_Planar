/**
 * @file map_provider_server.cpp
 * @author Seif ElShafey
 * @brief Implementation for the ROS2 Map server node methods
 * @version 0.1
 * @date 2026-05-29
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <cmath>
#include <limits>
#include <Eigen/Core>
#include "ros/map_provider/map_provider_server.hpp"
#include "core/map/map_signature.hpp"

CallbackReturn ROSMapProvider::on_configure(const State&){
    topic_ = get_parameter("map_topic").as_string();
    max_edt_dist_ = get_parameter("edt_max_dist").as_double();
    if (!(max_edt_dist_ > 0.0)) {
        RCLCPP_ERROR(get_logger(), "edt_max_dist must be positive (m), got %f", max_edt_dist_);
        return CallbackReturn::FAILURE;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn ROSMapProvider::on_activate(const State&){
    sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(topic_, rclcpp::QoS(1).transient_local(),
            [this] (nav_msgs::msg::OccupancyGrid::UniquePtr msg) {onMap(std::move(msg));} );

    bond_ = std::make_unique<bond::Bond>(
        "bond", get_name(),
        get_node_base_interface(), get_node_logging_interface(),
        get_node_parameters_interface(), get_node_timers_interface(),
        get_node_topics_interface());
    bond_->setHeartbeatPeriod(1.0);   // nav2_util default (DEFAULT_HEARTBEAT_PERIOD)
    bond_->setHeartbeatTimeout(4.0);  // nav2_util default (DEFAULT_HEARTBEAT_TIMEOUT)
    bond_->start();

    return CallbackReturn::SUCCESS;
}

CallbackReturn ROSMapProvider::on_deactivate(const State&){
    if (bond_) { bond_->breakBond(); bond_.reset(); }
    sub_.reset();
    return CallbackReturn::SUCCESS;
}

// The subscription is already released; resetting the params matters little — the point is to drop latest_ so no stale map lingers to be read.
CallbackReturn ROSMapProvider::on_cleanup(const State&){
    if (bond_) { bond_->breakBond(); bond_.reset(); }
    latest_.store(nullptr);
    has_sig_ = false;
    last_sig_ = MapSignature{};
    topic_.clear(); max_edt_dist_ = 0;
    return CallbackReturn::SUCCESS;
}

void ROSMapProvider::onMap(nav_msgs::msg::OccupancyGrid::UniquePtr msg){
    const auto & info = msg->info;

    const bool sized = info.width > 0 && info.height > 0 &&
                       info.width <= static_cast<uint32_t>(std::numeric_limits<int>::max()) &&
                       info.height <= static_cast<uint32_t>(std::numeric_limits<int>::max());
    const bool finite = std::isfinite(info.resolution) &&
                        std::isfinite(static_cast<float>(info.origin.position.x)) &&
                        std::isfinite(static_cast<float>(info.origin.position.y));
    if (!sized || !(info.resolution > 0.0f) || !finite ||
        msg->data.size() != static_cast<std::size_t>(info.width) * static_cast<std::size_t>(info.height)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "map ignored: width %u, height %u, resolution %f, origin (%f, %f), %zu cells; need positive size, positive finite resolution, finite origin and width * height cells",
            info.width, info.height, info.resolution, info.origin.position.x, info.origin.position.y,
            msg->data.size());
        return;
    }

    // Cheap change-detection: identical geometry + content => keep the existing snapshot pointer and
    // skip the EDT rebuild. Consumers detect "no change" via snapshot() returning the same pointer.
    const MapSignature sig = makeSignature(
        static_cast<int>(info.width), static_cast<int>(info.height), info.resolution,
        static_cast<float>(info.origin.position.x),
        static_cast<float>(info.origin.position.y), msg->data);
    if (has_sig_ && sig == last_sig_) { return; }
    last_sig_ = sig;
    has_sig_  = true;

    Eigen::Vector2f orgn{
        static_cast<float>(info.origin.position.x),
        static_cast<float>(info.origin.position.y)
    };
    GridMetaData meta(static_cast<int>(info.width), static_cast<int>(info.height), info.resolution, orgn);
    auto grid = OccupancyGrid(meta, std::move(msg->data));
    auto edt  = buildEDTGrid(grid, max_edt_dist_);
    latest_.store(std::make_shared<const MapSnapshot>(std::move(grid), std::move(edt)));
}
