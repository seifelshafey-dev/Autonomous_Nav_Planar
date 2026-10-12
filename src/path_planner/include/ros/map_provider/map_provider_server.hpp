/**
 * @file map_provider_server.hpp
 * @author Seif ElShafey
 * @brief Interface for a ROS2 node that reads the map from SLAM and adapts it to our core C++ Map Structure (map_provider.hpp)
 * @version 0.1
 * @date 2026-05-29
 * 
 * @copyright Copyright (c) 2026
 * 
 */

 #pragma once

 #include <string>
 #include <atomic>
 #include <memory>

 #include "core/map/map_provider.hpp"
 #include "core/map/map_signature.hpp"
 #include <nav_msgs/msg/occupancy_grid.hpp>
 #include "rclcpp_lifecycle/lifecycle_node.hpp"
 #include "rclcpp/rclcpp.hpp"
 #include "bondcpp/bond.hpp"

 using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;  ///< Result of a lifecycle transition callback.
 using State = rclcpp_lifecycle::State;  ///< Lifecycle state passed to a transition callback.
/**
 * @brief Interface for a spinnable ROS2 Node that subscribes to SLAM occupancy map messages and transforms them into the core C++ Grid + EDT structure used by the core package.

 @note This node is to be composed in the same process as the planner nodes, sending the map structure through atomic pointers, not ROS communication.
 *
 * A map with zero width or height, a non-positive or NaN resolution, a resolution or origin that is not finite (the origin is stored in single precision, so a coordinate outside the single-precision range counts as not finite), or a data array whose length is not width * height is ignored with a warning; the previous snapshot stays.
 *
 * @par Parameters
 * - `map_topic` (string, default `/map`): occupancy grid topic.
 * - `edt_max_dist` (double, m, default 5.0): distance-field clamp; must be positive or configuration fails.
 */

 class ROSMapProvider : public rclcpp_lifecycle::LifecycleNode, public IMapProvider{

    public:
    /**
     * @brief Construct a new ROSMapProvider object as a ROS2 lifecycle node.
     * 
     * @param opts Configuration parameters for the lifecycle node. Defaulted as empty (no explicit changes).
     */
        explicit ROSMapProvider(const rclcpp::NodeOptions & opts) : LifecycleNode("map_server", opts) {
            // Declared once here (not in on_configure) so configure -> cleanup -> configure is
            // idempotent: rclcpp throws if the same parameter is declared twice, and on_cleanup
            // does not undeclare. Parameters persist for the node's lifetime; on_configure only
            // reads them via get_parameter.
            declare_parameter<std::string>("map_topic", "/map");
            declare_parameter<double>("edt_max_dist", 5.0);
        }

        /**
         * @brief Returns the latest MapSnapshot built from the SLAM map; safe to call from any thread.
         * 
         * @return std::shared_ptr<const MapSnapshot> The latest grid + edt received and built from SLAM; null before the first map arrives and after cleanup. A map identical to the previous one leaves the same pointer.
         */
        inline std::shared_ptr<const MapSnapshot> snapshot() const override {
            return latest_.load();
        }

        /**
         * @brief Loads parameters, defines the subscription model, and readies the node for activation. Provides smooth spinup.
         * 
         * @return CallbackReturn `SUCCESS`, or `FAILURE` if `edt_max_dist` is not positive (or is NaN).
         */
        CallbackReturn on_configure(const State&) override;

        /**
         * @brief Triggers the topic subscription and the consequent node action on message reception.
         * 
         * @return CallbackReturn Enumerated variable for success validation.
         */

        CallbackReturn on_activate(const State&) override;

        /**
         * @brief Releases the map subscription and the heartbeat bond; the stored snapshot is kept.
         * 
         * @return CallbackReturn 
         */

        CallbackReturn on_deactivate(const State&) override;

        /**
         * @brief Drops the stored snapshot (snapshot() returns null again) and clears the loaded map topic and EDT distance settings. Prepares the node for deletion.
         * 
         * @return CallbackReturn 
         */

        CallbackReturn on_cleanup(const State&) override;

    private:

        void onMap(nav_msgs::msg::OccupancyGrid::UniquePtr msg); // Translates nav_msgs into C++ grid, builds EDT, and stores atomic snapshot ready to be used.

        std::string topic_;
        double max_edt_dist_ = 5.0f;
        std::atomic<std::shared_ptr<const MapSnapshot>> latest_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
        MapSignature last_sig_{};   ///< Fingerprint of the most recently stored snapshot.
        bool has_sig_ = false;      ///< Whether last_sig_ has been set yet.

        /// Heartbeat the lifecycle manager monitors after activation; the manager fails
        /// activation if its bond_timeout elapses before this bond opens. Created in
        /// on_activate, broken in on_deactivate/on_cleanup.
        std::unique_ptr<bond::Bond> bond_;

 };
