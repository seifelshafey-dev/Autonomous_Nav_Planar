/**
 * @file config_loader.cpp
 * @author Seif ElShafey
 * @brief YAML parser utility for typed configuration structs used across the path planner.
 * @version 0.2
 * @date 2026-06-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <cstdint>
#include <string>

#include "core/config/config_loader.hpp"
#include "core/diagnostics/check.hpp"

RobotParams loadRobotParams(const std::string & file_path){
    YAML::Node config = YAML::LoadFile(file_path);

    std::vector<Vec2> vertices;

    for(const auto & x: config["vertices"]){
        vertices.emplace_back(x[0].as<float>(), x[1].as<float>());
    }

    return buildRobotParams(std::move(vertices));
}

DiffDriveParams loadDiffDriveParams(const std::string & file_path){
    YAML::Node dd = YAML::LoadFile(file_path)["diff_drive"];
    DiffDriveParams p;
    p.rmin        = dd["rmin"].as<double>();
    p.vmax        = dd["vmax"].as<double>();
    p.atomic_time = dd["atomic_time"].as<double>(p.atomic_time);
    p.n_headings  = dd["n_headings"].as<int>(p.n_headings);

    CHECK(p.rmin > 0.0, "diff_drive.rmin must be positive");
    CHECK(p.vmax > 0.0, "diff_drive.vmax must be positive");
    CHECK(p.atomic_time > 0.0, "diff_drive.atomic_time must be positive");
    CHECK(p.n_headings >= 1, "diff_drive.n_headings must be at least 1");
    return p;
}

CostPolicy loadCostPolicy(const std::string & file_path){
    YAML::Node node = YAML::LoadFile(file_path)["cost_policy"];
    CostPolicy p;
    p.clearance_radius_   = node["clearance_radius"].as<float>();
    p.clearance_decay_    = node["clearance_decay"].as<float>();
    p.max_clearance_cost_ = node["max_clearance_cost"].as<float>();
    CHECK(!node["unknown_cost"],
          "cost_policy.unknown_cost is no longer accepted; use cost_policy.unknown_cost_factor (>= 1)");
    p.unknown_cost_factor_ = node["unknown_cost_factor"].as<float>();

    CHECK(p.clearance_radius_ >= 0.0f, "cost_policy.clearance_radius must be non-negative");
    CHECK(p.clearance_decay_ >= 0.0f, "cost_policy.clearance_decay must be non-negative");
    CHECK(p.max_clearance_cost_ >= 0.0f, "cost_policy.max_clearance_cost must be non-negative");
    CHECK(std::isfinite(p.unknown_cost_factor_) && p.unknown_cost_factor_ >= 1.0f,
          "cost_policy.unknown_cost_factor must be finite and at least 1");
    return p;
}

AStarConfig loadAStarConfig(const std::string & file_path){
    YAML::Node node = YAML::LoadFile(file_path)["a_star"];
    AStarConfig c;
    c.goal_pos_tol       = node["goal_pos_tol"].as<double>(c.goal_pos_tol);
    c.goal_heading_tol   = node["goal_heading_tol"].as<double>(c.goal_heading_tol);
    c.max_expansions     = node["max_expansions"].as<int32_t>(c.max_expansions);
    c.max_plan_time_sec  = node["max_plan_time_sec"].as<double>(c.max_plan_time_sec);
    c.enable_shot        = node["enable_shot"].as<bool>(c.enable_shot);
    c.shot_max_distance  = node["shot_max_distance"].as<double>(c.shot_max_distance);
    c.shot_allow_unknown = node["shot_allow_unknown"].as<bool>(c.shot_allow_unknown);
    c.closed_cell_size   = node["closed_cell_size"].as<double>(c.closed_cell_size);
    c.node_pool_reserve  = node["node_pool_reserve"].as<int32_t>(c.node_pool_reserve);
    c.closed_set_reserve = node["closed_set_reserve"].as<int32_t>(c.closed_set_reserve);
    c.costmap_weight     = node["costmap_weight"].as<double>(c.costmap_weight);

    // Tolerances, expansion cap and time budget must be strictly positive (a zero budget/tolerance
    // is a misconfiguration); reserves, shot distance and costmap weight only need to be non-negative.
    CHECK(c.goal_pos_tol > 0.0, "a_star.goal_pos_tol must be positive");
    CHECK(c.goal_heading_tol > 0.0, "a_star.goal_heading_tol must be positive");
    CHECK(c.max_expansions > 0, "a_star.max_expansions must be positive");
    CHECK(c.max_plan_time_sec > 0.0, "a_star.max_plan_time_sec must be positive");
    CHECK(c.closed_cell_size > 0.0, "a_star.closed_cell_size must be positive");
    CHECK(c.shot_max_distance >= 0.0, "a_star.shot_max_distance must be non-negative");
    CHECK(c.node_pool_reserve >= 0, "a_star.node_pool_reserve must be non-negative");
    CHECK(c.closed_set_reserve >= 0, "a_star.closed_set_reserve must be non-negative");
    CHECK(c.costmap_weight >= 0.0, "a_star.costmap_weight must be non-negative");
    return c;
}

HeuristicConfig loadHeuristicConfig(const std::string & file_path){
    YAML::Node node = YAML::LoadFile(file_path)["heuristic"];
    HeuristicConfig c;
    if (node) {
        c.weight = node["weight"].as<double>(c.weight);
        c.use_obstacle_heuristic = node["use_obstacle_heuristic"].as<bool>(c.use_obstacle_heuristic);
    }
    CHECK(c.weight > 0.0, "heuristic.weight must be positive (1.0 = unscaled estimate; >1 = greedier search)");
    return c;
}

ControllerParams loadControllerParams(const std::string & file_path){
    YAML::Node node = YAML::LoadFile(file_path)["controller"];
    ControllerParams p;
    p.lookahead_gain   = node["lookahead_gain"].as<double>();
    p.v_cruise         = node["v_cruise"].as<double>();
    p.a_lat_max        = node["a_lat_max"].as<double>();
    p.omega_pivot      = node["omega_pivot"].as<double>();
    p.goal_pos_tol     = node["goal_pos_tol"].as<double>();
    p.goal_heading_tol = node["goal_heading_tol"].as<double>();

    CHECK(p.lookahead_gain > 0.0, "controller.lookahead_gain must be positive");
    CHECK(p.v_cruise > 0.0, "controller.v_cruise must be positive");
    CHECK(p.a_lat_max > 0.0, "controller.a_lat_max must be positive");
    CHECK(p.omega_pivot > 0.0, "controller.omega_pivot must be positive");
    CHECK(p.goal_pos_tol > 0.0, "controller.goal_pos_tol must be positive");
    CHECK(p.goal_heading_tol > 0.0, "controller.goal_heading_tol must be positive");
    return p;
}

ScanGuardParams loadScanGuardParams(const std::string & file_path){
    YAML::Node node = YAML::LoadFile(file_path)["scan_guard"];
    ScanGuardParams p;
    // Every key is required (a silent default here would hide a misconfigured safety backstop):
    // a missing/null key throws via the default-less as<>() conversion, matching loadCostPolicy.
    p.linear_decel          = node["linear_decel"].as<double>();
    p.angular_decel         = node["angular_decel"].as<double>();
    p.forward_pad           = node["forward_pad"].as<double>();
    p.side_pad              = node["side_pad"].as<double>();
    p.stop_count            = node["stop_count"].as<int>();
    p.clear_count           = node["clear_count"].as<int>();
    p.stale_scan_timeout    = node["stale_scan_timeout"].as<double>();
    p.tf_lookup_timeout     = node["tf_lookup_timeout"].as<double>();

    CHECK(std::isfinite(p.linear_decel) && p.linear_decel > 0.0,
          "scan_guard.linear_decel must be finite and > 0 (m/s^2)");
    CHECK(std::isfinite(p.angular_decel) && p.angular_decel > 0.0,
          "scan_guard.angular_decel must be finite and > 0 (rad/s^2)");
    CHECK(std::isfinite(p.forward_pad) && p.forward_pad >= 0.0,
          "scan_guard.forward_pad must be finite and >= 0 (m)");
    CHECK(std::isfinite(p.side_pad) && p.side_pad > 0.0,
          "scan_guard.side_pad must be finite and > 0 (m)");
    CHECK(p.stop_count >= 1, "scan_guard.stop_count must be at least 1");
    CHECK(p.clear_count >= 1, "scan_guard.clear_count must be at least 1");
    CHECK(p.stale_scan_timeout > 0.0, "scan_guard.stale_scan_timeout must be positive");
    CHECK(p.tf_lookup_timeout > 0.0, "scan_guard.tf_lookup_timeout must be positive");
    return p;
}

NavigatorParams loadNavigatorParams(const std::string & file_path){
    YAML::Node node = YAML::LoadFile(file_path)["navigator"];
    NavigatorParams p;
    // Every key is required: a missing/null key throws via the default-less as<>() conversion.
    p.tick_period_sec       = node["tick_period_sec"].as<double>();
    p.stall_timeout         = node["stall_timeout"].as<double>();
    p.blocked_retry_period  = node["blocked_retry_period"].as<double>();
    p.max_clearing_attempts = node["max_clearing_attempts"].as<int>();
    p.max_stall_replans     = node["max_stall_replans"].as<int>();
    p.escape_speed_scale    = node["escape_speed_scale"].as<double>();
    YAML::Node e = node["escape"];
    p.escape.step_fraction     = e["step_fraction"].as<double>();
    p.escape.slack_cells       = e["slack_cells"].as<int>();
    p.escape.max_travel_factor = e["max_travel_factor"].as<double>();

    CHECK(std::isfinite(p.tick_period_sec) && p.tick_period_sec > 0.0,
          "navigator.tick_period_sec must be finite and positive");
    CHECK(std::isfinite(p.stall_timeout) && p.stall_timeout > 0.0,
          "navigator.stall_timeout must be finite and positive");
    CHECK(p.stall_timeout > p.tick_period_sec,
          "navigator.stall_timeout must be greater than navigator.tick_period_sec");
    CHECK(std::isfinite(p.blocked_retry_period) && p.blocked_retry_period > 0.0,
          "navigator.blocked_retry_period must be finite and positive");
    CHECK(p.max_clearing_attempts >= 1, "navigator.max_clearing_attempts must be at least 1");
    CHECK(p.max_stall_replans >= 0, "navigator.max_stall_replans must be at least 0");
    CHECK(std::isfinite(p.escape_speed_scale) && p.escape_speed_scale > 0.0 && p.escape_speed_scale <= 1.0,
          "navigator.escape_speed_scale must be in (0, 1]");
    CHECK(std::isfinite(p.escape.step_fraction) && p.escape.step_fraction > 0.0 && p.escape.step_fraction < 1.0,
          "navigator.escape.step_fraction must be in (0, 1)");
    CHECK(p.escape.slack_cells >= 0, "navigator.escape.slack_cells must be non-negative");
    CHECK(std::isfinite(p.escape.max_travel_factor) && p.escape.max_travel_factor > 0.0,
          "navigator.escape.max_travel_factor must be finite and positive");
    return p;
}
