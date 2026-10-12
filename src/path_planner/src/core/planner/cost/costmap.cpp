/**
 * @file costmap.cpp
 * @author Seif ElShafey
 * @brief Implementation for the costmap methods (costmap.hpp)
 * @version 0.1
 * @date 2026-05-29
 * 
 * @copyright Copyright (c) 2026
 * 
 */

 #include <algorithm>
 #include <cmath>
 #include <cstdint>
 #include <Eigen/Core>
 #include "core/planner/cost/costmap.hpp"
 #include "core/diagnostics/check.hpp"

 CostMap buildCostMap(const MapSnapshot& map_, const CostPolicy &p_, double r_in, double costmap_weight){
    const auto& meta = map_.grid_.getMetadata();
    const int n = meta.rows * meta.cols;

    CHECK(map_.edt_.getData().size() == static_cast<std::size_t>(n),
          "EDT distance field size must equal grid cell count");
    CHECK(p_.clearance_radius_ >= 0.0f && p_.clearance_decay_ >= 0.0f &&
          p_.max_clearance_cost_ >= 0.0f,
          "CostPolicy clearance parameters must be non-negative");
    CHECK(std::isfinite(p_.unknown_cost_factor_) && p_.unknown_cost_factor_ >= 1.0f,
          "CostPolicy unknown_cost_factor must be finite and at least 1");
    CHECK(std::isfinite(r_in) && r_in > 0.0, "r_in must be positive and finite");
    CHECK(std::isfinite(costmap_weight) && costmap_weight >= 0.0, "costmap_weight must be non-negative and finite");

    // Dearest known-free clearance cost: smallest distance-map value a collision-free centre can have.
    const double d_min = std::max(0.0, r_in - std::sqrt(2.0) / 2.0 * static_cast<double>(meta.resolution));
    const double b = (d_min < p_.clearance_radius_)
                         ? static_cast<double>(p_.max_clearance_cost_) * std::exp(-static_cast<double>(p_.clearance_decay_) * d_min)
                         : 0.0;
    const double u_d = costmap_weight > 0.0
                           ? (static_cast<double>(p_.unknown_cost_factor_) * (1.0 + costmap_weight * b) - 1.0) / costmap_weight
                           : 0.0;
    const float u = static_cast<float>(u_d);

    auto edt_arr  = Eigen::Map<const Eigen::ArrayXf>(map_.edt_.getData().data(), n); // Read in place (no copy)
    auto grid_arr = Eigen::Map<const Eigen::Array<int8_t, Eigen::Dynamic, 1>>(map_.grid_.getData().data(), n);

    std::vector<float> cost(n);

    auto cost_arr = Eigen::Map<Eigen::ArrayXf>(cost.data() ,n); // Write in place

    cost_arr = (edt_arr < p_.clearance_radius_).select(
        p_.max_clearance_cost_ * (edt_arr * -p_.clearance_decay_).exp(),
        Eigen::ArrayXf::Zero(n)
    ); // Binary mask in Eigen uses SIMD, better than an if branch

    cost_arr += (grid_arr == -1).cast<float>() * u;

    return CostMap{meta, std::move(cost), u};
 }