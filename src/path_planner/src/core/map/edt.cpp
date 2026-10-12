/**
 * @file edt.cpp
 * @author Seif ElShafey
 * @brief Implements the EDT factory function in edt.hpp.
 * @version 0.1
 * @date 2026-05-24
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <cstdint>
#include <cstring>
#include <opencv2/core/mat.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include "core/map/edt.hpp"

EDT buildEDTGrid(const OccupancyGrid & cs_grid, double edt_clamp){
    CHECK(edt_clamp > 0.0, "edt_clamp must be positive");
    const auto & meta = cs_grid.getMetadata();
    const int n = meta.rows * meta.cols;

    std::vector<uint8_t> obstacles(n);

    bool has_obstacle = false;
    for(int i = 0; i<n; i++){
        obstacles[i] = (cs_grid.rawAccess(i) >= meta.occupied_threshold) ? 0 : 255;
        if(obstacles[i] == 0) has_obstacle = true;
    }

    // With no obstacle (zero) pixel, cv::distanceTransform's output is unspecified. Every cell is
    // then maximally far from any obstacle, so report the clamp distance everywhere.
    if(!has_obstacle){
        return EDT{meta, std::vector<float>(n, static_cast<float>(edt_clamp)), edt_clamp};
    }

    cv::Mat mask(meta.rows, meta.cols, CV_8UC1, obstacles.data());

    cv::Mat dist_;
    cv::distanceTransform(mask, dist_, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    // distanceTransform returns distances in pixel units; convert to meters using the grid resolution.
    dist_ *= meta.resolution;

    std::vector<float> dfield_(n);
    std::memcpy(dfield_.data(), dist_.data, n * sizeof(float));

    return EDT{meta, std::move(dfield_), edt_clamp};
}