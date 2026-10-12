/**
 * @file collision_checker.cpp
 * @author Seif ElShafey
 * @brief Implementation of the collision_checker.hpp interface.
 * @version 0.1
 * @date 2026-05-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Core>
#include "core/collision_checker/collision_checker.hpp"
#include "core/motion/world_robot_geometry.hpp"
#include "core/map/rasterize.hpp"
#include "core/types/grid_coords.hpp"
#include "core/diagnostics/check.hpp"

Validity CollisionChecker::fastAcceptReject(const SE2State & current_state) const {
    auto current_state_grid = this->grid_->getMetadata().worldToGrid(current_state.x, current_state.y);
    return fastAcceptReject(current_state_grid);
}

Validity CollisionChecker::fastAcceptReject(const GridCoords & cell) const {
    // Out-of-grid cells have no EDT entry; reading distance_field would be an out-of-bounds access.
    // Report UNCERTAIN so callers fall back to the per-cell occupancy scan, which treats OOB as
    // UNKNOWN (not a collision). Returning VALID here would be unsafe: it would fast-accept the whole
    // trajectory off one OOB sample and skip the swept scan, masking an in-grid OCCUPIED swept cell.
    if(!this->grid_->getMetadata().inBounds(cell)) {return Validity::UNCERTAIN;}

    // The cell is the floor-cell of a continuous centre, so the centre lies within (sqrt2/2)h of the cell
    // centre; the obstacle's extent reaches (sqrt2/2)h beyond its own cell centre.
    const double edt_value = this->edt_->distanceAt(cell.i, cell.j);
    const double h = this->grid_->getMetadata().resolution;
    const double half_diag = std::sqrt(2.0) / 2.0 * h;

    if(edt_value > this->body_.r_out + 2.0 * half_diag) {return Validity::VALID;}        // Fast-accept
    else if (edt_value + half_diag < this->body_.r_in) {return Validity::COLLISION;} // Fast-reject

    return Validity::UNCERTAIN;
}


 bool CollisionChecker::checkState(const SE2State & state) const {

    const Validity status = fastAcceptReject(state);
    if(status == Validity::VALID)     {return true;}
    if(status == Validity::COLLISION) {return false;}

    // UNCERTAIN: scan the rasterized footprint per-cell.
    // Pass the map's origin to transformGeometry so the produced half-planes sit in map-grid-aligned
    // frame; the rasterizer can then floor coords directly and the resulting cells match findCellStatus.
    const auto & meta = this->grid_->getMetadata();
    WorldRobotGeometry world_geometry = transformGeometry(this->body_, state, meta.origin.cast<double>());
    std::vector<GridCoords> current_footprint = rasterizePolygon(world_geometry.half_planes, world_geometry.aabb,
                                                                 meta.resolution);

    for (const auto & x: current_footprint){
        if(this->grid_->findCellStatus(x) == CellState::OCCUPIED) {return false;}
    }

    return true;

 }

 bool CollisionChecker::startClear(const SE2State & start, double extra_margin) const {
    CHECK(std::isfinite(extra_margin) && extra_margin >= 0.0, "startClear: extra_margin must be finite and >= 0");
    const auto & meta = this->grid_->getMetadata();
    const double h = meta.resolution;
    WorldRobotGeometry world_geometry = transformGeometry(this->body_, start, meta.origin.cast<double>());
    return visitPolygonCells(world_geometry.half_planes, world_geometry.aabb, h, h / 2.0 + extra_margin,
        [&](const GridCoords & c){ return this->grid_->findCellStatus(c) != CellState::OCCUPIED; });
 }

 TrajectoryVerdict CollisionChecker::checkTrajectory(const std::vector<GridCoords> & swept_body,
                                                       const std::vector<GridCoords> & samples_body,
                                                       const std::vector<double> & sample_clearance,
                                                       const GridCoords & robot_cell) const {
        CHECK(sample_clearance.size() == samples_body.size(),
              "sample_clearance must have one entry per sample cell");
        bool need_full_scan = false;
        const auto & meta = this->grid_->getMetadata();
        const double r_in = this->body_.r_in;
        const double r_out = this->body_.r_out;

        for(std::size_t k = 0; k < samples_body.size(); ++k){
            const GridCoords world_cell = samples_body[k] + robot_cell;
            const double clearance = sample_clearance[k];
            CHECK(clearance >= r_out, "sample_clearance[" + std::to_string(k) + "] must be >= r_out");
            if(!meta.inBounds(world_cell)) {need_full_scan = true; break;}
            const double edt_value = this->edt_->distanceAt(world_cell.i, world_cell.j);
            if(edt_value + (clearance - r_out) < r_in) {
                return TrajectoryVerdict::FAST_REJECT;
            }
            if(!(edt_value > clearance)) {need_full_scan = true; break;}
        }

        // No samples means no evidence of clearance: scan.
        if(!need_full_scan && !samples_body.empty()) {return TrajectoryVerdict::FAST_ACCEPT;}

        for (const auto & body_cell : swept_body){
            const GridCoords world_cell = body_cell + robot_cell;
            if(this->grid_->findCellStatus(world_cell) == CellState::OCCUPIED) {return TrajectoryVerdict::SCAN_BLOCKED;}
        }

        return TrajectoryVerdict::SCAN_CLEAR;
}

 bool CollisionChecker::checkCell(const GridCoords & cell,
                                  bool reject_unknown) const {
        const CellState status = this->grid_->findCellStatus(cell);
        if (status == CellState::OCCUPIED) {return false;}
        return !(reject_unknown && status == CellState::UNKNOWN);
}

 bool CollisionChecker::checkCells(const std::vector<GridCoords> & cells_world,
                                   bool reject_unknown) const {
        for (const auto & cell : cells_world) {
            if (!this->checkCell(cell, reject_unknown)) {return false;}
        }
        return true;
}
