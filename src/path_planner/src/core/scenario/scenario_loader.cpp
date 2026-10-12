/**
 * @file scenario_loader.cpp
 * @author Seif ElShafey
 * @brief YAML (+ optional image) loader and corpus discovery for the shared scenario corpus.
 * @version 0.1
 * @date 2026-06-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <yaml-cpp/yaml.h>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>
#include <Eigen/Core>

#include "core/scenario/scenario_loader.hpp"
#include "core/diagnostics/check.hpp"

namespace {

// Validation throws std::invalid_argument with a `file:field` prefix so a misconfigured scenario
// points the author straight at the offending entry. The CHECK macro already embeds __FILE__:__LINE__
// of this loader; prepending the scenario file + field makes both ends of the boundary visible.
#define SCENARIO_CHECK(cond, file, field, why) \
    CHECK((cond), (file) + ":" + (field) + " " + (why))

// Require a key to be present (and non-null) on a node; returns the child node.
YAML::Node require(const YAML::Node & parent, const char * key,
                   const std::string & file, const std::string & ctx) {
    YAML::Node child = parent[key];
    SCENARIO_CHECK(child && !child.IsNull(), file, ctx + "." + key, "is required");
    return child;
}

// Parse an SE2State from a YAML sequence [x, y, theta].
SE2State parsePose(const YAML::Node & node, const std::string & file, const std::string & field) {
    SCENARIO_CHECK(node.IsSequence() && node.size() == 3, file, field,
                   "must be a [x, y, theta] sequence");
    return SE2State{node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

// Paint the inclusive cell rectangle [c0, c1] x [r0, r1] with value v into a row-major buffer.
// Out-of-range corners are clamped to the grid so an author over-shooting the border is benign
// rather than a crash.
void fillRect(std::vector<int8_t> & occ, int cols, int rows,
              int c0, int r0, int c1, int r1, int8_t v) {
    if (c0 > c1) std::swap(c0, c1);
    if (r0 > r1) std::swap(r0, r1);
    c0 = std::max(c0, 0);
    r0 = std::max(r0, 0);
    c1 = std::min(c1, cols - 1);
    r1 = std::min(r1, rows - 1);
    for (int r = r0; r <= r1; ++r)
        for (int c = c0; c <= c1; ++c)
            occ[static_cast<std::size_t>(r) * cols + c] = v;
}

// Apply a list of {rect: [c0,r0,c1,r1], value: N} entries onto `occ`. `default_value` is used when a
// rect omits `value` (UNKNOWN rects default to -1; obstacle rects default to OCCUPIED).
void applyRectList(const YAML::Node & list, std::vector<int8_t> & occ, int cols, int rows,
                   int8_t default_value, const std::string & file, const std::string & field) {
    if (!list) return;
    SCENARIO_CHECK(list.IsSequence(), file, field, "must be a sequence of {rect, value} entries");
    int idx = 0;
    for (const auto & entry : list) {
        const std::string item = field + "[" + std::to_string(idx++) + "]";
        const YAML::Node rect = require(entry, "rect", file, item);
        SCENARIO_CHECK(rect.IsSequence() && rect.size() == 4, file, item + ".rect",
                       "must be a [c0, r0, c1, r1] sequence");
        const int8_t value = entry["value"]
                                 ? static_cast<int8_t>(entry["value"].as<int>())
                                 : default_value;
        fillRect(occ, cols, rows, rect[0].as<int>(), rect[1].as<int>(),
                 rect[2].as<int>(), rect[3].as<int>(), value);
    }
}

// Resolve occupancy from a grayscale image using nav2's trinary scheme. Each pixel's 0-255 grayscale
// value is compressed into a [0, 1] darkness fraction `p = (255 - px) / 255` (darker => more occupied),
// then bucketed against two thresholds:
//   p > occupied_thresh  -> OCCUPIED (100)
//   p < free_thresh      -> FREE (0)
//   otherwise            -> UNKNOWN (-1)
// Author-set keys under the `image:` node:
//   occupied_thresh  in [0, 1], default 0.65  — darkness above which a cell is OCCUPIED
//   free_thresh      in [0, 1], default 0.196 — darkness below which a cell is FREE
//   free_thresh must be < occupied_thresh.
// These cut a static grayscale image into cells; they are pixel-darkness cutoffs, not sensor-fusion
// probabilities. The image is read with rows top-to-bottom; the grid is row-major with row 0 at the
// bottom, so the image is flipped vertically to keep occupancy[r*cols + c] aligned with the
// bottom-origin grid convention used elsewhere.
std::vector<int8_t> occupancyFromImage(const YAML::Node & image_node, int cols, int rows,
                                       const std::string & file) {
    SCENARIO_CHECK(image_node.IsScalar() || (image_node.IsMap() && image_node["path"] && image_node["path"].IsScalar()),
                   file, "input.image.path", "must be a file name or a mapping with a path key");
    const std::string image_path =
        image_node.IsMap() ? image_node["path"].as<std::string>() : image_node.as<std::string>();
    SCENARIO_CHECK(!image_path.empty(), file, "input.image.path", "must be a non-empty path");

    // A relative image path is resolved against the scenario YAML file's parent directory (not the
    // process CWD), so `image: maps/depot.pgm` in `scenarios/depot_map.yaml` always resolves to
    // `scenarios/maps/depot.pgm` regardless of where the loader is invoked from. Absolute paths pass
    // through unchanged.
    std::filesystem::path resolved{image_path};
    if (resolved.is_relative()) {
        resolved = std::filesystem::path(file).parent_path() / resolved;
    }
    const std::string resolved_str = resolved.string();

    cv::Mat img = cv::imread(resolved_str, cv::IMREAD_GRAYSCALE);
    SCENARIO_CHECK(!img.empty(), file, "input.image.path",
                   "could not be read as a grayscale image: " + resolved_str);
    SCENARIO_CHECK(img.cols == cols && img.rows == rows, file, "input.image",
                   "image dimensions must equal grid cols x rows");

    const bool is_map = image_node.IsMap();
    const double occupied_thresh = is_map && image_node["occupied_thresh"]
                                       ? image_node["occupied_thresh"].as<double>()
                                       : 0.65;
    const double free_thresh = is_map && image_node["free_thresh"]
                                   ? image_node["free_thresh"].as<double>()
                                   : 0.196;
    SCENARIO_CHECK(occupied_thresh >= 0.0 && occupied_thresh <= 1.0, file,
                   "input.image.occupied_thresh", "must be in [0, 1]");
    SCENARIO_CHECK(free_thresh >= 0.0 && free_thresh <= 1.0, file,
                   "input.image.free_thresh", "must be in [0, 1]");
    SCENARIO_CHECK(free_thresh < occupied_thresh, file, "input.image.free_thresh",
                   "must be < occupied_thresh");

    std::vector<int8_t> occ(static_cast<std::size_t>(cols) * rows, 0);
    for (int r = 0; r < rows; ++r) {
        // Flip: image row 0 is the top, grid row 0 is the bottom.
        const int img_r = rows - 1 - r;
        for (int c = 0; c < cols; ++c) {
            const uint8_t px = img.at<uint8_t>(img_r, c);
            const double p = (255.0 - static_cast<double>(px)) / 255.0;  // darkness fraction
            int8_t value;
            if (p > occupied_thresh)      value = static_cast<int8_t>(100);  // OCCUPIED
            else if (p < free_thresh)     value = static_cast<int8_t>(0);    // FREE
            else                          value = static_cast<int8_t>(-1);   // UNKNOWN
            occ[static_cast<std::size_t>(r) * cols + c] = value;
        }
    }
    return occ;
}

}  // namespace

ScenarioSpec loadScenario(const std::string & path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception & e) {
        throw std::invalid_argument(path + ": failed to parse YAML: " + e.what());
    }

    const YAML::Node input = require(root, "input", path, "");

    // --- Grid geometry ---
    const YAML::Node grid = require(input, "grid", path, "input");
    const int cols = require(grid, "cols", path, "input.grid").as<int>();
    const int rows = require(grid, "rows", path, "input.grid").as<int>();
    const double resolution = require(grid, "resolution", path, "input.grid").as<double>();
    SCENARIO_CHECK(cols > 0, path, "input.grid.cols", "must be positive");
    SCENARIO_CHECK(rows > 0, path, "input.grid.rows", "must be positive");
    SCENARIO_CHECK(resolution > 0.0, path, "input.grid.resolution", "must be positive");

    const YAML::Node origin_node = grid["origin"];
    Eigen::Vector2f origin = Eigen::Vector2f::Zero();
    if (origin_node) {
        SCENARIO_CHECK(origin_node.IsSequence() && origin_node.size() == 2, path,
                       "input.grid.origin", "must be a [x, y] sequence");
        origin = Eigen::Vector2f{origin_node[0].as<float>(), origin_node[1].as<float>()};
    }

    // GridMetaData has no default ctor; construct it explicitly. occupied_threshold defaults to 50.
    GridMetaData meta{cols, rows, resolution, origin};

    // --- Occupancy: image takes precedence; otherwise inline rect lists over a FREE base. ---
    std::vector<int8_t> occupancy;
    if (input["image"]) {
        SCENARIO_CHECK(!input["obstacles"] && !input["unknown"], path, "input.image",
                       "is mutually exclusive with input.obstacles/input.unknown");
        occupancy = occupancyFromImage(input["image"], cols, rows, path);
    } else {
        occupancy.assign(static_cast<std::size_t>(cols) * rows, 0);  // base fill = FREE
        applyRectList(input["obstacles"], occupancy, cols, rows,
                      static_cast<int8_t>(100), path, "input.obstacles");
        applyRectList(input["unknown"], occupancy, cols, rows,
                      static_cast<int8_t>(-1), path, "input.unknown");
    }
    SCENARIO_CHECK(occupancy.size() == static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows),
                   path, "input", "resolved occupancy buffer size must equal cols * rows");

    // ScenarioSpec is not default-constructible (GridMetaData has no default ctor), so the leading
    // members must be supplied positionally: name, then the explicitly-built meta, then occupancy.
    // The remaining members fall back to their struct defaults and are filled in below.
    const std::string name = root["name"] ? root["name"].as<std::string>()
                                          : std::filesystem::path(path).stem().string();
    ScenarioSpec spec{name, meta, std::move(occupancy)};
    if (input["edt_clamp"]) spec.edt_clamp = input["edt_clamp"].as<double>();
    SCENARIO_CHECK(spec.edt_clamp > 0.0, path, "input.edt_clamp", "must be positive");

    // --- Robot footprint ---
    const YAML::Node robot = require(input, "robot", path, "input");
    const YAML::Node verts = require(robot, "vertices", path, "input.robot");
    SCENARIO_CHECK(verts.IsSequence() && verts.size() >= 3, path, "input.robot.vertices",
                   "must list at least 3 [x, y] vertices");
    for (const auto & v : verts) {
        SCENARIO_CHECK(v.IsSequence() && v.size() == 2, path, "input.robot.vertices",
                       "each vertex must be a [x, y] pair");
        spec.robot_vertices.emplace_back(v[0].as<double>(), v[1].as<double>());
    }

    // --- Diff-drive kinematics (mirrors loadDiffDriveParams validation) ---
    const YAML::Node dd = require(input, "diff_drive", path, "input");
    spec.diff_drive.rmin        = require(dd, "rmin", path, "input.diff_drive").as<double>();
    spec.diff_drive.vmax        = require(dd, "vmax", path, "input.diff_drive").as<double>();
    spec.diff_drive.atomic_time = dd["atomic_time"].as<double>(spec.diff_drive.atomic_time);
    spec.diff_drive.n_headings  = dd["n_headings"].as<int>(spec.diff_drive.n_headings);
    SCENARIO_CHECK(spec.diff_drive.rmin > 0.0, path, "input.diff_drive.rmin", "must be positive");
    SCENARIO_CHECK(spec.diff_drive.vmax > 0.0, path, "input.diff_drive.vmax", "must be positive");
    SCENARIO_CHECK(spec.diff_drive.atomic_time > 0.0, path, "input.diff_drive.atomic_time",
                   "must be positive");
    SCENARIO_CHECK(spec.diff_drive.n_headings >= 1, path, "input.diff_drive.n_headings",
                   "must be at least 1");

    // --- Hybrid A* config (all fields optional; struct defaults apply, mirrors loadAStarConfig) ---
    if (const YAML::Node a = input["a_star"]) {
        spec.a_star.goal_pos_tol       = a["goal_pos_tol"].as<double>(spec.a_star.goal_pos_tol);
        spec.a_star.goal_heading_tol   = a["goal_heading_tol"].as<double>(spec.a_star.goal_heading_tol);
        spec.a_star.max_expansions     = a["max_expansions"].as<int32_t>(spec.a_star.max_expansions);
        spec.a_star.max_plan_time_sec  = a["max_plan_time_sec"].as<double>(spec.a_star.max_plan_time_sec);
        spec.a_star.enable_shot        = a["enable_shot"].as<bool>(spec.a_star.enable_shot);
        spec.a_star.shot_max_distance  = a["shot_max_distance"].as<double>(spec.a_star.shot_max_distance);
        spec.a_star.shot_allow_unknown = a["shot_allow_unknown"].as<bool>(spec.a_star.shot_allow_unknown);
        spec.a_star.closed_cell_size   = a["closed_cell_size"].as<double>(spec.a_star.closed_cell_size);
        spec.a_star.node_pool_reserve  = a["node_pool_reserve"].as<int32_t>(spec.a_star.node_pool_reserve);
        spec.a_star.closed_set_reserve = a["closed_set_reserve"].as<int32_t>(spec.a_star.closed_set_reserve);
        spec.a_star.costmap_weight     = a["costmap_weight"].as<double>(spec.a_star.costmap_weight);
    }
    SCENARIO_CHECK(spec.a_star.goal_pos_tol > 0.0, path, "input.a_star.goal_pos_tol", "must be positive");
    SCENARIO_CHECK(spec.a_star.goal_heading_tol > 0.0, path, "input.a_star.goal_heading_tol",
                   "must be positive");
    SCENARIO_CHECK(spec.a_star.max_expansions > 0, path, "input.a_star.max_expansions", "must be positive");
    SCENARIO_CHECK(spec.a_star.max_plan_time_sec > 0.0, path, "input.a_star.max_plan_time_sec",
                   "must be positive");
    SCENARIO_CHECK(spec.a_star.closed_cell_size > 0.0, path, "input.a_star.closed_cell_size",
                   "must be positive");
    SCENARIO_CHECK(spec.a_star.shot_max_distance >= 0.0, path, "input.a_star.shot_max_distance",
                   "must be non-negative");
    SCENARIO_CHECK(spec.a_star.node_pool_reserve >= 0, path, "input.a_star.node_pool_reserve",
                   "must be non-negative");
    SCENARIO_CHECK(spec.a_star.closed_set_reserve >= 0, path, "input.a_star.closed_set_reserve",
                   "must be non-negative");
    SCENARIO_CHECK(spec.a_star.costmap_weight >= 0.0, path, "input.a_star.costmap_weight",
                   "must be non-negative");

    // --- Heuristic config (optional; struct defaults apply, mirrors loadHeuristicConfig) ---
    if (const YAML::Node h = input["heuristic"]) {
        spec.heuristic.weight = h["weight"].as<double>(spec.heuristic.weight);
        spec.heuristic.use_obstacle_heuristic =
            h["use_obstacle_heuristic"].as<bool>(spec.heuristic.use_obstacle_heuristic);
    }
    SCENARIO_CHECK(spec.heuristic.weight > 0.0, path, "input.heuristic.weight",
                   "must be positive (1.0 = unscaled estimate; >1 = greedier search)");

    // --- Cost policy (required keys, mirrors loadCostPolicy) ---
    const YAML::Node cp = require(input, "cost_policy", path, "input");
    spec.cost_policy.clearance_radius_   = require(cp, "clearance_radius", path, "input.cost_policy").as<float>();
    spec.cost_policy.clearance_decay_    = require(cp, "clearance_decay", path, "input.cost_policy").as<float>();
    spec.cost_policy.max_clearance_cost_ = require(cp, "max_clearance_cost", path, "input.cost_policy").as<float>();
    SCENARIO_CHECK(!cp["unknown_cost"], path, "input.cost_policy.unknown_cost",
                   "no longer accepted; use unknown_cost_factor (>= 1)");
    spec.cost_policy.unknown_cost_factor_ = require(cp, "unknown_cost_factor", path, "input.cost_policy").as<float>();
    SCENARIO_CHECK(spec.cost_policy.clearance_radius_ >= 0.0f, path, "input.cost_policy.clearance_radius",
                   "must be non-negative");
    SCENARIO_CHECK(spec.cost_policy.clearance_decay_ >= 0.0f, path, "input.cost_policy.clearance_decay",
                   "must be non-negative");
    SCENARIO_CHECK(spec.cost_policy.max_clearance_cost_ >= 0.0f, path, "input.cost_policy.max_clearance_cost",
                   "must be non-negative");
    SCENARIO_CHECK(std::isfinite(spec.cost_policy.unknown_cost_factor_) && spec.cost_policy.unknown_cost_factor_ >= 1.0f,
                   path, "input.cost_policy.unknown_cost_factor", "must be finite and at least 1");

    // --- Start / goal query ---
    spec.start = parsePose(require(input, "start", path, "input"), path, "input.start");
    spec.goal  = parsePose(require(input, "goal", path, "input"), path, "input.goal");

    // --- Expectations (design-intent oracle; all fields optional) ---
    if (const YAML::Node exp = root["expectations"]) {
        spec.expect.expect_found = exp["expect_found"].as<bool>(spec.expect.expect_found);
        if (exp["cost_max"])       spec.expect.cost_max       = exp["cost_max"].as<double>();
        if (exp["expansions_max"]) spec.expect.expansions_max = exp["expansions_max"].as<int>();
        if (exp["unknown_poses_max"]) {
            spec.expect.unknown_poses_max = exp["unknown_poses_max"].as<int>();
            SCENARIO_CHECK(*spec.expect.unknown_poses_max >= 0, path, "expectations.unknown_poses_max",
                           "must be non-negative");
        }
    }

    return spec;
}

std::vector<std::string> discoverScenarioFiles(const std::string & dir) {
    std::vector<std::string> files;

    std::error_code ec;
    if (!std::filesystem::is_directory(dir, ec)) return files;

    for (const auto & entry : std::filesystem::directory_iterator(dir, ec)) {
        if (ec) break;
        if (!entry.is_regular_file()) continue;
        const std::string ext = entry.path().extension().string();
        if (ext == ".yaml" || ext == ".yml") {
            files.push_back(entry.path().string());
        }
    }

    std::sort(files.begin(), files.end());
    return files;
}
