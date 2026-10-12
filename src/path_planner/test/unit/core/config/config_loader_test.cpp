#include <gtest/gtest.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "core/config/config_loader.hpp"
#include "core/robot/robot_params.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/planner/cost/cost_policy.hpp"
#include "core/planner/hybrid_astar/config.hpp"
#include "core/planner/hybrid_astar/heuristics.hpp"

/*
Config-loader tests: each loader parses a YAML file into a typed struct and CHECK-validates it. Tests
write a temporary YAML file, then assert (a) a valid file round-trips to the expected values and honours
defaults for omitted fields, and (b) each out-of-range field throws std::invalid_argument. Robot vertex
validation lives in buildRobotParams (tested in robot_params_test); here we confirm the loader threads
through to it.
*/

namespace {

std::string writeTemp(const std::string & name, const std::string & content){
    const std::string path = std::string(testing::TempDir()) + name;
    std::ofstream out(path);
    out << content;
    out.close();
    return path;
}

}  // namespace

// ---- loadRobotParams -------------------------------------------------------------------------

TEST(LoadRobotParams, parsesValidSquareAndDerivesRadii){
    auto path = writeTemp("rp_ok.yml",
        "vertices: [[0.3,0.3],[-0.3,0.3],[-0.3,-0.3],[0.3,-0.3]]\n");
    RobotParams p = loadRobotParams(path);
    EXPECT_EQ(p.vertices.size(), 4u);
    EXPECT_EQ(p.half_planes.size(), 4u);
    EXPECT_NEAR(p.r_in, 0.3, 1e-6);
    EXPECT_NEAR(p.r_out, std::sqrt(0.18), 1e-6);
}

TEST(LoadRobotParams, throwsOnTooFewVertices){
    auto path = writeTemp("rp_few.yml", "vertices: [[0.0,0.0],[1.0,0.0]]\n");
    EXPECT_THROW(loadRobotParams(path), std::invalid_argument);
}

TEST(LoadRobotParams, throwsOnClockwiseWinding){
    // Same square wound clockwise -> validateConvexCCWPolygon rejects.
    auto path = writeTemp("rp_cw.yml",
        "vertices: [[0.3,0.3],[0.3,-0.3],[-0.3,-0.3],[-0.3,0.3]]\n");
    EXPECT_THROW(loadRobotParams(path), std::invalid_argument);
}

// ---- loadDiffDriveParams ---------------------------------------------------------------------

TEST(LoadDiffDriveParams, parsesValidValues){
    auto path = writeTemp("dd_ok.yml",
        "diff_drive:\n  rmin: 0.3\n  vmax: 3.0\n  atomic_time: 1.0\n  n_headings: 8\n");
    DiffDriveParams p = loadDiffDriveParams(path);
    EXPECT_DOUBLE_EQ(p.rmin, 0.3);
    EXPECT_DOUBLE_EQ(p.vmax, 3.0);
    EXPECT_DOUBLE_EQ(p.atomic_time, 1.0);
    EXPECT_EQ(p.n_headings, 8);
}

TEST(LoadDiffDriveParams, defaultsForOmittedOptionalFields){
    // atomic_time and n_headings are optional; rmin/vmax are required.
    auto path = writeTemp("dd_def.yml", "diff_drive:\n  rmin: 0.5\n  vmax: 1.0\n");
    DiffDriveParams p = loadDiffDriveParams(path);
    EXPECT_DOUBLE_EQ(p.atomic_time, 0.6);  // struct default
    EXPECT_EQ(p.n_headings, 16);           // struct default
}

TEST(LoadDiffDriveParams, throwsOnNonPositiveFields){
    EXPECT_THROW(loadDiffDriveParams(writeTemp("dd_r.yml",
        "diff_drive:\n  rmin: -0.1\n  vmax: 3.0\n")), std::invalid_argument);
    EXPECT_THROW(loadDiffDriveParams(writeTemp("dd_v.yml",
        "diff_drive:\n  rmin: 0.3\n  vmax: 0.0\n")), std::invalid_argument);
    EXPECT_THROW(loadDiffDriveParams(writeTemp("dd_t.yml",
        "diff_drive:\n  rmin: 0.3\n  vmax: 3.0\n  atomic_time: -1.0\n")), std::invalid_argument);
    EXPECT_THROW(loadDiffDriveParams(writeTemp("dd_n.yml",
        "diff_drive:\n  rmin: 0.3\n  vmax: 3.0\n  n_headings: 0\n")), std::invalid_argument);
}

// ---- loadCostPolicy --------------------------------------------------------------------------

TEST(LoadCostPolicy, parsesValidValues){
    auto path = writeTemp("cp_ok.yml",
        "cost_policy:\n  clearance_radius: 0.5\n  clearance_decay: 15.0\n"
        "  max_clearance_cost: 30.0\n  unknown_cost_factor: 1.5\n");
    CostPolicy p = loadCostPolicy(path);
    EXPECT_FLOAT_EQ(p.clearance_radius_, 0.5f);
    EXPECT_FLOAT_EQ(p.clearance_decay_, 15.0f);
    EXPECT_FLOAT_EQ(p.max_clearance_cost_, 30.0f);
    EXPECT_FLOAT_EQ(p.unknown_cost_factor_, 1.5f);
}

TEST(LoadCostPolicy, acceptsFactorOne){
    EXPECT_FLOAT_EQ(loadCostPolicy(writeTemp("cp_one.yml",
        "cost_policy:\n  clearance_radius: 0.5\n  clearance_decay: 15.0\n"
        "  max_clearance_cost: 30.0\n  unknown_cost_factor: 1.0\n")).unknown_cost_factor_, 1.0f);
}

TEST(LoadCostPolicy, throwsOnAnyInvalidField){
    EXPECT_THROW(loadCostPolicy(writeTemp("cp_rad.yml",
        "cost_policy:\n  clearance_radius: -0.5\n  clearance_decay: 15.0\n"
        "  max_clearance_cost: 30.0\n  unknown_cost_factor: 1.0\n")), std::invalid_argument);
    EXPECT_THROW(loadCostPolicy(writeTemp("cp_dec.yml",
        "cost_policy:\n  clearance_radius: 0.5\n  clearance_decay: -1.0\n"
        "  max_clearance_cost: 30.0\n  unknown_cost_factor: 1.0\n")), std::invalid_argument);
    EXPECT_THROW(loadCostPolicy(writeTemp("cp_max.yml",
        "cost_policy:\n  clearance_radius: 0.5\n  clearance_decay: 15.0\n"
        "  max_clearance_cost: -30.0\n  unknown_cost_factor: 1.0\n")), std::invalid_argument);
    EXPECT_THROW(loadCostPolicy(writeTemp("cp_unk.yml",
        "cost_policy:\n  clearance_radius: 0.5\n  clearance_decay: 15.0\n"
        "  max_clearance_cost: 30.0\n  unknown_cost_factor: 0.99\n")), std::invalid_argument);
    EXPECT_THROW(loadCostPolicy(writeTemp("cp_unk0.yml",
        "cost_policy:\n  clearance_radius: 0.5\n  clearance_decay: 15.0\n"
        "  max_clearance_cost: 30.0\n  unknown_cost_factor: 0.0\n")), std::invalid_argument);
}

TEST(LoadCostPolicy, rejectsFormerUnknownCostKey){
    EXPECT_THROW(loadCostPolicy(writeTemp("cp_old.yml",
        "cost_policy:\n  clearance_radius: 0.5\n  clearance_decay: 15.0\n"
        "  max_clearance_cost: 30.0\n  unknown_cost: 8.0\n  unknown_cost_factor: 1.0\n")), std::invalid_argument);
}

// ---- loadAStarConfig -------------------------------------------------------------------------

TEST(LoadAStarConfig, parsesValidValues){
    auto path = writeTemp("as_ok.yml",
        "a_star:\n  goal_pos_tol: 0.2\n  goal_heading_tol: 0.15\n  max_expansions: 1000\n"
        "  max_plan_time_sec: 2.5\n  enable_shot: false\n  shot_max_distance: 4.0\n"
        "  shot_allow_unknown: true\n  closed_cell_size: 0.3\n"
        "  node_pool_reserve: 100\n  closed_set_reserve: 200\n  costmap_weight: 1.5\n");
    AStarConfig c = loadAStarConfig(path);
    EXPECT_DOUBLE_EQ(c.goal_pos_tol, 0.2);
    EXPECT_DOUBLE_EQ(c.goal_heading_tol, 0.15);
    EXPECT_EQ(c.max_expansions, 1000);
    EXPECT_DOUBLE_EQ(c.max_plan_time_sec, 2.5);
    EXPECT_FALSE(c.enable_shot);
    EXPECT_DOUBLE_EQ(c.shot_max_distance, 4.0);
    EXPECT_TRUE(c.shot_allow_unknown);
    EXPECT_DOUBLE_EQ(c.closed_cell_size, 0.3);
    EXPECT_EQ(c.node_pool_reserve, 100);
    EXPECT_EQ(c.closed_set_reserve, 200);
    EXPECT_DOUBLE_EQ(c.costmap_weight, 1.5);
}

TEST(LoadAStarConfig, omittedFieldsFallBackToDefaults){
    auto path = writeTemp("as_def.yml", "a_star:\n  goal_pos_tol: 0.25\n");
    AStarConfig c = loadAStarConfig(path);
    EXPECT_DOUBLE_EQ(c.goal_pos_tol, 0.25);
    EXPECT_EQ(c.max_expansions, 200000);        // default
    EXPECT_DOUBLE_EQ(c.max_plan_time_sec, 5.0); // default
    EXPECT_TRUE(c.enable_shot);                 // default
    EXPECT_FALSE(c.shot_allow_unknown);         // default (conservative KNOWN-FREE shot)
    EXPECT_DOUBLE_EQ(c.closed_cell_size, 0.15); // default (closed-set dedup cell)
}

TEST(LoadAStarConfig, throwsOnInvalidFields){
    EXPECT_THROW(loadAStarConfig(writeTemp("as_pt.yml", "a_star:\n  goal_pos_tol: 0.0\n")),
                 std::invalid_argument);
    EXPECT_THROW(loadAStarConfig(writeTemp("as_ht.yml", "a_star:\n  goal_heading_tol: 0.0\n")),
                 std::invalid_argument);
    EXPECT_THROW(loadAStarConfig(writeTemp("as_me.yml", "a_star:\n  max_expansions: 0\n")),
                 std::invalid_argument);
    EXPECT_THROW(loadAStarConfig(writeTemp("as_mt.yml", "a_star:\n  max_plan_time_sec: 0.0\n")),
                 std::invalid_argument);
    EXPECT_THROW(loadAStarConfig(writeTemp("as_ccz.yml", "a_star:\n  closed_cell_size: 0.0\n")),
                 std::invalid_argument);
    EXPECT_THROW(loadAStarConfig(writeTemp("as_ccn.yml", "a_star:\n  closed_cell_size: -0.1\n")),
                 std::invalid_argument);
    EXPECT_THROW(loadAStarConfig(writeTemp("as_sd.yml", "a_star:\n  shot_max_distance: -1.0\n")),
                 std::invalid_argument);
    EXPECT_THROW(loadAStarConfig(writeTemp("as_np.yml", "a_star:\n  node_pool_reserve: -1\n")),
                 std::invalid_argument);
    EXPECT_THROW(loadAStarConfig(writeTemp("as_cw.yml", "a_star:\n  costmap_weight: -0.5\n")),
                 std::invalid_argument);
}

// ---- loadHeuristicConfig ---------------------------------------------------------------------

TEST(LoadHeuristicConfig, parsesValidWeight){
    auto path = writeTemp("h_ok.yml", "heuristic:\n  weight: 2.0\n");
    EXPECT_DOUBLE_EQ(loadHeuristicConfig(path).weight, 2.0);
}

TEST(LoadHeuristicConfig, defaultsWhenBlockMissing){
    auto path = writeTemp("h_missing.yml", "something_else: 1\n");
    EXPECT_DOUBLE_EQ(loadHeuristicConfig(path).weight, 1.0);  // struct default
}

TEST(LoadHeuristicConfig, parsesUseObstacleHeuristic){
    auto path = writeTemp("h_obs.yml", "heuristic:\n  weight: 1.0\n  use_obstacle_heuristic: false\n");
    EXPECT_FALSE(loadHeuristicConfig(path).use_obstacle_heuristic);
}

TEST(LoadHeuristicConfig, defaultsUseObstacleHeuristicWhenOmitted){
    auto path = writeTemp("h_obs_def.yml", "heuristic:\n  weight: 1.0\n");
    EXPECT_TRUE(loadHeuristicConfig(path).use_obstacle_heuristic);  // struct default
}

TEST(LoadHeuristicConfig, throwsOnNonPositiveWeight){
    EXPECT_THROW(loadHeuristicConfig(writeTemp("h_zero.yml", "heuristic:\n  weight: 0.0\n")),
                 std::invalid_argument);
    EXPECT_THROW(loadHeuristicConfig(writeTemp("h_neg.yml", "heuristic:\n  weight: -1.0\n")),
                 std::invalid_argument);
}

// ---- loadControllerParams (pure-pursuit follower gains) ---------------------------------------

namespace {
// The six controller gains live under a `controller:` root; all are required and strictly positive.
std::string controllerYaml(double lookahead_gain, double v_cruise, double a_lat_max,
                           double omega_pivot, double goal_pos_tol, double goal_heading_tol){
    return "controller:\n"
           "  lookahead_gain: "   + std::to_string(lookahead_gain)   + "\n"
           "  v_cruise: "         + std::to_string(v_cruise)         + "\n"
           "  a_lat_max: "        + std::to_string(a_lat_max)        + "\n"
           "  omega_pivot: "      + std::to_string(omega_pivot)      + "\n"
           "  goal_pos_tol: "     + std::to_string(goal_pos_tol)     + "\n"
           "  goal_heading_tol: " + std::to_string(goal_heading_tol) + "\n";
}
}  // namespace

TEST(LoadControllerParams, parsesValidValues){
    // The config/controller/controller.yaml seeds (k = 3.0 gain).
    auto path = writeTemp("ctrl_ok.yml",
        controllerYaml(3.0, 2.0, 2.0, 1.0, 0.10, 0.10));
    ControllerParams p = loadControllerParams(path);
    EXPECT_DOUBLE_EQ(p.lookahead_gain, 3.0);
    EXPECT_DOUBLE_EQ(p.v_cruise, 2.0);
    EXPECT_DOUBLE_EQ(p.a_lat_max, 2.0);
    EXPECT_DOUBLE_EQ(p.omega_pivot, 1.0);
    EXPECT_DOUBLE_EQ(p.goal_pos_tol, 0.10);
    EXPECT_DOUBLE_EQ(p.goal_heading_tol, 0.10);
}

TEST(LoadControllerParams, throwsOnMissingKey){
    // Every key is required (no defaults). Omit each in turn -> YAML conversion of a null node throws.
    EXPECT_THROW(loadControllerParams(writeTemp("ctrl_m_la.yml",
        "controller:\n  v_cruise: 2.0\n  a_lat_max: 2.0\n  omega_pivot: 1.0\n"
        "  goal_pos_tol: 0.1\n  goal_heading_tol: 0.1\n")), std::exception);
    EXPECT_THROW(loadControllerParams(writeTemp("ctrl_m_vc.yml",
        "controller:\n  lookahead_gain: 3.0\n  a_lat_max: 2.0\n  omega_pivot: 1.0\n"
        "  goal_pos_tol: 0.1\n  goal_heading_tol: 0.1\n")), std::exception);
    EXPECT_THROW(loadControllerParams(writeTemp("ctrl_m_alm.yml",
        "controller:\n  lookahead_gain: 3.0\n  v_cruise: 2.0\n  omega_pivot: 1.0\n"
        "  goal_pos_tol: 0.1\n  goal_heading_tol: 0.1\n")), std::exception);
    EXPECT_THROW(loadControllerParams(writeTemp("ctrl_m_op.yml",
        "controller:\n  lookahead_gain: 3.0\n  v_cruise: 2.0\n  a_lat_max: 2.0\n"
        "  goal_pos_tol: 0.1\n  goal_heading_tol: 0.1\n")), std::exception);
    EXPECT_THROW(loadControllerParams(writeTemp("ctrl_m_gpt.yml",
        "controller:\n  lookahead_gain: 3.0\n  v_cruise: 2.0\n  a_lat_max: 2.0\n"
        "  omega_pivot: 1.0\n  goal_heading_tol: 0.1\n")), std::exception);
    EXPECT_THROW(loadControllerParams(writeTemp("ctrl_m_ght.yml",
        "controller:\n  lookahead_gain: 3.0\n  v_cruise: 2.0\n  a_lat_max: 2.0\n"
        "  omega_pivot: 1.0\n  goal_pos_tol: 0.1\n")), std::exception);
}

TEST(LoadControllerParams, throwsOnNonPositiveGain){
    // Each gain has a CHECK(> 0). CHECK throws std::invalid_argument (see core/diagnostics/check.hpp),
    // matching the other loaders' out-of-range convention. Drive each field to 0 in turn.
    EXPECT_THROW(loadControllerParams(writeTemp("ctrl_np_la.yml",
        controllerYaml(0.0, 2.0, 2.0, 1.0, 0.10, 0.10))), std::invalid_argument);
    EXPECT_THROW(loadControllerParams(writeTemp("ctrl_np_vc.yml",
        controllerYaml(0.75, 0.0, 2.0, 1.0, 0.10, 0.10))), std::invalid_argument);
    EXPECT_THROW(loadControllerParams(writeTemp("ctrl_np_alm.yml",
        controllerYaml(0.75, 2.0, 0.0, 1.0, 0.10, 0.10))), std::invalid_argument);
    EXPECT_THROW(loadControllerParams(writeTemp("ctrl_np_op.yml",
        controllerYaml(0.75, 2.0, 2.0, 0.0, 0.10, 0.10))), std::invalid_argument);
    EXPECT_THROW(loadControllerParams(writeTemp("ctrl_np_gpt.yml",
        controllerYaml(0.75, 2.0, 2.0, 1.0, 0.0, 0.10))), std::invalid_argument);
    EXPECT_THROW(loadControllerParams(writeTemp("ctrl_np_ght.yml",
        controllerYaml(0.75, 2.0, 2.0, 1.0, 0.10, 0.0))), std::invalid_argument);
    // Negative also rejected.
    EXPECT_THROW(loadControllerParams(writeTemp("ctrl_neg_la.yml",
        controllerYaml(-0.5, 2.0, 2.0, 1.0, 0.10, 0.10))), std::invalid_argument);
}

// ---- loadScanGuardParams (reactive /scan safety guard tuning) --------------------------------

namespace {
// The eight scan-guard fields live under a `scan_guard:` root; all are required. The four doubles
// before the counts are passed as strings so tests can write .nan / .inf.
struct GuardVals {
    std::string linear_decel = "1.5", angular_decel = "2.5", forward_pad = "0.02", side_pad = "0.01";
    std::string stop_count = "2", clear_count = "10", stale_scan_timeout = "0.5", tf_lookup_timeout = "0.1";
};
std::string scanGuardYaml(const GuardVals & g, const std::string & drop = ""){
    const std::vector<std::pair<std::string, std::string>> kv = {
        {"linear_decel", g.linear_decel}, {"angular_decel", g.angular_decel},
        {"forward_pad", g.forward_pad}, {"side_pad", g.side_pad},
        {"stop_count", g.stop_count}, {"clear_count", g.clear_count},
        {"stale_scan_timeout", g.stale_scan_timeout}, {"tf_lookup_timeout", g.tf_lookup_timeout}};
    std::string y = "scan_guard:\n";
    for (const auto & [k, v] : kv)
        if (k != drop) y += "  " + k + ": " + v + "\n";
    return y;
}
std::string shippedConfigPath(const char * dir, const char * file){
    // <path_planner>/test/unit/core/config/config_loader_test.cpp -> up 5 levels to <path_planner>.
    std::filesystem::path root = std::filesystem::path(__FILE__);
    for (int i = 0; i < 5; ++i) root = root.parent_path();
    return (root / "config" / dir / file).string();
}
}  // namespace

TEST(LoadScanGuardParams, parsesValidValues){
    auto path = writeTemp("sg_ok.yml", scanGuardYaml(GuardVals{}));
    ScanGuardParams p = loadScanGuardParams(path);
    EXPECT_DOUBLE_EQ(p.linear_decel, 1.5);
    EXPECT_DOUBLE_EQ(p.angular_decel, 2.5);
    EXPECT_DOUBLE_EQ(p.forward_pad, 0.02);
    EXPECT_DOUBLE_EQ(p.side_pad, 0.01);
    EXPECT_EQ(p.stop_count, 2);
    EXPECT_EQ(p.clear_count, 10);
    EXPECT_DOUBLE_EQ(p.stale_scan_timeout, 0.5);
    EXPECT_DOUBLE_EQ(p.tf_lookup_timeout, 0.1);
}

TEST(LoadScanGuardParams, acceptsZeroForwardPad){
    GuardVals g; g.forward_pad = "0"; g.stop_count = "1"; g.clear_count = "1";
    ScanGuardParams p = loadScanGuardParams(writeTemp("sg_zero.yml", scanGuardYaml(g)));
    EXPECT_DOUBLE_EQ(p.forward_pad, 0.0);
    EXPECT_EQ(p.stop_count, 1);
    EXPECT_EQ(p.clear_count, 1);
}

TEST(LoadScanGuardParams, throwsOnMissingKey){
    // Every key is required (no defaults). Omit each in turn -> YAML conversion of a null node throws.
    int n = 0;
    for (const char * key : {"linear_decel", "angular_decel", "forward_pad", "side_pad", "stop_count",
                             "clear_count", "stale_scan_timeout", "tf_lookup_timeout"})
        EXPECT_THROW(loadScanGuardParams(writeTemp("sg_m" + std::to_string(n++) + ".yml",
                                                   scanGuardYaml(GuardVals{}, key))), std::exception) << key;
}

TEST(LoadScanGuardParams, throwsOnOutOfRangeFields){
    auto bad = [](auto set, const std::string & v){
        GuardVals g; set(g, v);
        return loadScanGuardParams(writeTemp("sg_bad.yml", scanGuardYaml(g)));
    };
    auto ld = [](GuardVals & g, const std::string & v){ g.linear_decel = v; };
    auto ad = [](GuardVals & g, const std::string & v){ g.angular_decel = v; };
    auto fp = [](GuardVals & g, const std::string & v){ g.forward_pad = v; };
    auto sp = [](GuardVals & g, const std::string & v){ g.side_pad = v; };
    EXPECT_THROW(bad(ld, "0"), std::invalid_argument);
    EXPECT_THROW(bad(ld, "-1.5"), std::invalid_argument);
    EXPECT_THROW(bad(ad, "0"), std::invalid_argument);
    EXPECT_THROW(bad(fp, "-0.01"), std::invalid_argument);
    EXPECT_THROW(bad(sp, "0"), std::invalid_argument);
    for (const char * nf : {".nan", ".inf"}) {
        EXPECT_THROW(bad(ld, nf), std::invalid_argument) << nf;
        EXPECT_THROW(bad(ad, nf), std::invalid_argument) << nf;
        EXPECT_THROW(bad(fp, nf), std::invalid_argument) << nf;
        EXPECT_THROW(bad(sp, nf), std::invalid_argument) << nf;
    }
    EXPECT_THROW(bad([](GuardVals & g, const std::string & v){ g.stop_count = v; }, "0"), std::invalid_argument);
    EXPECT_THROW(bad([](GuardVals & g, const std::string & v){ g.clear_count = v; }, "0"), std::invalid_argument);
    EXPECT_THROW(bad([](GuardVals & g, const std::string & v){ g.stale_scan_timeout = v; }, "0"), std::invalid_argument);
    EXPECT_THROW(bad([](GuardVals & g, const std::string & v){ g.tf_lookup_timeout = v; }, "0"), std::invalid_argument);
    EXPECT_THROW(bad([](GuardVals & g, const std::string & v){ g.stale_scan_timeout = v; }, "-0.5"), std::invalid_argument);
    EXPECT_THROW(bad([](GuardVals & g, const std::string & v){ g.tf_lookup_timeout = v; }, "-0.1"), std::invalid_argument);
}

TEST(LoadScanGuardParams, shippedFileLoads){
    ScanGuardParams p = loadScanGuardParams(shippedConfigPath("safety", "scan_safety_guard.yaml"));
    EXPECT_DOUBLE_EQ(p.linear_decel, 1.5);
    EXPECT_DOUBLE_EQ(p.angular_decel, 2.5);
    EXPECT_DOUBLE_EQ(p.forward_pad, 0.02);
    EXPECT_DOUBLE_EQ(p.side_pad, 0.01);
    EXPECT_EQ(p.stop_count, 2);
    EXPECT_EQ(p.clear_count, 10);
    EXPECT_DOUBLE_EQ(p.stale_scan_timeout, 0.5);
    EXPECT_DOUBLE_EQ(p.tf_lookup_timeout, 0.1);
}

// ---- loadNavigatorParams (navigator sequencing and clearing-move tuning) ---------------------

namespace {
// Every field is required. Values default to the shipped ones; `key`/`value` replace one, and
// `drop` omits a key entirely.
std::string navigatorYaml(const std::string & key = "", const std::string & value = "",
                          const std::string & drop = ""){
    const std::vector<std::pair<std::string, std::string>> top = {
        {"tick_period_sec", "0.5"}, {"stall_timeout", "3.0"}, {"blocked_retry_period", "2.0"},
        {"max_clearing_attempts", "2"}, {"max_stall_replans", "2"}, {"escape_speed_scale", "0.5"}};
    const std::vector<std::pair<std::string, std::string>> esc = {
        {"step_fraction", "0.4"}, {"slack_cells", "1"}, {"max_travel_factor", "5.0"}};
    std::string y = "navigator:\n";
    for (const auto & [k, v] : top)
        if (k != drop) y += "  " + k + ": " + (k == key ? value : v) + "\n";
    y += "  escape:\n";
    for (const auto & [k, v] : esc)
        if (k != drop) y += "    " + k + ": " + (k == key ? value : v) + "\n";
    return y;
}
}  // namespace

TEST(LoadNavigatorParams, parsesValidValues){
    NavigatorParams p = loadNavigatorParams(writeTemp("nav_ok.yml", navigatorYaml()));
    EXPECT_DOUBLE_EQ(p.tick_period_sec, 0.5);
    EXPECT_DOUBLE_EQ(p.stall_timeout, 3.0);
    EXPECT_DOUBLE_EQ(p.blocked_retry_period, 2.0);
    EXPECT_EQ(p.max_clearing_attempts, 2);
    EXPECT_EQ(p.max_stall_replans, 2);
    EXPECT_DOUBLE_EQ(p.escape_speed_scale, 0.5);
    EXPECT_DOUBLE_EQ(p.escape.step_fraction, 0.4);
    EXPECT_EQ(p.escape.slack_cells, 1);
    EXPECT_DOUBLE_EQ(p.escape.max_travel_factor, 5.0);
}

TEST(LoadNavigatorParams, throwsOnMissingKey){
    int n = 0;
    for (const char * key : {"tick_period_sec", "stall_timeout", "blocked_retry_period", "max_clearing_attempts",
                             "max_stall_replans", "escape_speed_scale", "step_fraction", "slack_cells", "max_travel_factor"})
        EXPECT_THROW(loadNavigatorParams(writeTemp("nav_m" + std::to_string(n++) + ".yml",
                                                   navigatorYaml("", "", key))), std::exception) << key;
}

TEST(LoadNavigatorParams, throwsOnOutOfRangeValue){
    const std::vector<std::pair<std::string, std::string>> bad = {
        {"tick_period_sec", "0"},          {"stall_timeout", "0.5"},   // stall_timeout == tick_period_sec
        {"stall_timeout", "0.4"},          {"blocked_retry_period", "0"},
        {"max_clearing_attempts", "0"},    {"max_stall_replans", "-1"},
        {"escape_speed_scale", "0"},
        {"escape_speed_scale", "1.01"},    {"step_fraction", "1.0"},
        {"step_fraction", "0"},            {"slack_cells", "-1"},
        {"max_travel_factor", "0"}};
    int n = 0;
    for (const auto & [k, v] : bad)
        EXPECT_THROW(loadNavigatorParams(writeTemp("nav_b" + std::to_string(n++) + ".yml", navigatorYaml(k, v))),
                     std::invalid_argument) << k << ": " << v;
}

TEST(LoadNavigatorParams, acceptsZeroMaxStallReplans){
    NavigatorParams p = loadNavigatorParams(writeTemp("nav_zero_stall.yml", navigatorYaml("max_stall_replans", "0")));
    EXPECT_EQ(p.max_stall_replans, 0);
}

TEST(LoadNavigatorParams, shippedFileLoads){
    NavigatorParams p = loadNavigatorParams(shippedConfigPath("navigation", "navigator.yaml"));
    EXPECT_EQ(p.max_stall_replans, 4);
}
