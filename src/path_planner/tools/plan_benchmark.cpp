/*
plan_benchmark: times the planner over a scenario sweep.

  plan_benchmark --scenario <file.yaml> [--robot <robot_params.yaml>] [--n-headings 16,32,64]
                 [--vmax 0.5] [--atomic-time 0.6] [--repeats 5] [--shot on|off|both]
                 [--max-plan-time <sec>] [--max-expansions <n>]
                 [--cost-policy <cost_policy.yaml>] [--planner <planner.yaml>]
                 [--start x,y,theta] [--goal x,y,theta]
  plan_benchmark --shot-cost --scenario <file.yaml> [--robot <robot_params.yaml>]
  plan_benchmark --cover-split --scenario <file.yaml> [--robot <robot_params.yaml>] [--vmax --atomic-time --n-headings]

Sweep mode prints one key=value line per scenario x n_headings x shot setting (times in ms, lengths
in m, counts in cells/nodes). The scenario supplies the map, start, goal, search config, heuristic and
cost policy; --start / --goal (m, m, rad, map frame) replace its start / goal pose. Each result line
also reports the poses used and, for the last repeat's path, path_hash (FNV-1a over the node points and
total cost), poses_total, poses_off_map and poses_unknown (all 0 when no path is found). The line also
reports duplicate_skipped and, for the last repeat, wave_cells / grid_cells: the cells the obstacle
heuristic's field must cover to price every node the search generated (0 when the field is inactive or the
obstacle heuristic is off). A WARN is printed when the per-primitive counters do not sum to
primitives * (expansions - found). Its robot body and kinematics are used with vmax, atomic_time and n_headings overridden
(with --robot, also the body polygon and rmin). Shot-cost mode times the exact-cover plus cell check
of straight segments from the scenario start pose, in microseconds. Cover-split mode splits the
exact-cover time of every library primitive into rasterization and the rest.
*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <string>
#include <vector>

#include "core/collision_checker/collision_checker.hpp"
#include "core/config/config_loader.hpp"
#include "core/map/edt.hpp"
#include "core/map/grid.hpp"
#include "core/map/map_provider.hpp"
#include "core/map/rasterize.hpp"
#include "core/motion/world_robot_geometry.hpp"
#include "core/planner/cost/costmap.hpp"
#include "core/planner/hybrid_astar/obstacle_heuristic.hpp"
#include "core/planner/search_observer.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/motion/kinematic_model.hpp"
#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/motion_context.hpp"
#include "core/planner/hybrid_astar/build_hybrid_astar.hpp"
#include "core/planner/hybrid_astar/search.hpp"
#include "core/planner/validation/path_validity.hpp"
#include "core/scenario/scenario.hpp"
#include "core/scenario/scenario_loader.hpp"

namespace {

using Clock = std::chrono::steady_clock;

double msSince(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

double median(std::vector<double> v) {
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

struct Args {
    std::string scenario, robot, cost_policy, planner, shot = "both";
    std::vector<int> n_headings{16, 32, 64};
    double vmax = 0.5, atomic_time = 0.6;
    int repeats = 5;
    bool shot_cost = false, cover_split = false;
    double max_plan_time = -1;
    long long max_expansions = -1;
    std::optional<SE2State> start, goal;
};

/// Parses "x,y,theta" (exactly three finite numbers).
SE2State parsePose(const std::string & flag, const std::string & text) {
    std::stringstream ss(text);
    double v[3];
    int n = 0;
    for (std::string t; std::getline(ss, t, ','); ++n) {
        size_t used = 0;
        if (n >= 3) { n = 4; break; }
        try { v[n] = std::stod(t, &used); } catch (const std::exception &) { used = 0; }
        if (used != t.size() || !std::isfinite(v[n]))
            throw std::invalid_argument(flag + " expects x,y,theta (three finite numbers), got '" + text + "'");
    }
    if (n != 3) throw std::invalid_argument(flag + " expects x,y,theta (three finite numbers), got '" + text + "'");
    SE2State p;
    p.x = v[0]; p.y = v[1]; p.theta = v[2];
    return p;
}

/// Applies the scenario/robot overrides and returns the spec to build.
ScenarioSpec makeSpec(const Args & a, int n_headings, bool shot) {
    ScenarioSpec s = loadScenario(a.scenario);
    if (!a.robot.empty()) {
        s.robot_vertices = loadRobotParams(a.robot).vertices;
        s.diff_drive.rmin = loadDiffDriveParams(a.robot).rmin;
    }
    if (!a.cost_policy.empty()) s.cost_policy = loadCostPolicy(a.cost_policy);
    if (!a.planner.empty()) {
        s.a_star = loadAStarConfig(a.planner);
        s.heuristic = loadHeuristicConfig(a.planner);
    }
    if (a.start) s.start = *a.start;
    if (a.goal) s.goal = *a.goal;
    s.diff_drive.vmax = a.vmax;
    s.diff_drive.atomic_time = a.atomic_time;
    s.diff_drive.n_headings = n_headings;
    s.a_star.enable_shot = shot;
    if (a.max_plan_time > 0) s.a_star.max_plan_time_sec = a.max_plan_time;
    if (a.max_expansions > 0) s.a_star.max_expansions = static_cast<int32_t>(a.max_expansions);
    return s;
}

/// The built objects a measurement needs; `planner` aliases `snapshot`.
struct Built {
    std::shared_ptr<const MapSnapshot> snapshot;
    RobotParams robot;
    KinematicModel kin;
    std::unique_ptr<Planner> planner;
};

/// Mirrors buildScenario but keeps the kinematic model so the primitive library can be inspected.
Built build(const ScenarioSpec & spec) {
    RobotParams robot = buildRobotParams(spec.robot_vertices);
    KinematicModel kin = buildKinematicModel(robot, spec.diff_drive, spec.meta.resolution);
    OccupancyGrid occupancy(spec.meta, spec.occupancy);
    EDT edt = buildEDTGrid(occupancy, spec.edt_clamp);
    auto snapshot = std::make_shared<const MapSnapshot>(MapSnapshot{std::move(occupancy), std::move(edt)});
    auto planner = buildHybridAstar(kin, spec.a_star, spec.heuristic, spec.cost_policy, snapshot);
    return Built{std::move(snapshot), std::move(robot), std::move(kin), std::move(planner)};
}

/// Collects every expansion event.
struct Recorder : SearchObserver {
    std::vector<SearchExpansion> ev;
    void onExpand(const SearchExpansion & e) override { ev.push_back(e); }
};

void runSweep(const Args & a) {
    for (int nh : a.n_headings) {
        for (const bool shot : {true, false}) {
            if ((a.shot == "on" && !shot) || (a.shot == "off" && shot)) continue;
            const ScenarioSpec spec = makeSpec(a, nh, shot);
            std::printf("# config clearance_radius=%g clearance_decay=%g max_clearance_cost=%g unknown_cost_factor=%g "
                        "costmap_weight=%g closed_cell_size=%g heuristic_weight=%g use_obstacle_heuristic=%d "
                        "enable_shot=%d shot_max_distance=%g shot_allow_unknown=%d goal_pos_tol=%g goal_heading_tol=%g "
                        "max_expansions=%d max_plan_time_sec=%g cost_policy=%s planner=%s\n",
                        spec.cost_policy.clearance_radius_, spec.cost_policy.clearance_decay_,
                        spec.cost_policy.max_clearance_cost_, spec.cost_policy.unknown_cost_factor_,
                        spec.a_star.costmap_weight, spec.a_star.closed_cell_size, spec.heuristic.weight,
                        spec.heuristic.use_obstacle_heuristic, spec.a_star.enable_shot, spec.a_star.shot_max_distance,
                        spec.a_star.shot_allow_unknown, spec.a_star.goal_pos_tol, spec.a_star.goal_heading_tol,
                        spec.a_star.max_expansions, spec.a_star.max_plan_time_sec,
                        a.cost_policy.empty() ? "scenario" : a.cost_policy.c_str(),
                        a.planner.empty() ? "scenario" : a.planner.c_str());
            const auto t0 = Clock::now();
            Built b = build(spec);
            const double build_ms = msSince(t0);

            size_t cells = 0;
            for (const auto & p : b.kin.prim_lib->primitives())
                for (const auto & c : p.swept_cells) cells += c.size();

            auto * hybrid = dynamic_cast<HybridAstar *>(b.planner.get());
            std::vector<double> times;
            bool differs = false;
            double first_cost = 0;
            size_t first_nodes = 0;
            std::optional<Path> path;
            for (int r = 0; r < a.repeats; ++r) {
                const auto t = Clock::now();
                path = b.planner->plan(spec.start, spec.goal).path;
                times.push_back(msSince(t));
                const double cost = path ? path->total_cost : std::numeric_limits<double>::quiet_NaN();
                const size_t nodes = path ? path->node_points.size() : 0;
                if (r == 0) { first_cost = cost; first_nodes = nodes; }
                else if (nodes != first_nodes || !(cost == first_cost || (std::isnan(cost) && std::isnan(first_cost))))
                    differs = true;
            }
            const PlanStats st = hybrid->lastPlanStats();
            if (st.exact_accept + st.exact_reject > st.scan_blocked)
                std::cout << "# WARN " << spec.name << " n_headings=" << nh << " shot=" << shot
                          << " exact_accept+exact_reject > scan_blocked\n";
            const long long n_prims = static_cast<long long>(b.kin.prim_lib->primitives().size());
            if (st.fast_accept + st.fast_reject + st.scan_clear + st.scan_blocked + st.duplicate_skipped !=
                n_prims * (st.expansions - (path ? 1 : 0)))
                std::cout << "# WARN " << spec.name << " n_headings=" << nh << " shot=" << shot
                          << " fast_accept+fast_reject+scan_clear+scan_blocked+duplicate_skipped != "
                             "primitives*(expansions-found)\n";
            if (differs)
                std::cout << "# WARN " << spec.name << " n_headings=" << nh << " shot=" << shot
                          << " path differs between repeats\n";

            const auto grid = std::shared_ptr<const OccupancyGrid>(b.snapshot, &b.snapshot->grid_);
            const auto edt = std::shared_ptr<const EDT>(b.snapshot, &b.snapshot->edt_);
            const CollisionChecker checker(grid, edt, b.robot);

            // prepare_ms: standalone obstacle-heuristic field build, as buildHybridAstar constructs it.
            const auto costmap = std::make_shared<const CostMap>(buildCostMap(*b.snapshot, spec.cost_policy, b.robot.r_in, spec.a_star.costmap_weight));
            std::printf("# unknown_cost_factor=%g unknown_cost=%g\n", spec.cost_policy.unknown_cost_factor_, costmap->unknownCost());
            const ObstacleHeuristic oh(grid, costmap, spec.a_star.costmap_weight, spec.heuristic.weight);
            std::vector<double> prep;
            for (int r = 0; r < 5; ++r) {
                const auto t = Clock::now();
                oh.prepare(spec.goal);
                prep.push_back(msSince(t));
            }

            // wave_cells: cells of the obstacle-heuristic field the search's nodes needed (`oh` is prepared
            // with the goal).
            const GridMetaData & wm = b.snapshot->grid_.getMetadata();
            const long long grid_cells = static_cast<long long>(wm.rows) * wm.cols;
            long long wave_cells = 0;
            const GridCoords goal_cell = wm.worldToGrid(spec.goal.x, spec.goal.y);
            if (spec.heuristic.use_obstacle_heuristic && wm.inBounds(goal_cell) &&
                oh.fieldAt(goal_cell) != ObstacleHeuristic::UNREACHABLE) {
                double vmax_q = -1;
                bool any = false, unreachable = false;
                for (const auto & node : hybrid->nodePool()) {
                    const GridCoords c = wm.worldToGrid(node.state.x, node.state.y);
                    if (!wm.inBounds(c)) continue;
                    any = true;
                    const double f = oh.fieldAt(c);
                    if (f == ObstacleHeuristic::UNREACHABLE) unreachable = true;
                    else vmax_q = std::max(vmax_q, f);
                }
                if (any)
                    for (int j = 0; j < wm.rows; ++j)
                        for (int i = 0; i < wm.cols; ++i) {
                            const double f = oh.fieldAt(GridCoords{i, j});
                            if (unreachable ? f < ObstacleHeuristic::UNREACHABLE : f <= vmax_q) ++wave_cells;
                        }
            }

            // Heuristic tightness: value at start vs. straight line, and with every cell the checker
            // certainly rejects for the robot centre marked OCCUPIED (cost map from the original grid).
            const double h_start = oh(spec.start);
            const double h_euclid = std::hypot(spec.goal.x - spec.start.x, spec.goal.y - spec.start.y);
            double h_start_inflated = std::numeric_limits<double>::quiet_NaN();
            long long inflated_cells = 0;
            {
                const GridMetaData & m0 = b.snapshot->grid_.getMetadata();
                std::vector<int8_t> data = b.snapshot->grid_.getData();
                const int8_t occ = static_cast<int8_t>(std::max<int>(m0.occupied_threshold, 100));
                for (int j = 0; j < m0.rows; ++j)
                    for (int i = 0; i < m0.cols; ++i) {
                        const GridCoords gc{i, j};
                        if (b.snapshot->grid_.findCellStatus(gc) != CellState::OCCUPIED &&
                            checker.fastAcceptReject(gc) == Validity::COLLISION) {
                            data[m0.index(gc)] = occ;
                            ++inflated_cells;
                        }
                    }
                const auto igrid = std::make_shared<const OccupancyGrid>(m0, data);
                const auto sc = m0.worldToGrid(spec.start.x, spec.start.y);
                const auto gcg = m0.worldToGrid(spec.goal.x, spec.goal.y);
                if (igrid->findCellStatus(sc) == CellState::OCCUPIED || igrid->findCellStatus(gcg) == CellState::OCCUPIED)
                    std::cout << "# NOTE start or goal cell blocked in the inflated grid; h_start_inflated=nan\n";
                else {
                    const ObstacleHeuristic oh2(igrid, costmap, spec.a_star.costmap_weight, spec.heuristic.weight);
                    oh2.prepare(spec.goal);
                    h_start_inflated = oh2(spec.start);
                }
            }

            // One untimed observed plan: node count and expanded-via-blocked-scan count.
            auto rec = std::make_shared<Recorder>();
            b.planner->plan(spec.start, spec.goal, rec);
            const long long nodes_generated = static_cast<long long>(hybrid->nodePool().size());
            std::unordered_map<int32_t, size_t> by_id;
            for (size_t i = 0; i < rec->ev.size(); ++i) by_id[rec->ev[i].id] = i;
            const auto & lib = *b.kin.prim_lib;
            long long matched = 0, via_blocked = 0, out_of_grid = 0;
            struct Group { std::vector<double> dist; double off = 0, cost = 0; };
            Group grp[2];  // 0 = A (FAST_ACCEPT / SCAN_CLEAR), 1 = B (SCAN_BLOCKED)
            const GridMetaData gm = b.snapshot->grid_.getMetadata();
            for (const auto & e : rec->ev) {
                if (e.parent_id < 0) continue;
                const auto pit = by_id.find(e.parent_id);
                if (pit == by_id.end()) continue;
                const SE2State & par = rec->ev[pit->second].state;
                const int16_t k = lib.headingBin(par.theta);
                const GridCoords pc = b.snapshot->grid_.getMetadata().worldToGrid(par.x, par.y);
                for (const auto & prim : lib.primitives()) {
                    const SE2State c = b.kin.motion->space().propagate(par, prim.control);
                    if (std::abs(c.x - e.state.x) > 1e-9 || std::abs(c.y - e.state.y) > 1e-9 ||
                        std::abs(std::remainder(c.theta - e.state.theta, 2 * M_PI)) > 1e-9) continue;
                    ++matched;
                    const bool blocked = checker.checkTrajectory(prim.swept_cells[k], prim.sample_cells[k],
                                             prim.sample_clearance, pc) == TrajectoryVerdict::SCAN_BLOCKED;
                    if (blocked) ++via_blocked;
                    const GridCoords nc = gm.worldToGrid(e.state.x, e.state.y);
                    if (nc.i >= 0 && nc.i < gm.cols && nc.j >= 0 && nc.j < gm.rows) {
                        Group & g = grp[blocked ? 1 : 0];
                        g.dist.push_back(b.snapshot->edt_.distanceAt(nc.i, nc.j));
                        const double q = std::remainder(e.state.theta, M_PI / 2);
                        g.off += std::abs(q) * 180.0 / M_PI;
                        g.cost += costmap->costAt(nc.i, nc.j);
                    } else {
                        ++out_of_grid;
                    }
                    break;
                }
            }

            const char * valid = "na";
            if (path) {
                valid = pathValid(*path, checker) ? "1" : "0";
            }
            // path_hash: FNV-1a over node_points (x, y, theta) then total_cost; pose counts vs. the map.
            uint64_t path_hash = 0;
            long long poses_total = 0, poses_off_map = 0, poses_unknown = 0;
            if (path) {
                path_hash = 14695981039346656037ull;
                auto mix = [&](double d) {
                    unsigned char bytes[sizeof d];
                    std::memcpy(bytes, &d, sizeof d);
                    for (const unsigned char c : bytes) path_hash = (path_hash ^ c) * 1099511628211ull;
                };
                for (const auto & p : path->node_points) { mix(p.x); mix(p.y); mix(p.theta); }
                mix(path->total_cost);
                for (const auto & seg : path->segments)
                    for (const auto & p : seg.samples_poses) {
                        ++poses_total;
                        const GridCoords pc = gm.worldToGrid(p.x, p.y);
                        if (!gm.inBounds(pc)) ++poses_off_map;
                        else if (b.snapshot->grid_.findCellStatus(pc) == CellState::UNKNOWN) ++poses_unknown;
                    }
            }
            const double us_per_exp = st.expansions > 0 ? median(times) * 1000.0 / static_cast<double>(st.expansions) : 0.0;
            std::printf("scenario=%s h=%.4g n_headings=%d shot=%s body=%s build_ms=%.3f lib_build_ms=%.3f "
                        "cached_cells=%zu plan_ms_median=%.3f plan_ms_min=%.3f found=%d expansions=%lld "
                        "fast_accept=%lld fast_reject=%lld scan_clear=%lld scan_blocked=%lld "
                        "exact_accept=%lld exact_reject=%lld path_cost=%.6g path_segments=%zu valid=%s us_per_expansion=%.3f "
                        "prepare_ms=%.3f nodes_generated=%lld expanded_via_blocked=%lld expanded_matched=%lld h_start=%.6g h_euclid=%.6g h_start_inflated=%.6g inflated_cells=%lld "
                        "start=%.4f,%.4f,%.4f goal=%.4f,%.4f,%.4f path_hash=%016llx poses_total=%lld "
                        "poses_off_map=%lld poses_unknown=%lld duplicate_skipped=%lld wave_cells=%lld grid_cells=%lld\n",
                        spec.name.c_str(), static_cast<double>(spec.meta.resolution), nh, shot ? "on" : "off",
                        a.robot.empty() ? "scenario" : "robot", build_ms, b.kin.prim_lib->buildTimeMs(), cells,
                        median(times), *std::min_element(times.begin(), times.end()), path ? 1 : 0,
                        static_cast<long long>(st.expansions), static_cast<long long>(st.fast_accept),
                        static_cast<long long>(st.fast_reject), static_cast<long long>(st.scan_clear),
                        static_cast<long long>(st.scan_blocked), static_cast<long long>(st.exact_accept),
                        static_cast<long long>(st.exact_reject),
                        path ? path->total_cost : std::numeric_limits<double>::quiet_NaN(),
                        path ? path->segments.size() : size_t{0}, valid, us_per_exp,
                        median(prep), nodes_generated, via_blocked, matched, h_start, h_euclid, h_start_inflated, inflated_cells,
                        spec.start.x, spec.start.y, spec.start.theta, spec.goal.x, spec.goal.y, spec.goal.theta,
                        static_cast<unsigned long long>(path_hash), poses_total, poses_off_map, poses_unknown,
                        static_cast<long long>(st.duplicate_skipped), wave_cells, grid_cells);
            std::cout.flush();
            for (int gi = 0; gi < 2; ++gi) {
                Group & g = grp[gi];
                std::sort(g.dist.begin(), g.dist.end());
                const size_t n = g.dist.size();
                auto pct = [&](double q) { return n ? g.dist[static_cast<size_t>(q * (n - 1))] : 0.0; };
                std::printf("clearance group=%c count=%zu dist_min=%.4f dist_p10=%.4f dist_p50=%.4f dist_p90=%.4f "
                            "dist_max=%.4f heading_off_axis_deg_mean=%.3f costmap_mean=%.4f\n",
                            gi ? 'B' : 'A', n, pct(0), pct(0.1), pct(0.5), pct(0.9), pct(1),
                            n ? g.off / n : 0.0, n ? g.cost / n : 0.0);
            }
            std::printf("clearance group=out_of_grid count=%lld (matched expansions whose cell is outside the map; "
                        "excluded from groups A and B)\n", out_of_grid);
            std::cout.flush();
        }
    }
}

/// Times computeExactCover + checkCells for straight segments of several lengths from the start pose.
void runShotCost(const Args & a) {
    const ScenarioSpec spec = makeSpec(a, a.n_headings.front(), true);
    const RobotParams robot = buildRobotParams(spec.robot_vertices);
    OccupancyGrid occupancy(spec.meta, spec.occupancy);
    EDT edt = buildEDTGrid(occupancy, spec.edt_clamp);
    auto snapshot = std::make_shared<const MapSnapshot>(MapSnapshot{std::move(occupancy), std::move(edt)});
    const auto grid = std::shared_ptr<const OccupancyGrid>(snapshot, &snapshot->grid_);
    const auto edt_ptr = std::shared_ptr<const EDT>(snapshot, &snapshot->edt_);
    const auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(spec.diff_drive.rmin * spec.diff_drive.rmin);
    const auto space = std::make_shared<const DiffDriveStateSpace>(grid->getMetadata(), dist);
    const MotionContext motion(robot, space, grid->getMetadata());
    const CollisionChecker checker(grid, edt_ptr, robot);

    std::cout << "# shotcost length_m cells cover_us_median check_us_median\n";
    for (const double len : {0.5, 1.0, 2.0, 3.0, 5.0}) {
        const Control u{spec.diff_drive.vmax, 0.0, len / spec.diff_drive.vmax};
        std::vector<double> cover_us, check_us;
        size_t n_cells = 0;
        for (int i = 0; i < 200; ++i) {
            const auto t0 = Clock::now();
            const auto res = motion.computeExactCover(spec.start, u);
            const auto t1 = Clock::now();
            volatile bool ok = checker.checkCells(res.cells, false);
            (void)ok;
            const auto t2 = Clock::now();
            cover_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
            check_us.push_back(std::chrono::duration<double, std::micro>(t2 - t1).count());
            n_cells = res.cells.size();
        }
        std::printf("shotcost %.4g %zu %.3f %.3f\n", len, n_cells, median(cover_us), median(check_us));
    }
}

/// Per library primitive: exact-cover time vs. its rasterization-only and geometry-only parts.
void runCoverSplit(const Args & a) {
    const ScenarioSpec spec = makeSpec(a, a.n_headings.front(), true);
    Built b = build(spec);
    const auto grid = std::shared_ptr<const OccupancyGrid>(b.snapshot, &b.snapshot->grid_);
    const auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(spec.diff_drive.rmin * spec.diff_drive.rmin);
    const auto space = std::make_shared<const DiffDriveStateSpace>(grid->getMetadata(), dist);
    const MotionContext motion(b.robot, space, grid->getMetadata());
    const double h = grid->getMetadata().resolution;
    const Vec2 origin = grid->getMetadata().origin.cast<double>();

    size_t idx = 0;
    for (const auto & prim : b.kin.prim_lib->primitives()) {
        const Control & u = prim.control;
        std::vector<double> cover_us, raster_us, geom_us;
        size_t raw = 0, unique = 0, poses = 0;
        volatile size_t sink = 0;
        for (int i = 0; i < 200; ++i) {
            auto t0 = Clock::now();
            const auto res = motion.computeExactCover(spec.start, u);
            auto t1 = Clock::now();
            unique = res.cells.size();
            cover_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());

            for (const bool rasterize : {true, false}) {
                t0 = Clock::now();
                const double reach = (std::abs(u.v) + b.robot.r_out * std::abs(u.omega)) * u.dt;
                const int n = std::max(1, static_cast<int>(std::ceil(reach / h)));
                const double m_s = reach / (2.0 * n);
                const auto sp = motion.sampleTrajectory(spec.start, u);
                size_t np = 0, acc = 0;
                size_t raster_total = 0;
                auto work = [&](const SE2State & p, double margin) {
                    const auto geo = transformGeometry(b.robot, p, origin);
                    if (rasterize) raster_total += rasterizePolygon(geo.half_planes, geo.aabb, h, margin).size();
                    ++np;
                    acc += geo.half_planes.size();
                };
                for (int j = 0; j <= n; ++j) work(space->propagate(spec.start, Control{u.v, u.omega, u.dt * j / n}), m_s);
                for (const auto & p : sp) work(p, 0.0);
                t1 = Clock::now();
                sink = sink + raster_total + acc;
                (rasterize ? raster_us : geom_us).push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
                if (rasterize) { raw = raster_total; poses = np; }
            }
        }
        std::printf("coversplit prim=%zu v=%.4g omega=%.4g dt=%.4g poses=%zu raw_cells=%zu unique_cells=%zu "
                    "cover_us=%.3f raster_us=%.3f geom_us=%.3f\n",
                    idx++, u.v, u.omega, u.dt, poses, raw, unique, median(cover_us), median(raster_us), median(geom_us));
    }
}

}  // namespace

int main(int argc, char ** argv) {
    Args a;
    try {
        for (int i = 1; i < argc; ++i) {
            const std::string k = argv[i];
            auto val = [&]() -> std::string {
                if (i + 1 >= argc) throw std::invalid_argument("missing value for " + k);
                return argv[++i];
            };
            if (k == "--scenario") a.scenario = val();
            else if (k == "--robot") a.robot = val();
            else if (k == "--cost-policy") a.cost_policy = val();
            else if (k == "--planner") a.planner = val();
            else if (k == "--vmax") a.vmax = std::stod(val());
            else if (k == "--atomic-time") a.atomic_time = std::stod(val());
            else if (k == "--repeats") a.repeats = std::stoi(val());
            else if (k == "--shot") a.shot = val();
            else if (k == "--max-plan-time") a.max_plan_time = std::stod(val());
            else if (k == "--max-expansions") a.max_expansions = std::stoll(val());
            else if (k == "--start") a.start = parsePose(k, val());
            else if (k == "--goal") a.goal = parsePose(k, val());
            else if (k == "--shot-cost") a.shot_cost = true;
            else if (k == "--cover-split") a.cover_split = true;
            else if (k == "--n-headings") {
                a.n_headings.clear();
                std::stringstream ss(val());
                for (std::string t; std::getline(ss, t, ',');) a.n_headings.push_back(std::stoi(t));
            } else throw std::invalid_argument("unknown argument " + k);
        }
        if (a.scenario.empty() || a.repeats < 1 || a.n_headings.empty() ||
            (a.shot != "on" && a.shot != "off" && a.shot != "both"))
            throw std::invalid_argument("usage: see top of plan_benchmark.cpp");
        if (a.shot_cost) { runShotCost(a); return 0; }
        if (a.cover_split) { runCoverSplit(a); return 0; }
        std::cout << "# plan_benchmark; times in ms, path_cost in the planner's cost units\n";
        runSweep(a);
    } catch (const std::exception & e) {
        std::cerr << "plan_benchmark: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
