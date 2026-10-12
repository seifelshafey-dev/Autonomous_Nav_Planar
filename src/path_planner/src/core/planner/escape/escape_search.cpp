#include "core/planner/escape/escape_search.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>
#include "core/diagnostics/check.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/motion/world_robot_geometry.hpp"
#include "core/types/math/wrap_angle.hpp"

namespace {
/// Penetration (m) of the square cell centred at `c` past edge `hp`.
double penE(const HalfPlane & hp, const Vec2 & c, double h) {
    return (std::abs(hp.normal.x()) + std::abs(hp.normal.y())) * h / 2 - (hp.normal.dot(c) - hp.offset);
}

double pen(const WorldRobotGeometry & g, const Vec2 & c, double h) {
    double p = 1e18;
    for (const HalfPlane & hp : g.half_planes) p = std::min(p, penE(hp, c, h));
    return p;
}

bool inContact(const WorldRobotGeometry & g, const Vec2 & c, double h, double m) {
    return c.x() >= g.aabb.min.x() - h / 2 - m && c.x() <= g.aabb.max.x() + h / 2 + m &&
           c.y() >= g.aabb.min.y() - h / 2 - m && c.y() <= g.aabb.max.y() + h / 2 + m && pen(g, c, h) >= -m;
}

/// True if stepping from `parent` to `child` touches no occupied cell it was not touching, nor
/// moves further into one.
bool stepAllowed(const SE2State & parent, const SE2State & child, const CollisionChecker & checker,
                 const MotionContext & motion, double m) {
    const double h = checker.gridMeta().resolution;
    const Vec2 origin = checker.gridMeta().origin.cast<double>();
    const WorldRobotGeometry gp = transformGeometry(motion.body(), parent, origin);
    const WorldRobotGeometry gc = transformGeometry(motion.body(), child, origin);
    const int i0 = static_cast<int>(std::floor((gc.aabb.min.x() - h / 2 - m) / h));
    const int i1 = static_cast<int>(std::floor((gc.aabb.max.x() + h / 2 + m) / h));
    const int j0 = static_cast<int>(std::floor((gc.aabb.min.y() - h / 2 - m) / h));
    const int j1 = static_cast<int>(std::floor((gc.aabb.max.y() + h / 2 + m) / h));
    for (int i = i0; i <= i1; ++i)
        for (int j = j0; j <= j1; ++j) {
            const Vec2 c((i + 0.5) * h, (j + 0.5) * h);
            if (!inContact(gc, c, h, m) || checker.checkCell(GridCoords{i, j})) continue;
            if (!inContact(gp, c, h, m) || pen(gc, c, h) > pen(gp, c, h) + 1e-9) return false;
        }
    return true;
}
}  // namespace

std::optional<Path> findEscape(const SE2State & start, const CollisionChecker & checker,
                               const MotionContext & motion, const DiffDriveParams & kin,
                               const EscapeConfig & config, double stop_short) {
    CHECK(config.step_fraction > 0.0 && config.step_fraction < 1.0, "EscapeConfig.step_fraction must be in (0, 1)");
    CHECK(config.slack_cells >= 0, "EscapeConfig.slack_cells must be >= 0");
    CHECK(std::isfinite(config.max_travel_factor) && config.max_travel_factor > 0.0,
          "EscapeConfig.max_travel_factor must be finite and > 0");

    CHECK(std::isfinite(stop_short) && stop_short >= 0.0, "stop_short must be finite and >= 0");

    Path path;
    path.total_cost = 0.0;
    path.node_points.push_back(start);
    if (checker.startClear(start, 0.0)) return path;

    const double h = checker.gridMeta().resolution;
    const double delta = config.step_fraction * h;
    const double slack = config.slack_cells * h;
    const int steps = static_cast<int>(std::floor(config.max_travel_factor * motion.body().r_out / delta));

    const int extra = static_cast<int>(std::ceil(stop_short / delta - 1e-9));

    int best_k = 0;
    double best_s = 0.0;
    bool best_full = false;
    for (const double s : {-1.0, 1.0}) {
        int full = 0, plain = 0, run = 0;
        std::vector<char> slack_ok(1, 0);  // slack_ok[k]: pose k passes the start check with slack
        SE2State prev = start;
        for (int k = 1; k <= steps; ++k) {
            const SE2State p{start.x + s * k * delta * std::cos(start.theta),
                             start.y + s * k * delta * std::sin(start.theta), start.theta};
            if (!stepAllowed(prev, p, checker, motion, delta / 2)) break;
            prev = p;
            run = checker.startClear(p, 0.0) ? run + 1 : 0;
            slack_ok.push_back(run > 0 && checker.startClear(p, slack));
            if (k - extra < 1 || run < extra + 1) continue;
            if (!plain) plain = k;
            if (slack_ok[k - extra]) { full = k; break; }
        }
        const int k = full ? full : plain;
        if (!k) continue;
        const bool is_full = full != 0;
        if (best_k == 0 || (is_full && !best_full) || (is_full == best_full && k < best_k)) {
            best_k = k;
            best_s = s;
            best_full = is_full;
        }
    }
    if (best_k == 0) return std::nullopt;

    const Control u{best_s * kin.vmax, 0.0, best_k * delta / kin.vmax};
    SE2State next = motion.space().propagate(start, u);
    next.theta = wrapToPi(next.theta);
    const double cost = makeControlCost(kin)(u);
    path.segments.push_back({u, motion.sampleTrajectory(start, u), cost});
    path.total_cost = cost;
    path.node_points.push_back(next);
    return path;
}
