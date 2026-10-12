#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "core/control/stop_zone.hpp"
#include "core/robot/robot_params.hpp"

/*
motionBlocked properties. The oracle (below) is an RK4 integration of the stop that shares no code with the
implementation: reaction at the commanded twist, then v and omega each braking to zero at their own rate,
then a pad phase from the rest pose at the commanded twist for forward_pad / S, S = |v| + r_out |w|.
  P1 cover   : a point inside the unpadded footprint at the oracle pose at any time in the whole motion => blocked.
  P2 bound   : blocked => the point is within D of the unpadded footprint at some oracle pose,
               D = side_pad (1/sin(a_min/2) + 1/2) + S dt_grid.
  P4 reach   : blocked => |p| <= 0.31*sqrt(2) + |v|(t_r + max(|v|/2a, |w|/2alpha)) + p_f.
  P5 real stop: every point the true footprint touches during [0, T_stop] is reported blocked
               (also with side_pad in [0.5, 2] m and decelerations in [0.2, 4]).
  P6 not large: a point farther than r_out + sqrt(2) p_s + S T_stop + p_f from the origin is never blocked.
  P3 monotone: straight motion only; blocked stays blocked with larger reaction time, smaller
               linear_decel, larger forward_pad, larger side_pad.
*/

namespace {

const RobotParams & robot() {
    static const RobotParams r = buildRobotParams({Vec2(0.3, 0.3), Vec2(-0.3, 0.3), Vec2(-0.3, -0.3), Vec2(0.3, -0.3)});
    return r;
}

const StopZoneParams kP{1.5, 2.5, 0.02, 0.01};

struct Case { double v, w, tr; Vec2 p; };

Case gen() {
    Case c;
    // A quarter of the cases draw v from [-0.01, 0.01], half draw omega from [-0.05, 0.05].
    const int vk = *rc::gen::inRange(0, 4), wk = *rc::gen::inRange(0, 2);
    c.v = vk == 0 ? *rc::gen::map(rc::gen::inRange(-1000, 1001), [](int i) { return i / 100000.0; })
                  : *rc::gen::map(rc::gen::inRange(-600, 601), [](int i) { return i / 1000.0; });
    c.w = wk == 0 ? *rc::gen::map(rc::gen::inRange(-5000, 5001), [](int i) { return i / 100000.0; })
                  : *rc::gen::map(rc::gen::inRange(-2000, 2001), [](int i) { return i / 1000.0; });
    c.tr = *rc::gen::map(rc::gen::inRange(0, 501), [](int i) { return i / 1000.0; });
    const double px = *rc::gen::map(rc::gen::inRange(-1500, 1501), [](int i) { return i / 1000.0; });
    const double py = *rc::gen::map(rc::gen::inRange(-1500, 1501), [](int i) { return i / 1000.0; });
    c.p = Vec2(px, py);
    if (c.p.norm() > 1.5) c.p *= 1.5 / c.p.norm();
    return c;
}

bool inside(const Vec2 & q, double slack) {
    for (const HalfPlane & e : robot().half_planes)
        if (!(e.normal.dot(q) <= e.offset + slack)) return false;
    return true;
}

// ---- independent oracle: RK4 integration of the braking profile ---------------------------------

struct Oracle { double v, w, tr, av, aw; };

double stopTime(const Oracle & o) { return o.tr + std::max(std::abs(o.v) / o.av, std::abs(o.w) / o.aw); }

// Speed after braking time s (>= 0): linear and angular speed each fall to zero at their own rate.
double brake(double x, double a, double s) {
    return std::abs(x) > a * s ? x - std::copysign(a * s, x) : 0.0;
}

void speedsAt(const Oracle & o, double t, double & v, double & w) {
    const double s = std::max(0.0, t - o.tr);
    v = brake(o.v, o.av, s);
    w = brake(o.w, o.aw, s);
}

struct Pose { double x, y, phi; };

Pose rk4Step(const Oracle & o, const Pose & p, double t, double h) {
    auto f = [&](double tt, const Pose & q) {
        double v, w;
        speedsAt(o, tt, v, w);
        return Pose{v * std::cos(q.phi), v * std::sin(q.phi), w};
    };
    auto add = [](const Pose & q, const Pose & d, double k) { return Pose{q.x + k * d.x, q.y + k * d.y, q.phi + k * d.phi}; };
    const Pose k1 = f(t, p), k2 = f(t + h / 2, add(p, k1, h / 2)), k3 = f(t + h / 2, add(p, k2, h / 2)),
               k4 = f(t + h, add(p, k3, h));
    return Pose{p.x + h / 6 * (k1.x + 2 * k2.x + 2 * k3.x + k4.x),
                p.y + h / 6 * (k1.y + 2 * k2.y + 2 * k3.y + k4.y),
                p.phi + h / 6 * (k1.phi + 2 * k2.phi + 2 * k3.phi + k4.phi)};
}

// Poses at the requested ascending times, dt = 1e-3 s steps with a short final step to each time.
std::vector<Pose> oraclePoses(const Oracle & o, const std::vector<double> & times) {
    constexpr double dt = 1e-3;
    std::vector<Pose> out;
    Pose p{0.0, 0.0, 0.0};
    double tc = 0.0;
    for (double t : times) {
        while (t - tc > dt) { p = rk4Step(o, p, tc, dt); tc += dt; }
        out.push_back(t - tc > 0.0 ? rk4Step(o, p, tc, t - tc) : p);
    }
    return out;
}

// Body point b at oracle pose q, expressed in the start frame.
Vec2 toStartFrame(const Pose & q, const Vec2 & b) {
    return Vec2(q.x + std::cos(q.phi) * b.x() - std::sin(q.phi) * b.y(),
                q.y + std::sin(q.phi) * b.x() + std::cos(q.phi) * b.y());
}

// Pose at pad-phase time tau after rest pose r: constant commanded twist, exact arc.
Pose padPose(const Oracle & o, const Pose & r, double tau) {
    double x, y, phi;
    if (o.w == 0.0) { x = o.v * tau; y = 0.0; phi = 0.0; }
    else { x = (o.v / o.w) * std::sin(o.w * tau); y = (o.v / o.w) * (1.0 - std::cos(o.w * tau)); phi = o.w * tau; }
    return Pose{r.x + std::cos(r.phi) * x - std::sin(r.phi) * y, r.y + std::sin(r.phi) * x + std::cos(r.phi) * y, r.phi + phi};
}

double padDuration(const Oracle & o, const StopZoneParams & q) {
    return q.forward_pad / (std::abs(o.v) + robot().r_out * std::abs(o.w));
}

// Poses over the whole motion: 1e-3 s grid through the stop, then the pad phase in at most 2000 steps.
// dtMax receives the largest time spacing used.
std::vector<Pose> motionPoses(const Oracle & o, const StopZoneParams & q, double & dtMax) {
    const double Ts = stopTime(o);
    std::vector<double> times;
    for (int k = 0; k * 1e-3 < Ts; ++k) times.push_back(k * 1e-3);
    times.push_back(Ts);
    std::vector<Pose> out = oraclePoses(o, times);
    const Pose rest = out.back();
    const double pd = padDuration(o, q);
    const int n = std::max(1, std::min(2000, static_cast<int>(std::ceil(pd / 1e-3))));
    for (int i = 1; i <= n; ++i) out.push_back(padPose(o, rest, pd * i / n));
    dtMax = std::max(1e-3, pd / n);
    return out;
}

// Distance from b to the convex footprint polygon: 0 inside, else the nearest edge segment.
double distToFootprint(const Vec2 & b) {
    if (inside(b, 0.0)) return 0.0;
    const auto & vs = robot().vertices;
    double best = 1e300;
    for (size_t i = 0; i < vs.size(); ++i) {
        const Vec2 a = vs[i], e = vs[(i + 1) % vs.size()] - a;
        const double u = std::clamp((b - a).dot(e) / e.dot(e), 0.0, 1.0);
        best = std::min(best, (b - (a + u * e)).norm());
    }
    return best;
}

}  // namespace

RC_GTEST_PROP(StopZoneProperty, coversUnpaddedFootprint, ()) {
    const Case c = gen();
    RC_PRE(!(c.v == 0.0 && c.w == 0.0));
    const Oracle o{c.v, c.w, c.tr, kP.linear_decel, kP.angular_decel};
    const double Ts = stopTime(o), pd = padDuration(o, kP);
    const double t = (Ts + pd) * *rc::gen::map(rc::gen::inRange(0, 1001), [](int i) { return i / 1000.0; });
    const double bx = *rc::gen::map(rc::gen::inRange(-300, 301), [](int i) { return i / 1000.0; });
    const double by = *rc::gen::map(rc::gen::inRange(-300, 301), [](int i) { return i / 1000.0; });
    const Pose q = t <= Ts ? oraclePoses(o, {t})[0] : padPose(o, oraclePoses(o, {Ts})[0], t - Ts);
    RC_ASSERT(motionBlocked(robot(), {toStartFrame(q, Vec2(bx, by))}, c.v, c.w, c.tr, kP));
}

RC_GTEST_PROP(StopZoneProperty, blockedImpliesWithinPaddedFootprint, ()) {
    const Case c = gen();
    RC_PRE(!(c.v == 0.0 && c.w == 0.0));
    if (!motionBlocked(robot(), {c.p}, c.v, c.w, c.tr, kP)) return;
    const Oracle o{c.v, c.w, c.tr, kP.linear_decel, kP.angular_decel};
    double dtMax;
    const std::vector<Pose> poses = motionPoses(o, kP, dtMax);
    const double S = std::abs(c.v) + robot().r_out * std::abs(c.w);
    const double D = kP.side_pad * (1.0 / std::sin(M_PI / 4.0) + 0.5) + S * dtMax + 1e-9;  // square: a_min = pi/2
    double best = 1e300;
    for (const Pose & q : poses) {
        const double dx = c.p.x() - q.x, dy = c.p.y() - q.y;
        best = std::min(best, distToFootprint(Vec2(std::cos(q.phi) * dx + std::sin(q.phi) * dy,
                                                   -std::sin(q.phi) * dx + std::cos(q.phi) * dy)));
    }
    RC_ASSERT(best <= D);
}

RC_GTEST_PROP(StopZoneProperty, straightMotionMonotone, ()) {
    Case c = gen();
    c.w = 0.0;
    RC_PRE(c.v != 0.0);
    if (!motionBlocked(robot(), {c.p}, c.v, 0.0, c.tr, kP)) return;
    RC_ASSERT(motionBlocked(robot(), {c.p}, c.v, 0.0, c.tr + 0.05, kP));
    StopZoneParams q = kP;
    q.linear_decel = 1.0;
    RC_ASSERT(motionBlocked(robot(), {c.p}, c.v, 0.0, c.tr, q));
    q = kP; q.forward_pad = 0.05;
    RC_ASSERT(motionBlocked(robot(), {c.p}, c.v, 0.0, c.tr, q));
    q = kP; q.side_pad = 0.02;
    RC_ASSERT(motionBlocked(robot(), {c.p}, c.v, 0.0, c.tr, q));
}

RC_GTEST_PROP(StopZoneProperty, blockedImpliesWithinReachBound, ()) {
    const Case c = gen();
    RC_PRE(!(c.v == 0.0 && c.w == 0.0));
    if (motionBlocked(robot(), {c.p}, c.v, c.w, c.tr, kP)) {
        const double bound = 0.31 * std::sqrt(2.0) +
            std::abs(c.v) * (c.tr + std::max(std::abs(c.v) / (2.0 * kP.linear_decel),
                                             std::abs(c.w) / (2.0 * kP.angular_decel))) +
            kP.forward_pad + 1e-9;
        RC_ASSERT(c.p.norm() <= bound);
    }
}

RC_GTEST_PROP(StopZoneProperty, everyPointTouchedDuringTheStopIsBlocked, ()) {
    const double v = *rc::gen::map(rc::gen::inRange(-500, 501), [](int i) { return i / 1000.0; });
    const double w = *rc::gen::map(rc::gen::inRange(-2000, 2001), [](int i) { return i / 1000.0; });
    RC_PRE(!(v == 0.0 && w == 0.0));
    const double tr = *rc::gen::map(rc::gen::inRange(0, 501), [](int i) { return i / 1000.0; });
    StopZoneParams q{1.5, 2.5, *rc::gen::element(0.0, 0.02), 0.01};
    const Oracle o{v, w, tr, q.linear_decel, q.angular_decel};
    const double t = stopTime(o) * *rc::gen::map(rc::gen::inRange(0, 1001), [](int i) { return i / 1000.0; });
    const size_t e = *rc::gen::inRange<size_t>(0, robot().vertices.size());
    const double f = *rc::gen::map(rc::gen::inRange(0, 1001), [](int i) { return i / 1000.0; });
    const Vec2 a = robot().vertices[e], b = robot().vertices[(e + 1) % robot().vertices.size()];
    const Vec2 pt = toStartFrame(oraclePoses(o, {t})[0], a + f * (b - a));
    RC_ASSERT(motionBlocked(robot(), {pt}, v, w, tr, q));
}

RC_GTEST_PROP(StopZoneProperty, everyPointTouchedDuringTheStopIsBlockedLargeSidePad, ()) {
    const double v = *rc::gen::map(rc::gen::inRange(-500, 501), [](int i) { return i / 1000.0; });
    const double w = *rc::gen::map(rc::gen::inRange(-2000, 2001), [](int i) { return i / 1000.0; });
    RC_PRE(!(v == 0.0 && w == 0.0));
    const double tr = *rc::gen::map(rc::gen::inRange(0, 501), [](int i) { return i / 1000.0; });
    const auto in = [](int lo, int hi) { return rc::gen::map(rc::gen::inRange(lo, hi + 1), [](int i) { return i / 1000.0; }); };
    const StopZoneParams q{*in(200, 4000), *in(200, 4000), *rc::gen::element(0.0, 0.02), *in(500, 2000)};
    const Oracle o{v, w, tr, q.linear_decel, q.angular_decel};
    const double t = stopTime(o) * *in(0, 1000);
    const size_t e = *rc::gen::inRange<size_t>(0, robot().vertices.size());
    const double f = *in(0, 1000);
    const Vec2 a = robot().vertices[e], b = robot().vertices[(e + 1) % robot().vertices.size()];
    const Vec2 pt = toStartFrame(oraclePoses(o, {t})[0], a + f * (b - a));
    RC_ASSERT(motionBlocked(robot(), {pt}, v, w, tr, q));
}

TEST(StopZoneRegression, fastArcVerticesBlockedThroughoutStop) {
    const double v = 0.27, w = 1.83;
    const Oracle o{v, w, 0.2, 1.5, 2.5};
    std::vector<double> times;
    for (int k = 0; k * 0.005 < stopTime(o); ++k) times.push_back(k * 0.005);
    times.push_back(stopTime(o));
    const std::vector<Pose> poses = oraclePoses(o, times);
    for (size_t i = 0; i < poses.size(); ++i)
        for (const Vec2 & vtx : robot().vertices)
            EXPECT_TRUE(motionBlocked(robot(), {toStartFrame(poses[i], vtx)}, v, w, 0.2, kP))
                << "t=" << times[i] << " vertex=(" << vtx.x() << "," << vtx.y() << ")";
}

TEST(StopZoneRegression, grownCornerReachesBeyondOuterRadiusPlusPad) {
    // Grown square has half-width 0.31: corner at 0.31 * sqrt(2) = 0.438406 > r_out + side_pad = 0.434264.
    const StopZoneParams q{1.5, 2.5, 0.0, 0.01};
    const auto at = [](double r) { return Vec2(r * std::cos(M_PI / 4.0), r * std::sin(M_PI / 4.0)); };
    EXPECT_TRUE(motionBlocked(robot(), {at(0.43527)}, 0.0, 0.001, 0.0, q));
    EXPECT_FALSE(motionBlocked(robot(), {at(0.4390)}, 0.0, 0.001, 0.0, q));
}

RC_GTEST_PROP(StopZoneProperty, pointBeyondReachBoundIsNeverBlocked, ()) {
    const double v = *rc::gen::map(rc::gen::inRange(-500, 501), [](int i) { return i / 1000.0; });
    const double w = *rc::gen::map(rc::gen::inRange(-2000, 2001), [](int i) { return i / 1000.0; });
    RC_PRE(!(v == 0.0 && w == 0.0));
    const double tr = *rc::gen::map(rc::gen::inRange(0, 501), [](int i) { return i / 1000.0; });
    const StopZoneParams q{1.5, 2.5, *rc::gen::element(0.0, 0.02), 0.01};
    const double S = std::abs(v) + robot().r_out * std::abs(w);
    // The footprint grown by side_pad (half-planes) has its corner side_pad / sin(a_min / 2) beyond the vertex
    // along the corner direction; the square has a_min = pi/2, so side_pad * sqrt(2).
    const double bound = robot().r_out + q.side_pad * std::sqrt(2.0) +
        S * (tr + std::max(std::abs(v) / q.linear_decel, std::abs(w) / q.angular_decel)) + q.forward_pad + 1e-6;
    const double r = bound + *rc::gen::map(rc::gen::inRange(0, 501), [](int i) { return i / 1000.0; });
    const double a = *rc::gen::map(rc::gen::inRange(0, 6283), [](int i) { return i / 1000.0; });
    RC_ASSERT(!motionBlocked(robot(), {Vec2(r * std::cos(a), r * std::sin(a))}, v, w, tr, q));
}
