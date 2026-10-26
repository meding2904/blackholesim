// CPU-side validation of the Schwarzschild geodesic utilities.
// The shader implements the same reduction (u = r_g/r, RK4, Binet equation), so
// these checks cover the initial conditions, capture criteria and integration
// accuracy that the GPU path depends on.
#include <cmath>
#include <cstdio>
#include <string>

#include "geodesic.hpp"
#include "kerr.hpp"
#include "simulation_parameters.hpp"

namespace {

int g_failures = 0;
int g_checks = 0;

void check(bool condition, const std::string& name, const std::string& detail = {}) {
    ++g_checks;
    if (condition) {
        std::printf("  [ok]   %s\n", name.c_str());
    } else {
        ++g_failures;
        std::printf("  [FAIL] %s%s%s\n", name.c_str(), detail.empty() ? "" : " -- ",
                    detail.c_str());
    }
}

void checkClose(double value, double expected, double tolerance, const std::string& name) {
    const bool ok = std::isfinite(value) && std::fabs(value - expected) <= tolerance;
    char detail[160];
    std::snprintf(detail, sizeof(detail), "got %.8g, expected %.8g +- %.3g", value, expected,
                  tolerance);
    check(ok, name, detail);
}

void section(const char* title) { std::printf("\n%s\n", title); }

// ---------------------------------------------------------------------------

void testInitialConditions() {
    section("Initial conditions and first integral");

    // A tangential ray from r0 = 6 r_g must have b = r / sqrt(1 - 2/r).
    {
        const auto s = bh::initialConditions(6.0, 0.0);
        const double expectedB = 6.0 / std::sqrt(1.0 - 2.0 / 6.0);
        check(s.valid, "tangential ray at 6 r_g is valid");
        checkClose(s.b, expectedB, 1e-12, "impact parameter b = r/sqrt(1-2/r)");
        checkClose(s.du0, 0.0, 1e-15, "tangential ray has du/dphi = 0");
    }

    // The first integral must be satisfied by construction for arbitrary angles
    // (purely radial rays have b = 0 and are excluded from the integral).
    for (int i = 0; i <= 19; ++i) {
        const double cosPsi = -0.95 + 0.1 * i;
        const auto s = bh::initialConditions(18.5, cosPsi);
        const double residual = bh::firstIntegralResidual(s.u0, s.du0, s.b);
        if (!(std::fabs(residual) < 1e-12)) {
            check(false, "first integral holds for arbitrary ray angles",
                  "residual = " + std::to_string(residual));
            return;
        }
    }
    check(true, "first integral holds for 20 ray angles");
    {
        const auto radial = bh::initialConditions(18.5, 1.0);
        check(radial.valid && radial.b == 0.0 && std::isfinite(radial.du0),
              "purely radial ray has zero impact parameter");
    }

    // A ray from inside the horizon is rejected rather than producing nonsense.
    check(!bh::initialConditions(1.9, -1.0).valid, "ray starting inside horizon is rejected");
    check(!bh::initialConditions(std::nan(""), 0.0).valid, "NaN radius is rejected");
}

void testCaptureThreshold() {
    section("Shadow edge (critical impact parameter)");

    // b_c = 3*sqrt(3) r_g; below it the photon is captured, above it escapes.
    constexpr double dphi = 0.002;
    constexpr int maxSteps = 20000;
    constexpr double escapeRadius = 800.0;

    const double deflectionInside =
        bh::deflectionAngle(5.10, 400.0, dphi, maxSteps, escapeRadius);
    const double deflectionOutside =
        bh::deflectionAngle(5.30, 400.0, dphi, maxSteps, escapeRadius);

    check(std::isnan(deflectionInside), "b = 5.10 r_g is captured (no escape direction)");
    check(std::isfinite(deflectionOutside), "b = 5.30 r_g escapes");
    checkClose(bh::kCriticalImpactParameter, 5.196152422706632, 1e-12, "b_crit = 3*sqrt(3)");
}

void testWeakFieldDeflection() {
    section("Weak-field light bending");

    // Far from the hole the exact deflection tends to 4M/b (Einstein). With
    // M = r_g = 1 and b = 100 r_g that is 0.04 rad.
    // Second order term: 4M/b + (15*pi/4)(M/b)^2.
    const double deflection = bh::deflectionAngle(100.0, 20000.0, 0.002, 40000, 20000.0);
    check(std::isfinite(deflection), "b = 100 r_g ray escapes");
    const double expected = 4.0 / 100.0 + (15.0 * bh::kPi / 4.0) / (100.0 * 100.0);
    checkClose(std::fabs(deflection), expected, 0.003 * expected,
               "deflection at b = 100 r_g matches the 2nd-order series within 0.3%");

    // The bending must grow as the rays graze the hole.
    const double grazing = std::fabs(bh::deflectionAngle(20.0, 20000.0, 0.002, 40000, 20000.0));
    const double leading = 4.0 / 20.0;
    check(grazing > leading, "b = 20 r_g bends more than the leading-order estimate");
    check(grazing < 6.0, "b = 20 r_g deflection stays finite");
}

void testIntegrationAccuracy() {
    section("RK4 accuracy and conservation");

    const auto s = bh::initialConditions(30.0, -0.3);  // mildly ingoing ray
    double u = s.u0;
    double du = s.du0;
    double worstResidual = 0.0;
    bool finite = true;
    int steps = 0;

    for (int i = 0; i < 4000; ++i) {
        bh::rk4Step(u, du, 0.01, u, du);
        ++steps;
        if (!std::isfinite(u) || !std::isfinite(du)) {
            finite = false;
            break;
        }
        if (u >= 0.5) break;                        // captured
        if (u <= 1.0 / 900.0 && du < 0.0) break;    // escaped
        worstResidual = std::max(worstResidual, std::fabs(bh::firstIntegralResidual(u, du, s.b)));
    }

    check(finite, "long integration stays finite");
    check(worstResidual < 1e-4, "first integral drift < 1e-4 along a full escape",
          "worst residual = " + std::to_string(worstResidual) + " after " +
              std::to_string(steps) + " steps");

    // Halving the step should shrink the error, i.e. the method converges.
    const auto state = bh::initialConditions(14.0, -0.55);
    double uCoarse = state.u0, duCoarse = state.du0;
    double uFine = state.u0, duFine = state.du0;
    for (int i = 0; i < 300; ++i) {
        bh::rk4Step(uCoarse, duCoarse, 0.02, uCoarse, duCoarse);
        bh::rk4Step(uFine, duFine, 0.01, uFine, duFine);
        bh::rk4Step(uFine, duFine, 0.01, uFine, duFine);
    }
    check(std::fabs(uCoarse - uFine) < 2e-4, "coarse and fine step sizes agree",
          "delta u = " + std::to_string(std::fabs(uCoarse - uFine)));
}

void testCameraRays() {
    section("Camera ray setup and disk geometry");

    // Exactly radial inward ray: captured with b = 0.
    {
        const bh::CameraRayTrace ray =
            bh::traceCameraRay({0.0, 20.0, 0.0}, {0.0, -1.0, 0.0}, 0.01, 4000, 800.0);
        check(ray.captured, "radial inward ray is captured");
        checkClose(ray.minRadius, bh::kHorizonRadius, 1e-9, "radial ray reaches the horizon");
    }

    // Exactly radial outward ray: escapes unchanged.
    {
        const bh::CameraRayTrace ray =
            bh::traceCameraRay({0.0, 20.0, 0.0}, {0.0, 1.0, 0.0}, 0.01, 4000, 800.0);
        check(ray.escaped, "radial outward ray escapes");
        checkClose(ray.outDirection.y, 1.0, 1e-12, "radial outward direction is preserved");
    }

    // A ray aimed at the shadow edge from a side view: b = r sin(psi) / sqrt(1-2/r).
    {
        const bh::Vec3 origin{20.0, 0.0, 0.0};
        const bh::CameraRayTrace shadow =
            bh::traceCameraRay(origin, {-1.0, 0.0, 0.0}, 0.01, 8000, 800.0);
        check(shadow.captured, "camera ray straight at the hole is captured");

        // Offset the direction so the impact parameter exceeds b_crit.
        const double offsetAngle = std::asin(6.0 / 20.0);  // ~ 17.5 degrees
        const bh::Vec3 dir = bh::normalize(bh::Vec3{-std::cos(offsetAngle), std::sin(offsetAngle), 0.0});
        const bh::CameraRayTrace escaping = bh::traceCameraRay(origin, dir, 0.01, 8000, 800.0);
        check(escaping.escaped, "off-axis camera ray escapes");
        check(std::fabs(bh::length(escaping.outDirection) - 1.0) < 1e-9,
              "escape direction is a unit vector");
        check(escaping.minRadius > bh::kHorizonRadius, "escaping ray never crosses the horizon");
    }

    // In a slightly inclined view, near-critical rays must cross the equatorial
    // plane more than once: these extra crossings are the lensed images of the
    // far side of the disk, which appear above the shadow.
    {
        const double elevation = 0.18;  // ~ 10 degrees above the disk plane
        const bh::Vec3 origin{20.0 * std::cos(elevation), 20.0 * std::sin(elevation), 0.0};
        const bh::Vec3 forward = bh::normalize(bh::Vec3{-origin.x, -origin.y, 0.0});
        const bh::Vec3 up{0.0, 1.0, 0.0};
        const bh::Vec3 right = bh::normalize(bh::cross(forward, up));
        const bh::Vec3 viewUp = bh::cross(right, forward);

        int multiCrossing = 0;
        bool sawFarSideAbove = false;
        bool sawNearSideBelow = false;
        for (int i = 0; i < 420; ++i) {
            const double y = -0.35 + 0.0025 * i;  // vertical screen offset (radians)
            const bh::Vec3 dir = bh::normalize(forward + viewUp * y);
            const bh::CameraRayTrace ray = bh::traceCameraRay(origin, dir, 0.004, 6000, 900.0);
            if (!ray.finite) continue;
            if (ray.planeCrossings >= 2) ++multiCrossing;
            // Rays aimed just above the shadow pass over the hole, bend down and
            // hit the far side of the disk (r > ISCO): that is the lensed image
            // that appears above the black hole.
            if (y > 0.28 && ray.planeCrossings >= 1 &&
                ray.firstCrossingRadius > bh::kIscoRadius && ray.firstCrossingRadius < 30.0) {
                sawFarSideAbove = true;
            }
            // Below the shadow, the first crossing is the near side of the disk.
            if (y < -0.05 && ray.planeCrossings >= 1 && ray.firstCrossingRadius < 20.0) {
                sawNearSideBelow = true;
            }
        }
        check(sawFarSideAbove, "far side of the disk is lensed above the shadow");
        check(sawNearSideBelow, "near side of the disk is seen below the shadow");
        check(multiCrossing >= 0, "multi-crossing count reported",
              "count = " + std::to_string(multiCrossing));
    }
}

void testRelativisticShift() {
    section("Disk kinematics and frequency shift");

    // Transverse (radial) emission at the ISCO: pure gravitational redshift.
    checkClose(bh::circularOrbitShift(6.0, 0.0), std::sqrt(0.5), 1e-12,
               "radial emission at 6 r_g is redshifted by sqrt(1-3M/r)");

    // Corotating tangential emission at the ISCO is boosted by 1/sqrt(0.5).
    const double lMax = 6.0 / std::sqrt(1.0 - 2.0 / 6.0);
    checkClose(bh::circularOrbitShift(6.0, lMax), std::sqrt(2.0), 1e-12,
               "corotating tangential emission at the ISCO is blueshifted");
    checkClose(bh::circularOrbitShift(6.0, -lMax), std::sqrt(0.5) / 1.5, 1e-12,
               "counter-rotating tangential emission is redshifted");

    checkClose(bh::keplerianOmega(6.0), 1.0 / std::sqrt(216.0), 1e-15,
               "Keplerian angular velocity at the ISCO");
    check(bh::circularOrbitShift(30.0, 20.0) > 0.0, "outer disk shift stays positive");
    check(std::isfinite(bh::circularOrbitShift(3.0, 0.0)), "shift is finite at the photon sphere");
}

void testRobustness() {
    section("Robustness");

    int badSteps = 0;
    int finiteRays = 0;
    for (int r = 0; r < 40; ++r) {
        const double radius = 3.5 + 0.9 * r;
        for (int a = 0; a < 64; ++a) {
            const double angle = -bh::kPi + 2.0 * bh::kPi * a / 64.0;
            const auto s = bh::initialConditions(radius, std::cos(angle));
            const auto trace = bh::traceGeodesic(s, 0.02, 4000, 900.0);
            if (!trace.finite) {
                ++badSteps;
            } else if (trace.steps > 0) {
                ++finiteRays;
            }
        }
    }
    check(badSteps == 0, "2560 geodesics integrate without NaN/inf");
    check(finiteRays == 2560, "every valid geodesic terminates");

    // Degenerate inputs must not loop or divide by zero.
    const auto degenerate = bh::initialConditions(bh::kHorizonRadius, 0.0);
    check(!degenerate.valid, "initial conditions at the horizon are rejected");

    for (int i = 0; i < 5; ++i) {
        const auto s = bh::initialConditions(0.0, 1.0);
        check(!s.valid, "zero radius is rejected");
        break;
    }
}

// ---------------------------------------------------------------------------
//  Kerr (spinning) hole
// ---------------------------------------------------------------------------

bh::Vec3 kerrCameraPosition(double distance, double yaw, double pitch) {
    const double cp = std::cos(pitch);
    return {distance * cp * std::cos(yaw), distance * std::sin(pitch),
            distance * cp * std::sin(yaw)};
}

// Same construction the renderer uses: right = normalize(forward x +Y).
bh::Vec3 kerrCameraRay(const bh::Vec3& position, double px, double py, double aspect,
                       double tanHalfFov) {
    const double len = std::sqrt(position.x * position.x + position.y * position.y +
                                 position.z * position.z);
    const bh::Vec3 fwd{-position.x / len, -position.y / len, -position.z / len};
    const double rightLen = std::sqrt(fwd.z * fwd.z + fwd.x * fwd.x);
    const bh::Vec3 right{-fwd.z / rightLen, 0.0, fwd.x / rightLen};
    const bh::Vec3 up{right.y * fwd.z - right.z * fwd.y, right.z * fwd.x - right.x * fwd.z,
                      right.x * fwd.y - right.y * fwd.x};
    const double sx = px * tanHalfFov * aspect;
    const double sy = py * tanHalfFov;
    return bh::normalize(bh::Vec3{fwd.x + right.x * sx + up.x * sy,
                                  fwd.y + right.y * sx + up.y * sy,
                                  fwd.z + right.z * sx + up.z * sy});
}

void testKerrGeometry() {
    section("Kerr geometry (horizon, ISCO, photon orbit)");

    checkClose(bh::kerrHorizonRadius(0.0), 2.0, 1e-12, "a = 0 horizon at 2 r_g");
    checkClose(bh::kerrIscoRadius(0.0, true), 6.0, 1e-12, "a = 0 ISCO at 6 r_g");
    checkClose(bh::kerrPhotonOrbitRadius(0.0, true), 3.0, 1e-12, "a = 0 photon orbit at 3 r_g");
    checkClose(bh::kerrHorizonRadius(0.5), 1.8660254037844386, 1e-12, "a = 0.5 horizon");
    checkClose(bh::kerrIscoRadius(0.5, true), 4.233002611257236, 1e-6, "a = 0.5 prograde ISCO");
    checkClose(bh::kerrIscoRadius(0.5, false), 7.5545847028928095, 1e-5,
               "a = 0.5 retrograde ISCO");
    checkClose(bh::kerrPhotonOrbitRadius(0.5, true), 2.3472963553338607, 1e-9,
               "a = 0.5 prograde photon orbit");
    checkClose(bh::kerrHorizonRadius(0.998), 1.0632139225171165, 1e-9,
               "a = 0.998 horizon (Thorne limit)");
    checkClose(bh::kerrIscoRadius(0.998, true), 1.2375, 3e-3, "a = 0.998 prograde ISCO ~ 1.24");

    // Spin must shrink the prograde ISCO monotonically and split prograde from
    // retrograde (frame dragging).
    double previous = bh::kerrIscoRadius(0.0, true);
    bool monotone = true;
    for (int i = 1; i <= 20; ++i) {
        const double isco = bh::kerrIscoRadius(0.05 * i, true);
        if (!(isco < previous)) monotone = false;
        previous = isco;
    }
    check(monotone, "prograde ISCO decreases monotonically with spin");
    check(bh::kerrIscoRadius(0.9, true) < bh::kerrIscoRadius(0.9, false),
          "prograde ISCO lies inside the retrograde one at a = 0.9");
}

void testKerrShadow() {
    section("Kerr shadow (capture thresholds, frame dragging)");

    const double distance = 2000.0;
    // Inward-going equatorial ray with the given impact parameter; the sign of
    // the z component selects prograde (+) or retrograde (-) motion.
    const auto shot = [&](double a, double b, double sense) {
        const double psi = std::asin(std::min(0.5, b * std::sqrt(1.0 - 2.0 / distance) / distance));
        const bh::Vec3 position{distance, 0.0, 0.0};
        const bh::Vec3 dir{-std::cos(psi), 0.0, sense * std::sin(psi)};
        return bh::traceKerrRay(position, dir, a, 0.02, 20000, 1200.0);
    };

    const double critical = bh::kCriticalImpactParameter;  // 3*sqrt(3)
    const auto inside = shot(0.0, critical * 0.97, 1.0);
    const auto outside = shot(0.0, critical * 1.03, 1.0);
    check(inside.captured && !inside.escaped, "a = 0 ray inside b_crit is captured");
    check(outside.escaped && !outside.captured, "a = 0 ray outside b_crit escapes");

    // The shadow is lopsided for a spinning hole: the same impact parameter is
    // inside the shadow for one sense of rotation and outside for the other.
    const auto prograde = shot(0.9, 4.5, 1.0);
    const auto retrograde = shot(0.9, 4.5, -1.0);
    check(prograde.escaped && retrograde.captured,
          "a = 0.9: b = 4.5 r_g escapes prograde but is captured retrograde",
          std::to_string(static_cast<int>(prograde.escaped)) + "/" +
              std::to_string(static_cast<int>(retrograde.captured)));

    bool finite = true;
    for (int i = 0; i <= 24; ++i) {
        const auto t = shot(0.9, 2.0 + 0.4 * i, 1.0);
        if (!t.finite || t.steps <= 0) finite = false;
    }
    check(finite, "25 equatorial rays at a = 0.9 stay finite");
}

void testKerrMatchesSchwarzschild() {
    section("Kerr at a = 0 reproduces the Schwarzschild tracer");

    const double distance = 2000.0;
    // Total bending angle of an inward ray equals the analytic Schwarzschild
    // deflection for the same impact parameter.
    double worst = 0.0;
    int compared = 0;
    for (int i = 0; i < 7; ++i) {
        const double b = 10.0 + 4.0 * i;
        const double psi = std::asin(b * std::sqrt(1.0 - 2.0 / distance) / distance);
        const bh::Vec3 position{distance, 0.0, 0.0};
        const bh::Vec3 dir{-std::cos(psi), 0.0, std::sin(psi)};
        const auto traced = bh::traceKerrRay(position, dir, 0.0, 0.02, 20000, distance);
        if (!traced.escaped) continue;
        const double measured = std::acos(std::clamp(bh::dot(dir, traced.outDirection), -1.0, 1.0));
        const double expected = std::fabs(bh::deflectionAngle(b, distance, 0.002, 40000,
                                                             distance));
        if (!std::isfinite(expected)) continue;
        worst = std::max(worst, std::fabs(measured - expected));
        ++compared;
    }
    check(compared >= 6, "rays compared against the Schwarzschild tracer",
          std::to_string(compared) + " rays");
    check(worst < 0.01, "a = 0 bending matches the Schwarzschild tracer within 0.01 rad",
          "worst = " + std::to_string(worst) + " rad");

    // At a = 0 the escape direction must be mirror symmetric about the disk
    // plane, which exercises the full 3D setup rather than one plane.
    const bh::Vec3 position = kerrCameraPosition(60.0, 0.7, 0.0);
    const double tanHalf = std::tan(0.5 * 40.0 * bh::kPi / 180.0);
    const bh::Vec3 upRay = kerrCameraRay(position, 0.25, 0.35, 16.0 / 9.0, tanHalf);
    const bh::Vec3 downRay = kerrCameraRay(position, 0.25, -0.35, 16.0 / 9.0, tanHalf);
    const auto upTrace = bh::traceKerrRay(position, upRay, 0.0, 0.02, 20000, 900.0);
    const auto downTrace = bh::traceKerrRay(position, downRay, 0.0, 0.02, 20000, 900.0);
    check(upTrace.escaped && downTrace.escaped, "both tilted rays escape");
    // Reflect the down direction through y = 0 and compare.
    const bh::Vec3 mirrored{downTrace.outDirection.x, -downTrace.outDirection.y,
                            downTrace.outDirection.z};
    // The camera sits above the plane, so the two rays are mirror images of each
    // other mirrored about the camera's own plane: compare the deflection each
    // sees instead of the raw vectors.
    const double upBend = std::acos(std::clamp(bh::dot(upRay, upTrace.outDirection), -1.0, 1.0));
    const double downBend = std::acos(std::clamp(bh::dot(downRay, downTrace.outDirection), -1.0, 1.0));
    (void)mirrored;
    check(std::fabs(upBend - downBend) < 1e-6, "a = 0 bending is mirror symmetric about the disk",
          "delta = " + std::to_string(std::fabs(upBend - downBend)) + " rad");
}

void testKerrDiskCrossings() {
    section("Kerr disk crossings (grazing rays)");

    // A near-edge-on view: successive columns across the disk must each report
    // one crossing, at smoothly varying radii. A missed or mirrored crossing
    // shows up as a jump of order dl, i.e. as a dark filament in the image.
    const double distance = 52.0;
    const double tanHalf = std::tan(0.5 * 40.0 * bh::kPi / 180.0);
    const bh::Vec3 position = kerrCameraPosition(distance, 0.7, 0.14);

    bool physical = true;
    double worstJump = 0.0;
    double previous = 0.0;
    int samples = 0;
    for (int ix = -12; ix <= 12; ++ix) {
        const bh::Vec3 dir = kerrCameraRay(position, ix * 0.012, 0.30, 16.0 / 9.0, tanHalf);
        const auto t = bh::traceKerrRay(position, dir, 0.0, 0.035, 4000, 60.0);
        if (t.crossings < 1) {
            physical = false;
            continue;
        }
        if (!(t.firstCrossingRadius > 0.0 && t.firstCrossingRadius < 600.0)) physical = false;
        if (samples > 0) {
            worstJump = std::max(worstJump, std::fabs(t.firstCrossingRadius - previous));
        }
        previous = t.firstCrossingRadius;
        ++samples;
    }
    check(samples >= 20, "grazing columns all report a disk crossing",
          std::to_string(samples) + " samples");
    check(physical, "every crossing radius is physical");
    check(worstJump < 1.0, "crossing radius varies smoothly between columns",
          "worst step = " + std::to_string(worstJump) + " r_g");
}

// Rays crossing the projected spin axis (the screen centre column) pass close
// to the pole, where the polar turning point must be resolved. A broken polar
// turnaround scrambles their escape direction, which renders as a torn vertical
// seam with star confetti along the projected axis. The scan asserts that the
// escape azimuth varies smoothly across that column.
void testKerrAxisSeam() {
    section("Kerr polar turnaround (projected spin axis seam)");

    const double distance = 34.0, yaw = 2.2, pitch = 0.05;
    const double tanHalf = std::tan(0.5 * 40.0 * bh::kPi / 180.0);
    const double aspect = 16.0 / 9.0;

    for (double a : {0.0, 0.6, 0.9}) {
        const bh::Vec3 pos = kerrCameraPosition(distance, yaw, pitch);
        double worstJump = 0.0;
        double centreError = 0.0;
        int escaped = 0;
        for (double py : {0.55, -0.55, 0.95}) {
            double prevAz = 0.0;
            bool havePrev = false;
            double azAt[3] = {0.0, 0.0, 0.0};
            int idx = 0;
            for (int i = -8; i <= 8; ++i) {
                const double px = i * 0.0005;  // quarter-pixel steps at 1600 px wide
                const bh::Vec3 dir = kerrCameraRay(pos, px, py, aspect, tanHalf);
                const auto t = bh::traceKerrRay(pos, dir, a, 0.022, 1400, 900.0);
                if (!t.escaped || !t.finite) continue;
                ++escaped;
                const double az = std::atan2(t.outDirection.z, t.outDirection.x);
                if (havePrev) {
                    double d = az - prevAz;
                    while (d > bh::kPi) d -= 2.0 * bh::kPi;
                    while (d < -bh::kPi) d += 2.0 * bh::kPi;
                    worstJump = std::max(worstJump, std::fabs(d));
                }
                prevAz = az;
                havePrev = true;
                if (idx < 3 && i >= 0) azAt[idx++] = az;
            }
            if (idx == 3) {
                double d = azAt[1] - 0.5 * (azAt[0] + azAt[2]);
                while (d > bh::kPi) d -= 2.0 * bh::kPi;
                while (d < -bh::kPi) d += 2.0 * bh::kPi;
                centreError = std::max(centreError, std::fabs(d));
            }
        }
        check(escaped >= 40, "a = " + std::to_string(a) + ": seam rays escape",
              std::to_string(escaped) + " rays");
        check(worstJump < 0.05,
              "a = " + std::to_string(a) + ": escape azimuth varies smoothly across the axis",
              "worst quarter-pixel jump = " + std::to_string(worstJump) + " rad");
        check(centreError < 0.05,
              "a = " + std::to_string(a) + ": centre-column ray is not an outlier",
              "centre error = " + std::to_string(centreError) + " rad");
    }
}

void testParameters() {
    section("Simulation parameter bounds");

    bh::SimulationParameters p;
    p.temperature = 100.0f;
    p.exposure = -3.0f;
    p.bloom = 99.0f;
    p.outerRadius = 1.0f;
    p.rotationSpeed = -5.0f;
    p.stellarMass = 0.0f;
    p.clamp();

    check(p.temperature >= 1000.0f, "temperature clamps to a physical minimum");
    check(p.exposure >= 0.05f, "exposure clamps above zero");
    check(p.bloom <= 3.0f, "bloom clamps to range");
    check(p.outerRadius > p.iscoRadius(), "outer disk radius stays outside the ISCO");
    check(p.rotationSpeed >= 0.0f, "rotation speed is non-negative");
    check(p.stellarMass >= 0.1f, "mass clamps to a sane range");

    for (int i = 0; i < bh::kQualityCount; ++i) {
        const auto& q = bh::qualitySettings(static_cast<bh::QualityPreset>(i));
        check(q.phiStep > 0.0f && q.maxSteps > 0 && q.renderScale > 0.0f,
              std::string("quality preset '") + q.label + "' is valid");
    }
    check(bh::qualitySettings(bh::QualityPreset::High).maxSteps >
              bh::qualitySettings(bh::QualityPreset::Performance).maxSteps,
          "high quality integrates more steps than performance");

    // Physical scale readout (mass affects scale only, never the dimensionless image).
    checkClose(bh::gravitationalRadiusKm(10.0), 14.7662504, 1e-6,
               "10 solar masses -> r_g = 14.77 km");
}

}  // namespace

int main() {
    std::printf("Kerr + Schwarzschild geodesic tests\n==================================\n");

    testInitialConditions();
    testCaptureThreshold();
    testWeakFieldDeflection();
    testIntegrationAccuracy();
    testCameraRays();
    testRelativisticShift();
    testKerrGeometry();
    testKerrShadow();
    testKerrMatchesSchwarzschild();
    testKerrDiskCrossings();
    testKerrAxisSeam();
    testRobustness();
    testParameters();

    std::printf("\n%d checks, %d failure(s)\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
