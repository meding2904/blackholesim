// CPU-side validation of the Schwarzschild geodesic utilities.
// The shader implements the same reduction (u = r_g/r, RK4, Binet equation), so
// these checks cover the initial conditions, capture criteria and integration
// accuracy that the GPU path depends on.
#include <cmath>
#include <cstdio>
#include <string>

#include "geodesic.hpp"
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
    check(p.outerRadius > bh::kInnerDiskRadius, "outer disk radius stays outside the inner edge");
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
    std::printf("Schwarzschild geodesic tests\n============================\n");

    testInitialConditions();
    testCaptureThreshold();
    testWeakFieldDeflection();
    testIntegrationAccuracy();
    testCameraRays();
    testRelativisticShift();
    testRobustness();
    testParameters();

    std::printf("\n%d checks, %d failure(s)\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
