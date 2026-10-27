// Schwarzschild null geodesics, reduced to the equatorial plane of each ray.
//
// Units: G = c = 1 and the gravitational radius r_g = GM/c^2 is the unit of
// distance, so the mass parameter is M = 1 and
//
//     event horizon        r = 2 r_g
//     photon sphere        r = 3 r_g
//     ISCO (Schwarzschild) r = 6 r_g
//     critical impact par. b = 3*sqrt(3) r_g ~ 5.196 r_g
//
// For a photon the exact null geodesic obeys the Binet equation
//
//     d^2u/dphi^2 + u = 3 u^2,      u = r_g / r,
//
// which follows from the Schwarzschild metric with x^mu = (t, r, theta, phi):
//     (dr/dlambda)^2 = E^2 - L^2 (1 - 2u) / r^2,
//     dphi/dlambda   = L / r^2.
//
// This header is the CPU reference used by the unit tests. The GLSL shader
// (shaders/blackhole.fs) performs the identical reduction with an RK4 step, so
// the tests here validate the same equations and initial conditions that the
// GPU executes per pixel.
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace bh {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kHorizonRadius = 2.0;        // Schwarzschild radius, r_g
inline constexpr double kPhotonSphereRadius = 3.0;   // unstable photon orbit
inline constexpr double kIscoRadius = 6.0;           // marginally stable orbit
inline constexpr double kCriticalImpactParameter = 3.0 * 1.73205080756887729353;  // 3*sqrt(3)

// A ray that starts at r0 with a direction making angle psi with the *outward*
// radial direction. cosPsi is the signed radial direction cosine, so cosPsi < 0
// means the ray is travelling inward.
struct RayInitialState {
    double u0 = 0.0;    // 1 / r0
    double du0 = 0.0;   // du/dphi at the camera
    double b = 0.0;     // impact parameter |L|/E, in r_g (>= 0)
    bool valid = false;
};

// Impact parameter and du/dphi for a ray leaving a static observer at r0.
// Static observers satisfy b = r0 sin(psi) / sqrt(1 - 2 r_g/r0); the derivative
// follows from dr/dlambda = E cos(psi) and dphi/dlambda = L/r^2:
//     du/dphi = -cos(psi) / b.
inline RayInitialState initialConditions(double r0, double cosPsi) {
    RayInitialState s;
    if (!std::isfinite(r0) || r0 <= kHorizonRadius) return s;

    const double c = std::clamp(cosPsi, -1.0, 1.0);
    const double sinPsi = std::sqrt(std::max(0.0, 1.0 - c * c));
    const double u0 = 1.0 / r0;
    const double lapse = std::sqrt(std::max(1.0 - 2.0 * u0, 1e-12));

    s.u0 = u0;
    s.b = r0 * sinPsi / lapse;
    // A purely radial ray has b = 0 and no well-defined phi motion; the caller
    // must treat it as a straight plunge or escape.
    s.du0 = (s.b > 1e-12) ? -c / s.b : 0.0;
    s.valid = true;
    return s;
}

// First integral of the geodesic: du^2 = 1/b^2 - u^2 + 2u^3. It is conserved by
// the exact solution and is used by the tests to measure integration error.
inline double firstIntegralResidual(double u, double du, double b) {
    if (b <= 0.0) return std::numeric_limits<double>::quiet_NaN();
    return du * du - (1.0 / (b * b) - u * u + 2.0 * u * u * u);
}

// One RK4 step of u'' = 3u^2 - u (autonomous, so no phi dependence).
inline void rk4Step(double u, double du, double dphi, double& uOut, double& duOut) {
    const auto f = [](double x) { return 3.0 * x * x - x; };

    const double k1u = du;
    const double k1d = f(u);

    const double k2u = du + 0.5 * dphi * k1d;
    const double k2d = f(u + 0.5 * dphi * k1u);

    const double k3u = du + 0.5 * dphi * k2d;
    const double k3d = f(u + 0.5 * dphi * k2u);

    const double k4u = du + dphi * k3d;
    const double k4d = f(u + dphi * k3u);

    uOut = u + (dphi / 6.0) * (k1u + 2.0 * k2u + 2.0 * k3u + k4u);
    duOut = du + (dphi / 6.0) * (k1d + 2.0 * k2d + 2.0 * k3d + k4d);
}

// Angular position of the ray direction relative to the outward radial
// direction at the current point:
//     sin(psi) = b u sqrt(1 - 2u),   cos(psi) = -b du.
inline double psiAngle(double u, double du, double b) {
    const double sinPsi = b * u * std::sqrt(std::max(1.0 - 2.0 * u, 0.0));
    const double cosPsi = -b * du;
    return std::atan2(sinPsi, cosPsi);
}

struct TraceResult {
    bool captured = false;   // crossed the event horizon
    bool escaped = false;    // reached the escape radius going outward
    bool finite = true;      // no NaN / infinity produced
    double phi = 0.0;
    double u = 0.0;
    double du = 0.0;
    double rMin = 0.0;
    int steps = 0;
};

// Integrate a null geodesic in phi. Steps are uniform; the ray is reported as
// captured when it crosses r = 2r_g and as escaped once u < 1/escapeRadius while
// moving outward. Rays that exhaust maxSteps are near-critical orbits that wind
// around the photon sphere; they are reported as captured (they would require
// unbounded integration to escape).
inline TraceResult traceGeodesic(const RayInitialState& s, double dphi, int maxSteps,
                                 double escapeRadius) {
    TraceResult result;
    result.u = s.u0;
    result.du = s.du0;
    result.rMin = (s.u0 > 0.0) ? 1.0 / s.u0 : 0.0;
    if (!s.valid || dphi <= 0.0 || maxSteps <= 0 || escapeRadius <= 0.0) {
        result.finite = false;
        return result;
    }

    const double uEscape = 1.0 / escapeRadius;
    for (int i = 0; i < maxSteps; ++i) {
        double un = 0.0;
        double dun = 0.0;
        rk4Step(result.u, result.du, dphi, un, dun);
        result.phi += dphi;
        result.steps = i + 1;

        if (!std::isfinite(un) || !std::isfinite(dun)) {
            result.finite = false;
            return result;
        }
        if (un > 0.5) {  // inside r = 2 r_g
            result.captured = true;
            result.u = 0.5;
            result.du = dun;
            return result;
        }
        if (un <= 0.0) {
            // A near-radial outward ray has |du/dphi| -> infinity, so a uniform
            // step in phi overshoots r = infinity. The physical ray escapes, and
            // its asymptotic direction is purely radial.
            result.u = 0.0;
            result.du = dun;
            result.escaped = true;
            return result;
        }
        result.u = un;
        result.du = dun;
        result.rMin = std::min(result.rMin, 1.0 / un);

        if (un <= uEscape && dun < 0.0) {
            result.escaped = true;
            return result;
        }
    }

    result.captured = true;
    return result;
}

// Total deflection of a ray that arrives from a large radius with impact
// parameter b and escapes again. Returns NaN when the ray is captured.
inline double deflectionAngle(double b, double startRadius, double dphi, int maxSteps,
                              double escapeRadius) {
    if (b <= 0.0 || startRadius <= 0.0) return std::numeric_limits<double>::quiet_NaN();

    const double u0 = 1.0 / startRadius;
    const double sinPsi = b * u0 * std::sqrt(std::max(1.0 - 2.0 * u0, 0.0));
    if (sinPsi > 1.0) return std::numeric_limits<double>::quiet_NaN();

    RayInitialState s;
    s.u0 = u0;
    s.b = b;
    s.du0 = std::sqrt(std::max(0.0, 1.0 - sinPsi * sinPsi)) / b;  // incoming
    s.valid = true;

    const double psiStart = psiAngle(s.u0, s.du0, s.b);
    const TraceResult trace = traceGeodesic(s, dphi, maxSteps, escapeRadius);
    if (!trace.escaped) return std::numeric_limits<double>::quiet_NaN();

    // The direction angle in the orbital plane is phi + psi; a straight line
    // sweeps pi and bends by zero.
    double total = trace.phi + psiAngle(trace.u, trace.du, s.b) - psiStart;
    while (total > kPi) total -= 2.0 * kPi;
    while (total < -kPi) total += 2.0 * kPi;
    return total;
}

// ---------------------------------------------------------------------------
// Relativistic disk kinematics
// ---------------------------------------------------------------------------

// Angular velocity of a circular geodesic orbit, dphi/dt = sqrt(M/r^3) = r^-3/2
// in units where M = r_g = 1.
inline double keplerianOmega(double r) { return std::pow(std::max(r, 1e-6), -1.5); }

// Total frequency shift for emission from a circular Keplerian orbit:
//     g = nu_obs / nu_emit = sqrt(1 - 3M/r) / (1 - Omega * L_phi/E),
// where L_phi/E is the photon's (signed) angular momentum about the disk axis.
// The numerator is the gravitational + transverse Doppler redshift of the orbit;
// the denominator carries the longitudinal Doppler shift.
inline double circularOrbitShift(double r, double lPhiOverE) {
    const double ut = 1.0 / std::sqrt(std::max(1.0 - 3.0 / r, 1e-3));
    const double denom = std::max(1.0 - keplerianOmega(r) * lPhiOverE, 0.05);
    return 1.0 / (ut * denom);
}

// ---------------------------------------------------------------------------
// Small vector helper (kept raylib-free so the tests build without a GPU)
// ---------------------------------------------------------------------------
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalize(Vec3 a) {
    const double len = length(a);
    return (len > 1e-12) ? a * (1.0 / len) : Vec3{};
}

// Reference implementation of the shader's camera-ray trace: builds the orbital
// plane basis, integrates the Binet equation, and reports the asymptotic escape
// direction plus every crossing of the equatorial plane (y = 0).
struct CameraRayTrace {
    bool captured = false;
    bool escaped = false;
    bool finite = true;
    Vec3 outDirection;
    double minRadius = 0.0;
    double impactParameter = 0.0;
    int planeCrossings = 0;
    double firstCrossingRadius = 0.0;   // 0 when the ray never crosses y = 0
    double secondCrossingRadius = 0.0;
    int steps = 0;
};

inline CameraRayTrace traceCameraRay(Vec3 origin, Vec3 direction, double dphi, int maxSteps,
                                     double escapeRadius) {
    CameraRayTrace out;
    const double r0 = length(origin);
    if (r0 <= kHorizonRadius || length(direction) < 1e-9) {
        out.finite = false;
        return out;
    }
    direction = normalize(direction);

    const Vec3 e1 = origin * (1.0 / r0);                 // outward radial basis
    const double cosPsi = dot(direction, e1);
    const Vec3 tangential = direction - e1 * cosPsi;     // points along +phi
    const double sinPsi = length(tangential);

    if (sinPsi < 1e-9) {
        // Exactly radial: straight in or straight out.
        if (cosPsi < 0.0) {
            out.captured = true;
            out.minRadius = kHorizonRadius;
        } else {
            out.escaped = true;
            out.outDirection = direction;
        }
        return out;
    }

    const RayInitialState s = initialConditions(r0, cosPsi);
    if (!s.valid) {
        out.finite = false;
        return out;
    }
    out.impactParameter = s.b;

    const Vec3 e2 = normalize(tangential);
    const Vec3 planeNormal = normalize(cross(e1, e2));  // orbital angular momentum
    const double cosStep = std::cos(dphi);
    const double sinStep = std::sin(dphi);

    double u = s.u0;
    double du = s.du0;
    Vec3 radial = e1;
    Vec3 position = origin;
    out.minRadius = r0;

    const double uEscape = 1.0 / escapeRadius;
    for (int i = 0; i < maxSteps; ++i) {
        double un = 0.0;
        double dun = 0.0;
        rk4Step(u, du, dphi, un, dun);
        out.steps = i + 1;
        if (!std::isfinite(un) || !std::isfinite(dun)) {
            out.finite = false;
            return out;
        }
        if (un <= 0.0) {
            // Near-radial escape: the step in phi overshot infinity.
            out.outDirection = radial * (-s.b * dun);
            out.outDirection = normalize(out.outDirection);
            out.escaped = true;
            return out;
        }

        // Rotate the radial basis about the orbital normal (a fixed +phi
        // rotation), not about a fixed basis vector.
        const Vec3 nextRadial = radial * cosStep + cross(planeNormal, radial) * sinStep;
        const Vec3 nextPosition = nextRadial * (1.0 / un);

        // Equatorial-plane crossings (thin-disk geometry).
        if ((position.y > 0.0) != (nextPosition.y > 0.0)) {
            const double t = position.y / (position.y - nextPosition.y);
            const Vec3 hit = position + (nextPosition - position) * t;
            const double radius = length(hit);
            if (out.planeCrossings == 0) out.firstCrossingRadius = radius;
            else if (out.planeCrossings == 1) out.secondCrossingRadius = radius;
            ++out.planeCrossings;
        }

        u = un;
        du = dun;
        radial = nextRadial;
        position = nextPosition;
        out.minRadius = std::min(out.minRadius, 1.0 / u);

        if (u > 0.5) {
            out.captured = true;
            return out;
        }
        if (u <= uEscape && du < 0.0) {
            // Asymptotic direction: cos(psi) = -b du, sin(psi) = b u sqrt(1-2u).
            // The tangential basis is the current one, cross(planeNormal, radial),
            // not the initial e2: radial has rotated through the whole sweep by
            // now, and e2 is only the tangential direction at the camera (good
            // for a straight line, wrong by the swept angle for a bent one).
            const Vec3 tangentialNow = normalize(cross(planeNormal, radial));
            const Vec3 dir =
                radial * (-s.b * du) + tangentialNow * (s.b * u * std::sqrt(1.0 - 2.0 * u));
            out.outDirection = normalize(dir);
            out.escaped = true;
            return out;
        }
    }

    out.captured = true;  // near-critical orbit left winding
    return out;
}

}  // namespace bh
