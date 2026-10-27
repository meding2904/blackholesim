// Kerr null geodesics in Boyer-Lindquist coordinates (M = 1, r_g = 1).
//
// The Schwarzschild reduction in geodesic.hpp cannot describe a spinning hole:
// with a != 0 the geodesics are not planar and the orbital plane of each ray no
// longer exists. What survives is the Carter separation, so each ray is described
// by its two exact constants
//
//     E = -p_t,  L = p_phi   (energy and axial angular momentum)
//     Q = p_theta^2 + cos^2(theta) [ L^2/sin^2(theta) - a^2 E^2 ]
//
// and the motion obeys the first-order equations
//
//     Sigma dr/dlambda  = +-sqrt(R(r))
//     Sigma dtheta/dlambda = +-sqrt(Theta(theta))
//     Sigma dphi/dlambda = [ 2 a r E + L (Sigma - 2r)/sin^2(theta) ] / Delta
//
// with
//
//     R(r) = [E (r^2 + a^2) - a L]^2 - Delta [ (L - a E)^2 + Q ]
//     Theta(theta) = Q + cos^2(theta) (a^2 E^2 - L^2/sin^2(theta))
//     Sigma = r^2 + a^2 cos^2(theta),   Delta = r^2 - 2r + a^2.
//
// The two signs are the direction of radial / polar motion; they flip only at a
// turning point (R = 0 or Theta = 0). Because E, L and Q are integrals of the
// motion, the trajectory cannot drift off the null cone, which is why this
// form is preferred over integrating the second-order Hamiltonian.
//
// Distances are gravitational radii (r_g = GM/c^2); the spin is the
// dimensionless a/M. The disk axis is world +Y, matching the renderer.
#pragma once

#include <algorithm>
#include <cmath>

#include "geodesic.hpp"

namespace bh {

// ---- Geometry --------------------------------------------------------------

inline double kerrHorizonRadius(double a) {
    const double a2 = a * a;
    return 1.0 + std::sqrt(std::max(0.0, 1.0 - a2));
}

// Innermost stable circular orbit (Bardeen, Press & Teukolsky 1972).
inline double kerrIscoRadius(double a, bool prograde) {
    const double aa = std::clamp(a, 0.0, 0.9999);
    const double z1 = 1.0 + std::cbrt(1.0 - aa * aa) * (std::cbrt(1.0 + aa) + std::cbrt(1.0 - aa));
    const double z2 = std::sqrt(3.0 * aa * aa + z1 * z1);
    const double root = std::sqrt(std::max(0.0, (3.0 - z1) * (3.0 + z1 + 2.0 * z2)));
    return prograde ? (3.0 + z2 - root) : (3.0 + z2 + root);
}

// Circular photon orbit.
inline double kerrPhotonOrbitRadius(double a, bool prograde) {
    const double aa = std::clamp(a, 0.0, 0.9999);
    const double sign = prograde ? -1.0 : 1.0;
    return 2.0 * (1.0 + std::cos((2.0 / 3.0) * std::acos(sign * aa)));
}

inline double kerrSigma(double r, double cosTheta, double a) {
    return r * r + a * a * cosTheta * cosTheta;
}

inline double kerrDelta(double r, double a) { return r * r - 2.0 * r + a * a; }

// ---- Camera ray -> conserved constants -------------------------------------
//
// The camera is a static observer outside the ergosphere. Its orthonormal frame
// is  e_t = (1/sqrt(-g_tt)) d_t,  e_r = sqrt(Delta/Sigma) d_r,
// e_theta = d_theta/sqrt(Sigma),  e_phi = beta (d_phi - (g_tphi/g_tt) d_t),
// with beta normalising e_phi. A photon whose local direction cosines are n is
// k = e_t + n_r e_r + n_theta e_theta + n_phi e_phi (any affine scale), which is
// exactly the Schwarzschild construction at a = 0.
struct KerrRay {
    bool valid = false;
    double r = 0.0;
    double theta = 0.0;
    double phi = 0.0;
    double E = 0.0;
    double L = 0.0;
    double Q = 0.0;
    double lOverE = 0.0;
    int signR = 1;
    int signTheta = 1;
};

inline KerrRay kerrRayFromCamera(Vec3 origin, Vec3 dir, double a) {
    KerrRay ray;
    const double r = length(origin);
    const double radialLen = length(dir);
    if (r <= kerrHorizonRadius(a) || radialLen < 1e-9) return ray;
    dir = normalize(dir);

    const double cosT = std::clamp(origin.y / r, -1.0, 1.0);
    const double sinT = std::sqrt(std::max(0.0, 1.0 - cosT * cosT));
    const double phi = std::atan2(origin.z, origin.x);

    const Vec3 rhat{sinT * std::cos(phi), cosT, sinT * std::sin(phi)};
    const Vec3 that{cosT * std::cos(phi), -sinT, cosT * std::sin(phi)};
    const Vec3 phat{-std::sin(phi), 0.0, std::cos(phi)};

    const double nr = dot(dir, rhat);
    const double nt = dot(dir, that);
    const double np = dot(dir, phat);

    const double sigma = kerrSigma(r, cosT, a);
    const double delta = kerrDelta(r, a);
    const double sin2 = std::max(sinT * sinT, 1e-8);
    const double gtt = -(1.0 - 2.0 * r / sigma);
    const double gtphi = -2.0 * a * r * sin2 / sigma;
    const double gphiphi =
        (r * r + a * a + 2.0 * a * a * r * sin2 / sigma) * sin2;
    if (gtt >= 0.0) return ray;  // inside the ergosphere: no static observer

    const double beta = 1.0 / std::sqrt(std::max(gphiphi - gtphi * gtphi / gtt, 1e-12));
    const double alpha = -(gtphi / gtt) * beta;

    const double kt = 1.0 / std::sqrt(-gtt) + np * alpha;
    const double kr = nr * std::sqrt(std::max(delta / sigma, 0.0));
    const double kth = nt / std::sqrt(sigma);
    const double kphi = np * beta;

    const double E = -(gtt * kt + gtphi * kphi);
    const double L = gtphi * kt + gphiphi * kphi;
    const double pTheta = sigma * kth;

    ray.r = r;
    ray.theta = std::acos(cosT);
    ray.phi = phi;
    ray.E = E;
    ray.L = L;
    ray.Q = pTheta * pTheta + cosT * cosT * (L * L / sin2 - a * a * E * E);
    ray.lOverE = (std::fabs(E) > 1e-12) ? L / E : 0.0;
    ray.signR = (kr >= 0.0) ? 1 : -1;
    ray.signTheta = (kth >= 0.0) ? 1 : -1;
    ray.valid = std::fabs(E) > 1e-12;
    return ray;
}

// Frequency shift of emission from a circular equatorial orbit, as seen by a
// static observer at infinity:
//     g = 1 / (u^t (1 - Omega L/E)),   Omega = 1/(r^{3/2} + a)   (prograde)
//     u^t = (r^{3/2} + a) / (r^{3/4} sqrt(r^{3/2} - 3 r^{1/2} + 2a))
inline double kerrOrbitShift(double r, double a, double lOverE, bool prograde) {
    const double sign = prograde ? 1.0 : -1.0;
    const double sqrtR = std::sqrt(std::max(r, 1e-6));
    const double omega = sign / (r * sqrtR + sign * a);
    const double ut = (r * sqrtR + sign * a) /
                      (std::pow(std::max(r, 1e-6), 0.75) *
                       std::sqrt(std::max(r * sqrtR - 3.0 * sqrtR + 2.0 * sign * a, 1e-4)));
    const double denom = std::max(1.0 - omega * lOverE, 0.05);
    return 1.0 / (ut * denom);
}

// ---- Integration -----------------------------------------------------------

struct KerrDerivatives {
    double dr = 0.0;
    double dtheta = 0.0;
    double dphi = 0.0;
    double R = 0.0;
    double Theta = 0.0;
    bool finite = true;
};

// R(r) >= 0 is the centrifugal barrier of the radial motion; it depends on r
// alone. Theta(theta) plays the same role for the polar motion.
inline double kerrRadialPotential(double E, double L, double Q, double a, double r) {
    const double tr = E * (r * r + a * a) - a * L;
    const double delta = r * r - 2.0 * r + a * a;
    return tr * tr - delta * ((L - a * E) * (L - a * E) + Q);
}

// dR/dr, used for the Newton refinement of a radial turning radius.
inline double kerrRadialSlope(double E, double L, double Q, double a, double r) {
    const double tr = E * (r * r + a * a) - a * L;
    const double carter = (L - a * E) * (L - a * E) + Q;
    return 4.0 * E * r * tr - (2.0 * r - 2.0) * carter;
}

inline double kerrPolarPotential(double E, double L, double Q, double a, double theta) {
    const double cosT = std::cos(theta);
    // sin^2(theta) must NOT be clamped to a visible-scale floor here: the
    // L^2/sin^2(theta) term is the centrifugal barrier that turns rays around
    // near the spin axis. A floor of 1e-6 caps it at L^2 * 1e6 and silently
    // deletes the barrier for |L| < sqrt(Q) * 1e-3, letting those rays cross
    // straight through the pole instead of bouncing -- which scrambles their
    // escape direction (the torn seam along the projected spin axis). The tiny
    // guard only prevents a 0/0 for the exactly-polar L = 0 ray, where the L^2
    // numerator is zero anyway.
    const double sin2 = std::max(std::sin(theta) * std::sin(theta), 1e-30);
    return Q + cosT * cosT * (a * a * E * E - L * L / sin2);
}

inline KerrDerivatives kerrDerivatives(double a, double E, double L, double Q, double r,
                                       double theta, int signR, int signTheta) {
    KerrDerivatives d;
    const double cosT = std::cos(theta);
    const double sinT = std::sin(theta);
    // Same as kerrPolarPotential: no visible-scale clamp on sin^2(theta), or
    // the azimuthal sweep L(Sigma-2r)/sin^2(theta) through a polar passage is
    // undercounted and the escape direction comes out wrong.
    const double sin2 = std::max(sinT * sinT, 1e-30);
    const double sigma = r * r + a * a * cosT * cosT;
    const double delta = r * r - 2.0 * r + a * a;
    if (sigma < 1e-9) {
        d.finite = false;
        return d;
    }

    d.R = kerrRadialPotential(E, L, Q, a, r);
    d.Theta = kerrPolarPotential(E, L, Q, a, theta);
    if (!std::isfinite(d.R) || !std::isfinite(d.Theta)) {
        d.finite = false;
        return d;
    }

    d.dr = static_cast<double>(signR) * std::sqrt(std::max(d.R, 0.0)) / sigma;
    d.dtheta = static_cast<double>(signTheta) * std::sqrt(std::max(d.Theta, 0.0)) / sigma;
    d.dphi = (2.0 * a * r * E + L * (sigma - 2.0 * r) / sin2) / (sigma * delta);
    if (!std::isfinite(d.dphi)) {
        d.finite = false;
        d.dphi = 0.0;
    }
    return d;
}

struct KerrTrace {
    bool captured = false;
    bool escaped = false;
    bool finite = true;
    Vec3 outDirection{};
    double minRadius = 0.0;
    double lOverE = 0.0;
    double firstCrossingRadius = 0.0;
    double secondCrossingRadius = 0.0;
    int crossings = 0;
    int steps = 0;
};

// Polar turning angle thetaT: a ray with L != 0 cannot pass |theta| < thetaT
// (the centrifugal barrier). thetaT = 0 exactly for L = 0, whose geodesic
// passes through the axis instead of bouncing. Computed as the small positive
// root of Theta(theta) = 0 in sin^2(theta), in a cancellation-free form:
//     sin^2(thetaT) = 2 L^2 / (a^2 E^2 + Q + L^2 + sqrt(B^2 + 4 a^2 E^2 Q))
// with B = a^2 E^2 - Q - L^2. Taking acos(uMax) instead loses most of the
// significant digits when thetaT is small.
inline double kerrPolarThetaT(double E, double L, double Q, double a) {
    const double A = a * a * E * E;
    const double B = A - Q - L * L;
    const double disc = std::sqrt(std::max(B * B + 4.0 * A * Q, 0.0));
    const double s = 2.0 * L * L / std::max(A + Q + L * L + disc, 1e-300);
    return std::asin(std::sqrt(std::min(std::max(s, 0.0), 1.0)));
}

// Step control. Two rulers, whichever is stricter:
//   * geometric, dlambda = hRadial * r: strides through the nearly straight far
//     field and keeps a finite step at a turning point (where a pure rate-based
//     ruler would blow up);
//   * angular, dlambda = hAngle / max(|dphi|, |dtheta|): the bending near the
//     hole is what the image is made of, and a purely geometric ruler
//     under-resolves it badly (~4x too little deflection at h = 0.035).
// Largest |cos(theta)| the ray reaches: the positive root of Theta_u = 0. It is
// a ray constant, so the step ruler and the polar mirroring both use it.
inline double kerrUMax(double E, double L, double Q, double a) {
    const double A = a * a * E * E;
    const double B = A - Q - L * L;
    if (A < 1e-12) return std::sqrt(std::max(Q / std::max(Q + L * L, 1e-12), 0.0));
    const double disc = std::sqrt(std::max(B * B + 4.0 * A * Q, 0.0));
    return std::sqrt(std::max((B + disc) / (2.0 * A), 0.0));
}

// Step control. Three rulers, whichever is strictest:
//   * geometric, dlambda = hRadial * r: strides through the nearly straight far
//     field and keeps a finite step at a radial turning point;
//   * angular, dlambda = hAngle / max(|dphi|, |dtheta|): the bending near the
//     hole is what the image is made of, and a purely geometric ruler
//     under-resolves it badly (~4x too little deflection at h = 0.035);
//   * polar, staying short of |cos(theta)| = uMax, so the mirror below is a
//     small correction instead of a wrap through the equatorial plane.
inline double kerrStepLength(const KerrDerivatives& d, double r, double theta, double thetaT,
                             double hRadial, double hAngle) {
    const double rate = std::max(std::fabs(d.dphi), std::fabs(d.dtheta));
    double dl = (rate > 1e-12) ? std::min(hRadial * std::max(r, 1.0), hAngle / rate)
                               : hRadial * std::max(r, 1.0);
    // Pole-proximity ruler. The dphi rate carries an L/sin^2(theta) term that
    // spikes by orders of magnitude as a near-axis ray approaches its polar
    // turning point; that spike IS the ~pi sweep of the ray, so a step that
    // jumps from outside the spike into it integrates it as garbage (one RK4
    // stage samples the spike while the others do not). Cap the step's theta
    // travel to a quarter of the distance to the nearest pole so the rate stays
    // nearly constant across a step; inside the spike the angular ruler above
    // then takes over and resolves it. Only needed where the centrifugal
    // barrier exists (thetaT > 0, i.e. L != 0); the L = 0 geodesic legitimately
    // crosses the axis, has no spike, and must not stall.
    if (thetaT > 1e-12 && std::fabs(d.dtheta) > 1e-12) {
        const double poleDist = std::min(theta, kPi - theta);
        dl = std::min(dl, 0.25 * poleDist / std::fabs(d.dtheta));
    }
    return dl;
}

inline Vec3 kerrEscapeDirection(double theta, double phi, const KerrDerivatives& d, double r) {
    const double cosT = std::cos(theta);
    const double sinT = std::sin(theta);
    const double dr = d.dr;
    const double dth = r * d.dtheta;
    const double dph = r * sinT * d.dphi;
    const double norm = std::sqrt(dr * dr + dth * dth + dph * dph);
    if (!(norm > 1e-12)) return Vec3{};
    const Vec3 rhat{sinT * std::cos(phi), cosT, sinT * std::sin(phi)};
    const Vec3 that{cosT * std::cos(phi), -sinT, cosT * std::sin(phi)};
    const Vec3 phat{-std::sin(phi), 0.0, std::cos(phi)};
    const Vec3 out = rhat * (dr / norm) + that * (dth / norm) + phat * (dph / norm);
    return normalize(out);
}

// Cubic Hermite root of cos(theta) inside a step. The linear root is off by
// O(dl^2) with dl of a few r_g, which walks the hit radius by a visible fraction
// of r_g from pixel to pixel. signR / signTheta are the *live* branch signs of
// the step: they flip at turning points, so the end-point derivative has to be
// rebuilt with them, not with the camera's initial signs (hardcoding the latter
// put the interpolated hit radius up to ~0.8 r_g off after a radial turn).
inline double kerrCrossingRadius(double a, const KerrRay& ray, double r0, double theta0,
                                 double cos0, double r1, double theta1, double cos1,
                                 const KerrDerivatives& d0, double dl, int signR,
                                 int signTheta) {
    const KerrDerivatives d1 =
        kerrDerivatives(a, ray.E, ray.L, ray.Q, r1, theta1, signR, signTheta);
    const double mu0 = -d0.dtheta * std::sin(theta0) * dl;   // d(cos)/dlambda * dl
    const double mu1 = -d1.dtheta * std::sin(theta1) * dl;
    const double mr0 = d0.dr * dl;
    const double mr1 = d1.dr * dl;
    const auto h00 = [](double s) { return 2 * s * s * s - 3 * s * s + 1; };
    const auto h10 = [](double s) { return s * s * s - 2 * s * s + s; };
    const auto h01 = [](double s) { return -2 * s * s * s + 3 * s * s; };
    const auto h11 = [](double s) { return s * s * s - s * s; };
    const auto dh00 = [](double s) { return 6 * s * s - 6 * s; };
    const auto dh10 = [](double s) { return 3 * s * s - 4 * s + 1; };
    const auto dh01 = [](double s) { return -6 * s * s + 6 * s; };
    const auto dh11 = [](double s) { return 3 * s * s - 2 * s; };

    double s = std::clamp(cos0 / (cos0 - cos1), 0.0, 1.0);
    for (int it = 0; it < 3; ++it) {
        const double f = h00(s) * cos0 + h10(s) * mu0 + h01(s) * cos1 + h11(s) * mu1;
        const double df = dh00(s) * cos0 + dh10(s) * mu0 + dh01(s) * cos1 + dh11(s) * mu1;
        if (std::fabs(df) < 1e-12) break;
        s = std::clamp(s - f / df, 0.0, 1.0);
    }
    return h00(s) * r0 + h10(s) * mr0 + h01(s) * r1 + h11(s) * mr1;
}

// March optimizations the GPU shader applies, mirrored here so the tests can
// pin them: both only change *when* a departing or doomed ray stops
// integrating, never its verdict or escape direction.
struct KerrMarchOptions {
    bool farRuler = true;    // grow the step beyond kFarRadius for outward rays
    bool captureCut = true;  // stop a plunging ray inside the prograde photon orbit
};

// Mirrors of FAR_RADIUS / FAR_GROWTH in shaders/blackhole.fs.
constexpr double kFarRadius = 60.0;
constexpr double kFarGrowth = 6.0;

inline KerrTrace traceKerrRay(Vec3 origin, Vec3 direction, double a, double stepSize, int maxSteps,
                              double escapeRadius, KerrMarchOptions opts = {}) {
    KerrTrace out;
    const KerrRay ray = kerrRayFromCamera(origin, direction, a);
    if (!ray.valid) {
        out.finite = false;
        return out;
    }
    out.lOverE = ray.lOverE;

    const double rHorizon = kerrHorizonRadius(a);
    double r = ray.r;
    double theta = ray.theta;
    double phi = ray.phi;
    int signR = ray.signR;
    int signTheta = ray.signTheta;
    out.minRadius = r;

    const double uMax = kerrUMax(ray.E, ray.L, ray.Q, a);
    const double thetaT = kerrPolarThetaT(ray.E, ray.L, ray.Q, a);
    const double rPhotonPro = kerrPhotonOrbitRadius(a, true);
    for (int i = 0; i < maxSteps; ++i) {
        const KerrDerivatives d =
            kerrDerivatives(a, ray.E, ray.L, ray.Q, r, theta, signR, signTheta);
        if (!d.finite) {
            out.finite = false;
            return out;
        }
        const double cosNow = std::cos(theta);
        // Far-field ruler, same gate as the shader: moving outward beyond the
        // far radius with no turning point between here and 2r (R(2r) > 0), the
        // geometric cap grows by kFarGrowth. The angular and pole rulers keep
        // their authority inside kerrStepLength, so the growth only ever
        // lengthens a step the bending would have allowed anyway.
        double hRadial = stepSize;
        if (opts.farRuler && signR > 0 && r > kFarRadius &&
            kerrRadialPotential(ray.E, ray.L, ray.Q, a, 2.0 * r) > 0.0) {
            hRadial = std::max(hRadial, kFarGrowth * stepSize);
        }
        const double dl = kerrStepLength(d, r, theta, thetaT, hRadial, stepSize * 0.6);
        if (!(dl > 0.0) || !std::isfinite(dl)) {
            out.finite = false;
            return out;
        }

        // RK4 on (r, theta, phi). The sign of the square-root branch is part of
        // the state, so it is held fixed inside a step and only flipped when the
        // step turns out to cross a turning point.
        const auto deriv = [&](double rr, double tt, double* k) {
            const KerrDerivatives kd =
                kerrDerivatives(a, ray.E, ray.L, ray.Q, rr, tt, signR, signTheta);
            k[0] = kd.dr;
            k[1] = kd.dtheta;
            k[2] = kd.dphi;
        };
        double k1[3], k2[3], k3[3], k4[3];
        deriv(r, theta, k1);
        deriv(r + 0.5 * dl * k1[0], theta + 0.5 * dl * k1[1], k2);
        deriv(r + 0.5 * dl * k2[0], theta + 0.5 * dl * k2[1], k3);
        deriv(r + dl * k3[0], theta + dl * k3[1], k4);

        double rNew = r + (dl / 6.0) * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]);
        double thetaNew = theta + (dl / 6.0) * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]);
        double phiNew = phi + (dl / 6.0) * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]);

        if (!std::isfinite(rNew) || !std::isfinite(thetaNew) || !std::isfinite(phiNew)) {
            out.finite = false;
            return out;
        }

        // Disk crossings are tested on the raw step end, before any mirror: a
        // grazing ray that overshoots the lower polar turning point would
        // otherwise be reflected back to the positive side and lose its hit.
        const double cosRaw = std::cos(thetaNew);
        if ((cosNow > 0.0) != (cosRaw > 0.0)) {
            const double rHit = kerrCrossingRadius(a, ray, r, theta, cosNow, rNew, thetaNew,
                                                  cosRaw, d, dl, signR, signTheta);
            if (out.crossings == 0) out.firstCrossingRadius = rHit;
            else if (out.crossings == 1) out.secondCrossingRadius = rHit;
            ++out.crossings;
        }

        // Turning points. R depends only on r and Theta only on theta, so each
        // can be tested in isolation. A negative value means the step crossed
        // the turning point: mirror the coordinate about it and flip the branch
        // sign, which is second-order accurate and never stalls.
        if (kerrRadialPotential(ray.E, ray.L, ray.Q, a, rNew) < 0.0) {
            // Mirror about the actual turning radius (Newton on R(r) = 0) rather
            // than about the step start: the latter is only second order and at
            // dlambda = h*r with h = 0.035 it costs ~0.01 rad of deflection.
            double rt = 0.5 * (r + rNew);
            const double slope = kerrRadialSlope(ray.E, ray.L, ray.Q, a, rt);
            if (std::fabs(slope) > 1e-12) {
                rt = std::clamp(rt - kerrRadialPotential(ray.E, ray.L, ray.Q, a, rt) / slope,
                                std::min(r, rNew), std::max(r, rNew));
            }
            rNew = 2.0 * rt - rNew;
            signR = -signR;
        }
        if (kerrPolarPotential(ray.E, ray.L, ray.Q, a, thetaNew) < 0.0) {
            // Mirror about the turning point itself (|cos| = uMax), not about
            // the step start, which would land on the wrong side of the plane.
            // The hemisphere is taken from the step *start* (cosNow), the side
            // the ray is actually approaching from; the raw step end can already
            // be past the pole for a grazing step and would pick the wrong wall.
            const double mirrored = (cosNow >= 0.0 ? 2.0 * uMax : -2.0 * uMax) - cosRaw;
            thetaNew = std::acos(std::clamp(mirrored, -1.0, 1.0));
            signTheta = -signTheta;
        }
        // Axis crossing: theta is a chart angle on [0, pi]. A ray with L = 0 has
        // no centrifugal barrier and passes through the spin axis, where theta
        // would step out of range. Relabel to the same physical point on the
        // opposite meridian: theta -> -theta (north) or 2 pi - theta (south),
        // phi -> phi + pi, dtheta -> -dtheta. The relabel is a chart change, so
        // the geodesic is unchanged; skipping it used to make the escape
        // azimuth jump by pi across the projected axis.
        if (thetaNew < 0.0 || thetaNew > kPi) {
            thetaNew = (thetaNew < 0.0) ? -thetaNew : 2.0 * kPi - thetaNew;
            phiNew += kPi;
            signTheta = -signTheta;
        }
        r = rNew;
        theta = thetaNew;
        phi = phiNew;
        out.steps = i + 1;
        out.minRadius = std::min(out.minRadius, r);

        // Capture theorem, same as the shader: inside the prograde photon orbit
        // -- the smallest radius any photon can turn at -- and moving inward, the
        // perigee lies below every turning point, so the ray is provably
        // captured here, not eventually. Without the cut a plunging ray keeps
        // integrating to its step budget before being declared captured.
        if (opts.captureCut && signR < 0 && r <= rPhotonPro) {
            out.captured = true;
            return out;
        }

        // Horizon: Delta < 0 is exactly the region between the two horizons.
        if (r <= rHorizon || kerrDelta(r, a) <= 0.0) {
            out.captured = true;
            out.minRadius = std::min(out.minRadius, rHorizon);
            return out;
        }

        const KerrDerivatives dNow =
            kerrDerivatives(a, ray.E, ray.L, ray.Q, r, theta, signR, signTheta);
        if (r >= escapeRadius && dNow.dr > 0.0) {
            out.outDirection = kerrEscapeDirection(theta, phi, dNow, r);
            out.escaped = true;
            return out;
        }
    }

    out.captured = true;  // near-critical orbit that keeps winding
    return out;
}

// Null-condition residual of the reconstructed 4-momentum: a cheap global
// accuracy probe for the tests (0 for an exact geodesic).
inline double kerrNullResidual(double a, double E, double L, double Q, double r, double theta,
                               int signR, int signTheta) {
    const KerrDerivatives d = kerrDerivatives(a, E, L, Q, r, theta, signR, signTheta);
    if (!d.finite) return std::numeric_limits<double>::quiet_NaN();
    const double cosT = std::cos(theta);
    const double sinT = std::sin(theta);
    const double sigma = r * r + a * a * cosT * cosT;
    const double delta = r * r - 2.0 * r + a * a;
    const double sin2 = std::max(sinT * sinT, 1e-30);
    // Covariant components from the velocities:
    //     Sigma p_r = Sigma^2/Delta dr,  p_theta = Sigma dtheta,  p_phi = L,  p_t = -E.
    const double pr = (sigma / std::max(delta, 1e-9)) * d.dr;
    const double pTheta = sigma * d.dtheta;
    const double pp = L;
    const double grr = delta / sigma;
    const double gthth = 1.0 / sigma;

    // g^{ab} p_a p_b, which is identically zero for a null geodesic.
    const double ginvTT = -(r * r + a * a + 2.0 * a * a * r * sin2 / sigma) / delta;
    const double ginvTPhi = -2.0 * a * r / (sigma * delta);
    const double ginvPhiPhi = (delta - a * a * sin2) / (sigma * delta * sin2);
    return ginvTT * E * E + 2.0 * ginvTPhi * (-E) * pp + ginvPhiPhi * pp * pp +
           pr * pr * grr + pTheta * pTheta * gthth;
}

}  // namespace bh
