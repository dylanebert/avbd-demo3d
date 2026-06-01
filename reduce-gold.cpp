// Gold-vector generator for the contact-manifold reduction (Phase 4.8.1). `PruneContactPoints`
// below is a verbatim port of Jolt's reduction (Jolt/Physics/Collision/ManifoldBetweenTwoFaces.cpp,
// Jorrit Rouwe; the Dirk Gregorius GDC-2015 "Robust Contact Creation" recipe) — the only change is
// JPH's Vec3 / StaticArray swapped for the minimal `f3` below. It is the INDEPENDENT reference the
// shallot oracle's `reduceManifold` (tests/avbd/collide.ts) must reproduce (tests/avbd/reduce.test.ts);
// keeping it separate from our own ports is what makes that test a real cross-check, not a tautology.
//
// Emits each input point cloud (xA/xB anchors, penetration axis, body-A center) + the indices Jolt
// keeps, as JSON. Double precision, to match the f64 oracle (the GPU f32 port is gated against the
// oracle, not this); the input clouds are asymmetric so the argmax selections never sit on a tie.
//
// Build/run: handled by tests/avbd/gen-reduce-gold.ts (g++ -std=c++17).

#include <cstdio>
#include <cmath>
#include <cfloat>
#include <vector>

struct f3 {
    double x, y, z;
};
static f3 operator-(f3 a, f3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static f3 operator*(f3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
static double dot(f3 a, f3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static f3 cross(f3 a, f3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
static double len2(f3 a) { return dot(a, a); }

// Verbatim Jolt PruneContactPoints. Inputs are relative to the body-A center (the caller subtracts
// it, matching Jolt, where points arrive COM-relative). Returns the kept indices in polygon order
// [point1, point3, point2, point4]; point3/point4 are dropped (-1) when every candidate lies on one
// side of the point1→point2 line. Only called with > 4 points.
static std::vector<int> pruneContactPoints(f3 axis, const std::vector<f3> &on1, const std::vector<f3> &on2) {
    constexpr double cMinDistanceSq = 1.0e-6;
    int n = (int)on1.size();

    std::vector<f3> projected(n);
    std::vector<double> penetration_depth_sq(n);
    for (int i = 0; i < n; i++) {
        f3 v1 = on1[i];
        projected[i] = v1 - axis * dot(v1, axis);
        f3 v2 = on2[i];
        penetration_depth_sq[i] = std::fmax(cMinDistanceSq, len2(v2 - v1));
    }

    int point1 = 0;
    double val = std::fmax(cMinDistanceSq, len2(projected[0])) * penetration_depth_sq[0];
    for (int i = 0; i < n; i++) {
        double v = std::fmax(cMinDistanceSq, len2(projected[i])) * penetration_depth_sq[i];
        if (v > val) { val = v; point1 = i; }
    }
    f3 point1v = projected[point1];

    int point2 = -1;
    val = -DBL_MAX;
    for (int i = 0; i < n; i++)
        if (i != point1) {
            double v = std::fmax(cMinDistanceSq, len2(projected[i] - point1v)) * penetration_depth_sq[i];
            if (v > val) { val = v; point2 = i; }
        }
    f3 point2v = projected[point2];

    int point3 = -1, point4 = -1;
    double min_val = 0.0, max_val = 0.0;
    f3 perp = cross(point2v - point1v, axis);
    for (int i = 0; i < n; i++)
        if (i != point1 && i != point2) {
            double v = dot(perp, projected[i] - point1v);
            if (v < min_val) { min_val = v; point3 = i; }
            else if (v > max_val) { max_val = v; point4 = i; }
        }

    std::vector<int> keep;
    keep.push_back(point1);
    if (point3 != -1) keep.push_back(point3);
    keep.push_back(point2);
    if (point4 != -1) keep.push_back(point4);
    return keep;
}

struct Case {
    const char *name;
    f3 axis;
    f3 com;
    std::vector<f3> xA;
    std::vector<f3> xB; // xB = xA - axis * depth
};

// build a cloud: N points around the contact plane (⊥ axis), each pushed `depth[i]` into B along axis
static Case cloud(const char *name, f3 axis, f3 com, std::vector<f3> pts, std::vector<double> depth) {
    Case c{name, axis, com, pts, {}};
    for (size_t i = 0; i < pts.size(); i++) c.xB.push_back(pts[i] - axis * depth[i]);
    return c;
}

int main() {
    f3 up{0, 1, 0};
    // a slightly tilted axis to exercise the projection
    double s = std::sin(0.15), co = std::cos(0.15);
    f3 tilt{s, co, 0};

    std::vector<Case> cases = {
        // an octagon-ish ring with one clearly deepest point + asymmetric radii (no argmax ties)
        cloud("ring-deep", up, {0, 0, 0},
              {{1.0, 0, 0.1}, {0.7, 0, 0.75}, {0.05, 0, 1.05}, {-0.7, 0, 0.7}, {-1.05, 0, 0.0}, {-0.7, 0, -0.72}, {0.0, 0, -1.0}, {0.74, 0, -0.7}},
              {0.02, 0.03, 0.09, 0.025, 0.018, 0.022, 0.04, 0.05}),
        // a hexagon, uniform-ish depth, irregular radii
        cloud("hex-irreg", up, {0.2, 0, -0.1},
              {{1.2, 0, 0}, {0.55, 0, 0.95}, {-0.6, 0, 0.9}, {-1.15, 0, 0.05}, {-0.5, 0, -0.88}, {0.62, 0, -0.92}},
              {0.03, 0.05, 0.04, 0.06, 0.035, 0.045}),
        // a pentagon offset from the com
        cloud("penta-offset", up, {-0.4, 0, 0.3},
              {{0.9, 0, 0.2}, {0.1, 0, 0.95}, {-0.85, 0, 0.55}, {-0.7, 0, -0.6}, {0.5, 0, -0.85}},
              {0.04, 0.02, 0.07, 0.03, 0.05}),
        // tilted axis, 7 points
        cloud("tilt-seven", tilt, {0, 0, 0},
              {{1.0, 0.1, 0.0}, {0.6, 0.05, 0.7}, {-0.1, 0.0, 1.0}, {-0.75, -0.05, 0.6}, {-1.0, -0.1, -0.1}, {-0.4, -0.05, -0.85}, {0.7, 0.05, -0.65}},
              {0.02, 0.06, 0.03, 0.08, 0.04, 0.05, 0.03}),
        // a degenerate near-collinear cloud (all on one side of the p1->p2 line -> 3 kept)
        cloud("collinear-ish", up, {0, 0, 0},
              {{-1.0, 0, 0.02}, {-0.5, 0, 0.0}, {0.0, 0, 0.01}, {0.5, 0, -0.01}, {1.0, 0, 0.0}},
              {0.05, 0.03, 0.08, 0.02, 0.06}),
        // a dense 8-point asymmetric blob
        cloud("blob-eight", up, {0.1, 0, 0.05},
              {{0.9, 0, 0.3}, {0.45, 0, 0.8}, {-0.2, 0, 1.0}, {-0.8, 0, 0.5}, {-0.95, 0, -0.2}, {-0.4, 0, -0.8}, {0.3, 0, -0.95}, {0.85, 0, -0.45}},
              {0.033, 0.071, 0.052, 0.019, 0.088, 0.041, 0.06, 0.027}),
    };

    printf("{\"cases\":[");
    bool first = true;
    for (auto &c : cases) {
        std::vector<f3> on1, on2;
        for (size_t i = 0; i < c.xA.size(); i++) {
            on1.push_back(c.xA[i] - c.com);
            on2.push_back(c.xB[i] - c.com);
        }
        std::vector<int> keep = pruneContactPoints(c.axis, on1, on2);

        if (!first) printf(",");
        first = false;
        printf("{\"name\":\"%s\",", c.name);
        printf("\"axis\":[%.17g,%.17g,%.17g],", c.axis.x, c.axis.y, c.axis.z);
        printf("\"com\":[%.17g,%.17g,%.17g],", c.com.x, c.com.y, c.com.z);
        printf("\"points\":[");
        for (size_t i = 0; i < c.xA.size(); i++) {
            if (i) printf(",");
            printf("{\"xA\":[%.17g,%.17g,%.17g],\"xB\":[%.17g,%.17g,%.17g]}",
                   c.xA[i].x, c.xA[i].y, c.xA[i].z, c.xB[i].x, c.xB[i].y, c.xB[i].z);
        }
        printf("],\"keep\":[");
        for (size_t i = 0; i < keep.size(); i++) {
            if (i) printf(",");
            printf("%d", keep[i]);
        }
        printf("]}");
    }
    printf("]}\n");
    return 0;
}
