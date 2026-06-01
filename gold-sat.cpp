// Gold-vector generator for the box-box SAT crux. Drives the reference
// `Manifold::collide` over a fixed set of box-pair configs and dumps the expected
// manifold (count, basis, per-contact feature key + local arms) as JSON. The TS
// oracle's collide.ts must reproduce these (tests/avbd/sat.test.ts).
//
// Build: g++ -std=c++17 -O2 source/solver.cpp source/rigid.cpp source/force.cpp source/manifold.cpp source/collide.cpp source/joint.cpp source/spring.cpp gold-sat.cpp -I source -o gold-sat
// Run:   ./gold-sat > ../../shallot/packages/shallot/tests/avbd/sat-gold-vectors.json

#include "solver.h"
#include <cstdio>
#include <cmath>
#include <vector>

struct Cfg {
    const char *name;
    float3 sizeA; float3 posA; quat quatA;
    float3 sizeB; float3 posB; quat quatB;
};

// rotation `deg` degrees about a unit axis
static quat aa(float3 axis, float deg) {
    float r = deg * 0.01745329251994329577f * 0.5f;
    float s = sinf(r);
    return quat{axis.x * s, axis.y * s, axis.z * s, cosf(r)};
}
static const quat ID = {0, 0, 0, 1};

static void writeFloat(float v) { printf("%.17g", v); }

int main() {
    std::vector<Cfg> cfgs = {
        // ── face contacts (the dominant case: box resting / pressed onto a face) ──
        {"face-y-overlap",   {1,1,1},{0,0,0},ID,           {1,1,1},{0,0.9f,0},ID},
        {"face-y-shallow",   {1,1,1},{0,0,0},ID,           {1,1,1},{0,0.99f,0},ID},
        {"face-x-overlap",   {1,1,1},{0,0,0},ID,           {1,1,1},{0.9f,0,0},ID},
        {"face-y-offset",    {1,1,1},{0,0,0},ID,           {1,1,1},{0.4f,0.9f,0.3f},ID},
        {"face-y-corner",    {1,1,1},{0,0,0},ID,           {1,1,1},{0.9f,0.9f,0.9f},ID},
        {"face-big-ground",  {10,1,10},{0,0,0},ID,         {1,1,1},{0,0.9f,0},ID},
        {"face-b-reference", {1,1,1},{0,0,0},ID,           {10,2,10},{0,1.4f,0},ID},
        {"face-asymmetric",  {2,1,3},{0,0,0},ID,           {1,2,1},{0.3f,1.4f,0},ID},
        // ── tilted face (incident face slightly off-axis) ──
        {"face-tilt-10deg",  {4,1,4},{0,0,0},ID,           {1,1,1},{0,0.9f,0},aa({0,0,1},10)},
        {"face-yaw-45deg",   {4,1,4},{0,0,0},ID,           {1,1,1},{0,0.9f,0},aa({0,1,0},45)},
        // a 45°-diamond (A) resting on a flat slab (B): none of A's axes align with the
        // vertical contact normal but B's +y face does → the FACE_B reference branch
        {"face-b-diag",      {1,1,1},{0,1.15f,0},aa({0,0,1},45), {10,1,10},{0,0,0},ID},
        // ── edge-edge (rotated boxes whose edges cross; edge axis wins the SAT) ──
        {"edge-x-y-cross",   {1,4,1},{0,0,0},aa({1,0,0},45), {1,4,1},{0,0,0.9f},aa({0,0,1},45)},
        // ── separated (no overlap → 0 contacts; basis undefined) ──
        {"sep-far",          {1,1,1},{0,0,0},ID,           {1,1,1},{5,0,0},ID},
        {"sep-gap",          {1,1,1},{0,0,0},ID,           {1,1,1},{1.05f,0,0},ID},
    };

    Solver solver;
    printf("{\"configs\":[");
    bool first = true;
    for (auto &c : cfgs) {
        solver.clear();
        Rigid *a = new Rigid(&solver, c.sizeA, 1.0f, 0.5f, c.posA);
        a->positionAng = c.quatA;
        Rigid *b = new Rigid(&solver, c.sizeB, 1.0f, 0.5f, c.posB);
        b->positionAng = c.quatB;

        Manifold::Contact contacts[8] = {0};
        float3x3 basis{};
        // Manifold::collide applies the Phase-4.8.1 reduction + sort + re-ordinal internally
        // (collide.cpp), so the emitted feature keys are already canonical.
        int n = Manifold::collide(a, b, contacts, basis);

        if (!first) printf(",");
        first = false;

        printf("{\"name\":\"%s\",", c.name);
        printf("\"a\":{\"size\":[%.17g,%.17g,%.17g],\"pos\":[%.17g,%.17g,%.17g],\"quat\":[%.17g,%.17g,%.17g,%.17g]},",
            c.sizeA.x, c.sizeA.y, c.sizeA.z, c.posA.x, c.posA.y, c.posA.z, c.quatA.x, c.quatA.y, c.quatA.z, c.quatA.w);
        printf("\"b\":{\"size\":[%.17g,%.17g,%.17g],\"pos\":[%.17g,%.17g,%.17g],\"quat\":[%.17g,%.17g,%.17g,%.17g]},",
            c.sizeB.x, c.sizeB.y, c.sizeB.z, c.posB.x, c.posB.y, c.posB.z, c.quatB.x, c.quatB.y, c.quatB.z, c.quatB.w);
        printf("\"numContacts\":%d,", n);

        printf("\"basis\":");
        if (n > 0) {
            printf("[");
            for (int r = 0; r < 3; r++)
                for (int col = 0; col < 3; col++) {
                    if (r || col) printf(",");
                    writeFloat(basis[r][col]);
                }
            printf("],");
        } else {
            printf("null,");
        }

        printf("\"contacts\":[");
        for (int i = 0; i < n; i++) {
            if (i) printf(",");
            Manifold::Contact &ct = contacts[i];
            printf("{\"feature\":%d,", ct.feature.key);
            printf("\"rA\":[%.17g,%.17g,%.17g],", ct.rA.x, ct.rA.y, ct.rA.z);
            printf("\"rB\":[%.17g,%.17g,%.17g]}", ct.rB.x, ct.rB.y, ct.rB.z);
        }
        printf("]}");
    }
    printf("]}\n");
    return 0;
}
