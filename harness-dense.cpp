// Dense fixture generator for AVBD parity testing.
// Outputs Y-up JSON fixtures (converted from native Z-up).
//
// Build: g++ -std=c++17 -O2 source/solver.cpp source/rigid.cpp source/force.cpp source/manifold.cpp source/collide.cpp source/joint.cpp source/spring.cpp harness-dense.cpp -I source -o harness-dense
// Run:   ./harness-dense 600 ../../shallot/packages/shallot/tests/fixtures/avbd/

#include "solver.h"
#include "scenes.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

struct SceneEntry { const char *name; void (*fn)(Solver *); };

// Harness-only parity scenes — the rotation / tipping / settling-friction dynamics the demo's own box
// scenes never exercise (ground/stack/stack-ratio/pyramid/two-boxes are all axis-aligned, ~zero angular
// velocity). corner-rest + leaning are bit-identical to the TS corpus (tests/avbd/corpus.ts cornerRest()
// / leaning()), so the gold dump backs those corpus topologies. friction-settle is a bounded slide-to-rest:
// the demo's dynamic/static-friction boxes slide off the 100-wide ground and free-fall (never settle), so
// they can't verify friction *stops* a box — these decelerate to rest within the ground, lower μ farther.

// a unit box tilted 45° about x then z, dropped — lands on a vertex, tips vertex->edge->face to rest flat.
static void sceneCornerRest(Solver *solver)
{
    solver->clear();
    new Rigid(solver, {40, 1, 40}, 0.0f, 0.6f, {0, 0, 0});
    float a = rad(45.0f);
    quat qx = {sinf(a * 0.5f), 0, 0, cosf(a * 0.5f)};
    quat qz = {0, 0, sinf(a * 0.5f), cosf(a * 0.5f)};
    Rigid *b = new Rigid(solver, {1, 1, 1}, 1.0f, 0.6f, {0, 2.5f, 0});
    b->positionAng = qz * qx;
}

// a 5-box stack each offset 0.4 in +x — the cumulative lean puts the upper COM past the base, so it
// topples, scatters, and settles. The chaotic stress: a topple must dissipate, never inject energy.
static void sceneLeaning(Solver *solver)
{
    solver->clear();
    new Rigid(solver, {40, 1, 40}, 0.0f, 0.5f, {0, 0, 0});
    for (int i = 0; i < 5; i++)
        new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {i * 0.4f, 1.0f + i, 0});
}

// four boxes resting on the ground, launched in +x at 5 m/s; Coulomb friction brings each to rest inside
// the ground bounds. Lower μ slides farther (d = v0^2/(2 μ g)) — the friction signature. Each in its own
// z-lane so they slide independently (no box-box contact muddying the per-μ stop distance). μ < 1 so a box
// slides rather than tipping (the sliding normal-force shift stays inside the base).
static void sceneFrictionSettle(Solver *solver)
{
    solver->clear();
    new Rigid(solver, {40, 1, 40}, 0.0f, 0.5f, {0, 0, 0});
    const float v0 = 5.0f;
    for (int i = 0; i < 4; i++)
    {
        float mu = 0.2f + i * 0.2f; // 0.2, 0.4, 0.6, 0.8
        new Rigid(solver, {1, 1, 1}, 1.0f, mu, {-6.0f, 1.0f, -3.0f + i * 2.0f}, {v0, 0, 0});
    }
}

static SceneEntry denseScenes[] = {
    {"ground", sceneGround},
    {"stack", sceneStack},
    {"stack-ratio", sceneStackRatio},
    {"pyramid", scenePyramid},
    {"rope", sceneRope},
    {"heavy-rope", sceneHeavyRope},
    {"spring", sceneSpring},
    {"spring-ratio", sceneSpringsRatio},
    {"bridge", sceneBridge},
    {"breakable", sceneBreakable},
    {"dynamic-friction", sceneDynamicFriction},
    {"static-friction", sceneStaticFriction},
    {"soft-body", sceneSoftBody},
    {"soft-joint", sceneSoftJoint},
    {"two-boxes", sceneTwoBoxes},
    {"rigid-joint", sceneRigidJoint},
    {"soft-joint-free", sceneSoftJointFree},
    {"bridge-mini", sceneBridgeMini},
    {"corner-rest", sceneCornerRest},
    {"leaning", sceneLeaning},
    {"friction-settle", sceneFrictionSettle},
    {"joint-pendulum", scenePendulumJoint},
    {"joint-spherical-chain", sceneSphericalChain},
    {"joint-fixed-chain", sceneFixedChain},
};

static int countBodies(Solver *s) {
    int n = 0;
    for (Rigid *b = s->bodies; b; b = b->next) n++;
    return n;
}

static Rigid **bodyArray(Solver *s, int n) {
    auto **arr = new Rigid*[n];
    int i = 0;
    for (Rigid *b = s->bodies; b; b = b->next) arr[i++] = b;
    // Reverse to match creation order (linked list is prepended)
    for (int a = 0, z = n - 1; a < z; a++, z--) {
        auto *t = arr[a]; arr[a] = arr[z]; arr[z] = t;
    }
    return arr;
}

static void writeFloat(FILE *f, float v) {
    fprintf(f, "%.17g", v);
}

int main(int argc, char **argv) {
    int frames = argc > 1 ? atoi(argv[1]) : 600;
    const char *outDir = argc > 2 ? argv[2] : ".";
    // Optional param-set overrides; absent => the reference solver defaults
    // (10 iters, betaLin 1e4, alpha 0.99) = the canonical AVBD set.
    int iterations = argc > 3 ? atoi(argv[3]) : -1;
    float betaLin = argc > 4 ? (float)atof(argv[4]) : -1.0f;

    Solver solver;
    int nScenes = sizeof(denseScenes) / sizeof(denseScenes[0]);

    for (int si = 0; si < nScenes; si++) {
        auto &sc = denseScenes[si];
        sc.fn(&solver);

        if (iterations > 0) solver.iterations = iterations;
        if (betaLin > 0.0f) solver.betaLin = betaLin;

        int n = countBodies(&solver);
        Rigid **bodies = bodyArray(&solver, n);

        std::string path = std::string(outDir) + "/dense-" + sc.name + ".json";
        FILE *f = fopen(path.c_str(), "w");
        if (!f) { fprintf(stderr, "cannot open %s\n", path.c_str()); continue; }

        // Header
        fprintf(f, "{\"scene\":\"%s\",", sc.name);
        fprintf(f, "\"params\":{\"dt\":%.17g,\"gravity\":%.17g,\"iterations\":%d,\"alpha\":%.17g,\"betaLin\":%.17g,\"betaAng\":%.17g,\"gamma\":%.17g},",
            solver.dt, solver.gravity, solver.iterations, solver.alpha, solver.betaLin, solver.betaAng, solver.gamma);
        fprintf(f, "\"bodyCount\":%d,", n);

        // Bodies (initial state, Y-up)
        fprintf(f, "\"bodies\":[");
        for (int i = 0; i < n; i++) {
            if (i) fprintf(f, ",");
            Rigid *b = bodies[i];
            fprintf(f, "{\"mass\":%.17g,\"friction\":%.17g,", b->mass, b->friction);
            fprintf(f, "\"size\":[%.17g,%.17g,%.17g],", b->size.x, b->size.y, b->size.z);
            fprintf(f, "\"initialPos\":[%.17g,%.17g,%.17g],", b->positionLin.x, b->positionLin.y, b->positionLin.z);
            fprintf(f, "\"initialQuat\":[%.17g,%.17g,%.17g,%.17g],",
                b->positionAng.x, b->positionAng.y, b->positionAng.z, b->positionAng.w);
            fprintf(f, "\"initialVel\":[%.17g,%.17g,%.17g]}", b->velocityLin.x, b->velocityLin.y, b->velocityLin.z);
        }
        fprintf(f, "],");

        // Springs (authored constraints, body-index-referenced) — the oracle reconstructs them to
        // reproduce the scene; the body indices match the creation-order `bodies` array above.
        auto indexOf = [&](Rigid *r) -> int {
            for (int i = 0; i < n; i++) if (bodies[i] == r) return i;
            return -1;
        };
        fprintf(f, "\"springs\":[");
        bool firstSpring = true;
        for (Force *fc = solver.forces; fc; fc = fc->next) {
            Spring *sp = dynamic_cast<Spring *>(fc);
            if (!sp) continue;
            if (!firstSpring) fprintf(f, ",");
            firstSpring = false;
            fprintf(f, "{\"a\":%d,\"b\":%d,", indexOf(sp->bodyA), indexOf(sp->bodyB));
            fprintf(f, "\"rA\":[%.17g,%.17g,%.17g],", sp->rA.x, sp->rA.y, sp->rA.z);
            fprintf(f, "\"rB\":[%.17g,%.17g,%.17g],", sp->rB.x, sp->rB.y, sp->rB.z);
            fprintf(f, "\"stiffness\":%.17g,\"rest\":%.17g}", sp->stiffness, sp->rest);
        }
        fprintf(f, "],");

        // Joints (authored constraints, body-index-referenced). Spherical = stiffnessAng 0, fixed = ∞.
        // INFINITY can't round-trip through JSON, so emit a 1e30 sentinel the loader maps back to Infinity
        // (the rigid-joint stabilization branch keys on isinf). fracture + world-anchored joints are not
        // modelled (non-standard AVBD: absent from the paper + webphysics) so they aren't dumped.
        auto writeStiffness = [&](float v) { writeFloat(f, std::isinf(v) ? 1e30f : v); };
        fprintf(f, "\"joints\":[");
        bool firstJoint = true;
        for (Force *fc = solver.forces; fc; fc = fc->next) {
            Joint *jt = dynamic_cast<Joint *>(fc);
            if (!jt) continue;
            if (!firstJoint) fprintf(f, ",");
            firstJoint = false;
            fprintf(f, "{\"a\":%d,\"b\":%d,", indexOf(jt->bodyA), indexOf(jt->bodyB));
            fprintf(f, "\"rA\":[%.17g,%.17g,%.17g],", jt->rA.x, jt->rA.y, jt->rA.z);
            fprintf(f, "\"rB\":[%.17g,%.17g,%.17g],", jt->rB.x, jt->rB.y, jt->rB.z);
            fprintf(f, "\"stiffnessLin\":");
            writeStiffness(jt->stiffnessLin);
            fprintf(f, ",\"stiffnessAng\":");
            writeStiffness(jt->stiffnessAng);
            fprintf(f, "}");
        }
        fprintf(f, "],");

        // Frames
        fprintf(f, "\"frames\":[");
        for (int frame = 1; frame <= frames; frame++) {
            solver.step();

            if (frame > 1) fprintf(f, ",");
            fprintf(f, "{\"frame\":%d,", frame);

            fprintf(f, "\"pos\":[");
            for (int i = 0; i < n; i++) {
                if (i) fprintf(f, ",");
                writeFloat(f, bodies[i]->positionLin.x); fprintf(f, ",");
                writeFloat(f, bodies[i]->positionLin.y); fprintf(f, ",");
                writeFloat(f, bodies[i]->positionLin.z);
            }
            fprintf(f, "],");

            fprintf(f, "\"quat\":[");
            for (int i = 0; i < n; i++) {
                if (i) fprintf(f, ",");
                writeFloat(f, bodies[i]->positionAng.x); fprintf(f, ",");
                writeFloat(f, bodies[i]->positionAng.y); fprintf(f, ",");
                writeFloat(f, bodies[i]->positionAng.z); fprintf(f, ",");
                writeFloat(f, bodies[i]->positionAng.w);
            }
            fprintf(f, "],");

            fprintf(f, "\"vel\":[");
            for (int i = 0; i < n; i++) {
                if (i) fprintf(f, ",");
                writeFloat(f, bodies[i]->velocityLin.x); fprintf(f, ",");
                writeFloat(f, bodies[i]->velocityLin.y); fprintf(f, ",");
                writeFloat(f, bodies[i]->velocityLin.z);
            }
            fprintf(f, "],");

            fprintf(f, "\"angVel\":[");
            for (int i = 0; i < n; i++) {
                if (i) fprintf(f, ",");
                writeFloat(f, bodies[i]->velocityAng.x); fprintf(f, ",");
                writeFloat(f, bodies[i]->velocityAng.y); fprintf(f, ",");
                writeFloat(f, bodies[i]->velocityAng.z);
            }
            fprintf(f, "]}");
        }
        fprintf(f, "]}");

        fclose(f);
        fprintf(stderr, "%s: %d bodies, %d frames -> %s\n", sc.name, n, frames, path.c_str());
        delete[] bodies;
    }
    return 0;
}
