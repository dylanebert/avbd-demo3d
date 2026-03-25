// Dense fixture generator for AVBD parity testing.
// Outputs Y-up JSON fixtures (converted from native Z-up).
//
// Build: g++ -std=c++17 -O2 source/solver.cpp source/rigid.cpp source/force.cpp source/manifold.cpp source/collide.cpp source/joint.cpp source/spring.cpp harness-dense.cpp -I source -o harness-dense
// Run:   ./harness-dense 600 ../../shallot/packages/shallot/tests/fixtures/avbd/

#include "solver.h"
#include "scenes.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

struct SceneEntry { const char *name; void (*fn)(Solver *); };

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

// Z-up to Y-up conversion
// pos: (x, y, z) -> (x, z, y)
// quat: (x, y, z, w) -> (-x, -z, -y, w)
// vel/size: same as pos

static void writeFloat(FILE *f, float v) {
    fprintf(f, "%.17g", v);
}

int main(int argc, char **argv) {
    int frames = argc > 1 ? atoi(argv[1]) : 600;
    const char *outDir = argc > 2 ? argv[2] : ".";

    Solver solver;
    int nScenes = sizeof(denseScenes) / sizeof(denseScenes[0]);

    for (int si = 0; si < nScenes; si++) {
        auto &sc = denseScenes[si];
        sc.fn(&solver);

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
            // size: Z-up (sx, sy, sz) -> Y-up (sx, sz, sy)
            fprintf(f, "\"size\":[%.17g,%.17g,%.17g],", b->size.x, b->size.z, b->size.y);
            // initialPos: Z-up (x, y, z) -> Y-up (x, z, y)
            fprintf(f, "\"initialPos\":[%.17g,%.17g,%.17g],", b->positionLin.x, b->positionLin.z, b->positionLin.y);
            // initialQuat: Z-up (x, y, z, w) -> Y-up (-x, -z, -y, w)
            fprintf(f, "\"initialQuat\":[%.17g,%.17g,%.17g,%.17g]}",
                -b->positionAng.x, -b->positionAng.z, -b->positionAng.y, b->positionAng.w);
        }
        fprintf(f, "],");

        // Frames
        fprintf(f, "\"frames\":[");
        for (int frame = 1; frame <= frames; frame++) {
            solver.step();

            if (frame > 1) fprintf(f, ",");
            fprintf(f, "{\"frame\":%d,", frame);

            // pos (Y-up)
            fprintf(f, "\"pos\":[");
            for (int i = 0; i < n; i++) {
                if (i) fprintf(f, ",");
                writeFloat(f, bodies[i]->positionLin.x); fprintf(f, ",");
                writeFloat(f, bodies[i]->positionLin.z); fprintf(f, ",");
                writeFloat(f, bodies[i]->positionLin.y);
            }
            fprintf(f, "],");

            // quat (Y-up)
            fprintf(f, "\"quat\":[");
            for (int i = 0; i < n; i++) {
                if (i) fprintf(f, ",");
                writeFloat(f, -bodies[i]->positionAng.x); fprintf(f, ",");
                writeFloat(f, -bodies[i]->positionAng.z); fprintf(f, ",");
                writeFloat(f, -bodies[i]->positionAng.y); fprintf(f, ",");
                writeFloat(f, bodies[i]->positionAng.w);
            }
            fprintf(f, "],");

            // vel (Y-up)
            fprintf(f, "\"vel\":[");
            for (int i = 0; i < n; i++) {
                if (i) fprintf(f, ",");
                writeFloat(f, bodies[i]->velocityLin.x); fprintf(f, ",");
                writeFloat(f, bodies[i]->velocityLin.z); fprintf(f, ",");
                writeFloat(f, bodies[i]->velocityLin.y);
            }
            fprintf(f, "],");

            // angVel (Y-up)
            fprintf(f, "\"angVel\":[");
            for (int i = 0; i < n; i++) {
                if (i) fprintf(f, ",");
                writeFloat(f, -bodies[i]->velocityAng.x); fprintf(f, ",");
                writeFloat(f, -bodies[i]->velocityAng.z); fprintf(f, ",");
                writeFloat(f, -bodies[i]->velocityAng.y);
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
