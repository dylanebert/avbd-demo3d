/*
 * Copyright (c) 2026 Chris Giles
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Chris Giles makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 */

#pragma once

#include "maths.h"
#include "solver.h"

static void sceneEmpty(Solver *solver)
{
    solver->clear();
}

static void sceneGround(Solver *solver)
{
    solver->clear();
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, 0, 0}, {0, 0, 0});
    new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {0, 4, 0});
}

static void sceneDynamicFriction(Solver *solver)
{
    solver->clear();
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, 0, 0}, {0, 0, 0});
    for (int x = 0; x <= 10; x++)
        new Rigid(solver, {1, 0.5f, 1}, 1.0f, 5.0f - (x / 10.0f * 5.0f), {0, 0.75f, -30.0f + x * 2.0f}, {10.0f, 0, 0});
}

static void sceneStaticFriction(Solver *solver)
{
    solver->clear();
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, 0, 0});

    const float angle = rad(30.0f);
    Rigid *ramp = new Rigid(solver, {40, 1, 24}, 0.0f, 1.0f, {0, 3, 0});
    ramp->positionAng = {0, 0, sinf(angle * 0.5f), cosf(angle * 0.5f)};

    float3 rampTangent = normalize(rotate(ramp->positionAng, float3{1, 0, 0}));
    float3 rampNormal = normalize(rotate(ramp->positionAng, float3{0, 1, 0}));

    for (int i = 0; i <= 10; i++)
    {
        float friction = i / 10.0f * 0.25f + 0.25f;
        float z = -10.0f + i * 2.0f;
        float3 pos = ramp->positionLin + rampTangent * -12.0f + float3{0, 0, z} + rampNormal * 1.05f;
        new Rigid(solver, {1, 1, 1}, 1.0f, friction, pos);
    }
}

static void scenePyramid(Solver *solver)
{
    const int SIZE = 16;
    solver->clear();
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0.0f, -0.5f, 0.0f});

    for (int y = 0; y < SIZE; y++)
        for (int x = 0; x < SIZE - y; x++)
            new Rigid(solver, {1, 0.5f, 0.5f}, 1.0f, 0.5f, {x * 1.01f + y * 0.5f - SIZE / 2.0f, y * 0.85f + 0.5f, 0.0f});
}

static void sceneRope(Solver *solver)
{
    solver->clear();
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, -20, 0});

    Rigid *prev = 0;
    for (int i = 0; i < 20; i++)
    {
        Rigid *curr = new Rigid(solver, {1, 0.5f, 0.5f}, i == 0 ? 0.0f : 1.0f, 0.5f, {(float)i, 10.0f, 0.0f});
        if (prev)
            new Joint(solver, prev, curr, {0.5f, 0, 0}, {-0.5f, 0, 0});
        prev = curr;
    }
}

static void sceneHeavyRope(Solver *solver)
{
    const int N = 20;
    const float SIZE = 5;
    solver->clear();
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, -20, 0});

    Rigid *prev = 0;
    for (int i = 0; i < N; i++)
    {
        Rigid *curr = new Rigid(solver, i == N - 1 ? float3{SIZE, SIZE, SIZE} : float3{1, 0.5f, 0.5f},
                                i == 0 ? 0.0f : 1.0f, 0.5f, {(float)i + (i == N - 1 ? SIZE / 2 : 0), 10.0f, 0.0f});
        if (prev)
            new Joint(solver, prev, curr, {0.5f, 0, 0}, i == N - 1 ? float3{-SIZE / 2, 0, 0} : float3{-0.5f, 0, 0});
        prev = curr;
    }
}

static void sceneSpring(Solver *solver)
{
    solver->clear();
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, 0, 0});

    Rigid *anchor = new Rigid(solver, {1, 1, 1}, 0.0f, 0.5f, {0, 14.0f, 0});
    Rigid *block = new Rigid(solver, {2, 2, 2}, 1.0f, 0.5f, {0, 8.0f, 0});
    new Spring(solver, anchor, block, {0, 0, 0}, {0, 0, 0}, 100.0f, 4.0f);
}

static void sceneSpringsRatio(Solver *solver)
{
    const int N = 8;
    solver->clear();
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, -10, 0});

    Rigid *prev = 0;
    for (int i = 0; i < N; i++)
    {
        float x = (i - (N - 1) * 0.5f) * 3.0f;
        Rigid *curr = new Rigid(solver, {1, 0.75f, 0.75f}, i == 0 || i == N - 1 ? 0.0f : 1.0f, 0.5f, {x, 12.0f, 0.0f});
        if (prev)
            new Spring(solver, prev, curr, {0.5f, 0, 0}, {-0.5f, 0, 0}, i % 2 == 0 ? 10.0f : 10000.0f, 3.0f);
        prev = curr;
    }
}

static void sceneStack(Solver *solver)
{
    solver->clear();
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, 0, 0});
    for (int i = 0; i < 10; i++)
        new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {0, i * 1.5f + 1.0f, 0});
}

static void sceneStackRatio(Solver *solver)
{
    solver->clear();
    const float groundThickness = 1.0f;
    new Rigid(solver, {100, groundThickness, 100}, 0.0f, 0.5f, {0, 0, 0});

    float topY = groundThickness * 0.5f;
    float s = 1.0f;
    for (int i = 0; i < 4; i++)
    {
        float half = s * 0.5f;
        float centerY = topY + half;
        new Rigid(solver, {s, s, s}, 1.0f, 0.5f, {0, centerY, 0});
        topY = centerY + half;
        s *= 2.0f;
    }
}

static void sceneSoftBody(Solver *solver)
{
    solver->clear();
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, 0, 0});

    const float Klin = 1000.0f;
    const float Kang = 250.0f;
    const int W = 4;
    const int D = 4;
    const int H = 4;
    const int N = 3;
    const float size = 0.8f;
    const float half = size * 0.5f;
    const float baseY = 8.0f;
    const float stackGap = 2.0f;

    for (int i = 0; i < N; i++)
    {
        Rigid *grid[W][D][H];
        float stackY = i * (H * size + stackGap);

        for (int x = 0; x < W; x++)
        {
            for (int z = 0; z < D; z++)
            {
                for (int y = 0; y < H; y++)
                {
                    float px = (x - (W - 1) * 0.5f) * size;
                    float pz = (z - (D - 1) * 0.5f) * size;
                    float py = baseY + stackY + y * size;
                    grid[x][z][y] = new Rigid(solver, {size, size, size}, 1.0f, 0.5f, {px, py, pz});
                }
            }
        }

        for (int x = 1; x < W; x++)
            for (int z = 0; z < D; z++)
                for (int y = 0; y < H; y++)
                    new Joint(solver, grid[x - 1][z][y], grid[x][z][y], {half, 0, 0}, {-half, 0, 0}, Klin, Kang);

        for (int x = 0; x < W; x++)
            for (int z = 1; z < D; z++)
                for (int y = 0; y < H; y++)
                    new Joint(solver, grid[x][z - 1][y], grid[x][z][y], {0, 0, half}, {0, 0, -half}, Klin, Kang);

        for (int x = 0; x < W; x++)
            for (int z = 0; z < D; z++)
                for (int y = 1; y < H; y++)
                    new Joint(solver, grid[x][z][y - 1], grid[x][z][y], {0, half, 0}, {0, -half, 0}, Klin, Kang);

        for (int x = 1; x < W; x++)
            for (int z = 0; z < D; z++)
                for (int y = 1; y < H; y++)
                {
                    new IgnoreCollision(solver, grid[x - 1][z][y - 1], grid[x][z][y]);
                    new IgnoreCollision(solver, grid[x][z][y - 1], grid[x - 1][z][y]);
                }

        for (int x = 0; x < W; x++)
            for (int z = 1; z < D; z++)
                for (int y = 1; y < H; y++)
                {
                    new IgnoreCollision(solver, grid[x][z - 1][y - 1], grid[x][z][y]);
                    new IgnoreCollision(solver, grid[x][z][y - 1], grid[x][z - 1][y]);
                }

        for (int x = 1; x < W; x++)
            for (int z = 1; z < D; z++)
                for (int y = 0; y < H; y++)
                {
                    new IgnoreCollision(solver, grid[x - 1][z - 1][y], grid[x][z][y]);
                    new IgnoreCollision(solver, grid[x][z - 1][y], grid[x - 1][z][y]);
                }
    }
}

static void sceneBridge(Solver *solver)
{
    const int N = 40;
    const float plankLength = 1.0f;
    const float plankWidth = 4.0f;
    const float plankHeight = 0.5f;
    const float halfLength = plankLength * 0.5f;
    const float halfWidth = plankWidth * 0.5f;

    solver->clear();
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, 0, 0});

    Rigid *prev = 0;
    for (int i = 0; i < N; i++)
    {
        Rigid *curr = new Rigid(solver, {plankLength, plankHeight, plankWidth}, i == 0 || i == N - 1 ? 0.0f : 1.0f, 0.5f, {(float)i - N / 2.0f, 10.0f, 0.0f});
        if (prev)
        {
            new Joint(solver, prev, curr, {halfLength, 0, halfWidth}, {-halfLength, 0, halfWidth}, INFINITY, 0.0f);
            new Joint(solver, prev, curr, {halfLength, 0, -halfWidth}, {-halfLength, 0, -halfWidth}, INFINITY, 0.0f);
        }
        prev = curr;
    }

    for (int x = 0; x < N / 4; x++)
    {
        for (int y = 0; y < N / 8; y++)
        {
            new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {(float)x - N / 8.0f, (float)y + 12.0f, 0.0f});
        }
    }
}

static void sceneBreakable(Solver *solver)
{
    const int N = 10;
    const int M = 5;
    const float breakForce = 90.0f;

    solver->clear();
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, 0, 0});

    Rigid *prev = 0;
    for (int i = 0; i <= N; i++)
    {
        Rigid *curr = new Rigid(solver, {1, 0.5f, 1}, 1.0f, 0.5f, {(float)i - N / 2.0f, 6.0f, 0.0f});
        if (prev)
            new Joint(solver, prev, curr, {0.5f, 0, 0}, {-0.5f, 0, 0}, INFINITY, INFINITY, breakForce);
        prev = curr;
    }

    new Rigid(solver, {1, 5, 1}, 0.0f, 0.5f, {-N / 2.0f, 2.5f, 0});
    new Rigid(solver, {1, 5, 1}, 0.0f, 0.5f, {N / 2.0f, 2.5f, 0});

    for (int i = 0; i < M; i++)
        new Rigid(solver, {2, 1, 1}, 1.0f, 0.5f, {0, i * 2.0f + 8.0f, 0});
}

static void sceneSoftJoint(Solver *solver)
{
    solver->clear();
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, 0, 0});
    Rigid *a = new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {0, 4, 0});
    Rigid *b = new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {1, 4, 0});
    new Joint(solver, a, b, {0.5f, 0, 0}, {-0.5f, 0, 0}, 1000.0f, 250.0f);
}

static void sceneTwoBoxes(Solver *solver)
{
    solver->clear();
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, 0, 0});
    new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {0, 4, 0});
    new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {3, 4, 0});
}

static void sceneRigidJoint(Solver *solver)
{
    solver->clear();
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, 0, 0});
    Rigid *a = new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {0, 4, 0});
    Rigid *b = new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {1, 4, 0});
    new Joint(solver, a, b, {0.5f, 0, 0}, {-0.5f, 0, 0});
}

static void sceneSoftJointFree(Solver *solver)
{
    solver->clear();
    Rigid *a = new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {0, 4, 0});
    Rigid *b = new Rigid(solver, {1, 1, 1}, 1.0f, 0.5f, {1, 4, 0});
    new Joint(solver, a, b, {0.5f, 0, 0}, {-0.5f, 0, 0}, 1000.0f, 250.0f);
}

static void sceneBridgeMini(Solver *solver)
{
    const int N = 5;
    const float plankLength = 1.0f;
    const float plankWidth = 4.0f;
    const float plankHeight = 0.5f;
    const float halfLength = plankLength * 0.5f;
    const float halfWidth = plankWidth * 0.5f;

    solver->clear();
    new Rigid(solver, {100, 1, 100}, 0.0f, 0.5f, {0, 0, 0});

    Rigid *prev = 0;
    for (int i = 0; i < N; i++)
    {
        Rigid *curr = new Rigid(solver, {plankLength, plankHeight, plankWidth}, i == 0 || i == N - 1 ? 0.0f : 1.0f, 0.5f, {(float)i - N / 2.0f, 10.0f, 0.0f});
        if (prev)
        {
            new Joint(solver, prev, curr, {halfLength, 0, halfWidth}, {-halfLength, 0, halfWidth}, INFINITY, 0.0f);
            new Joint(solver, prev, curr, {halfLength, 0, -halfWidth}, {-halfLength, 0, -halfWidth}, INFINITY, 0.0f);
        }
        prev = curr;
    }
}

// Harness-only joint parity scenes (roadmap "Phase 6.2 — Joints"). The demo's own joint scenes
// (rope/bridge/breakable) have a ground, dozens of bodies, and contacts, so they confound the joint
// math with the contact solve. These three isolate it: contact-free (small boxes spaced so the
// broadphase sphere never overlaps a non-adjacent link), bounded (anchored to a static body so f32
// coordinate magnitude stays small over 600 frames), and non-chaotic (so the f64 oracle tracks the
// f32 C++ whole-run). They exercise the spherical joint (3 linear rows, rotation free) and the fixed
// joint (+ 3 angular rows) end to end.

// A single physical pendulum: a static pivot + a bob whose COM hangs L = 3 to the side, pinned by a
// spherical joint at the pivot. The bob swings from horizontal (rotation free) — the spherical joint's
// position+angular Jacobian coupling makes it a pin. Contact-free (the bob COM never nears the pivot).
static void scenePendulumJoint(Solver *solver)
{
    solver->clear();
    Rigid *pivot = new Rigid(solver, {0.5f, 0.5f, 0.5f}, 0.0f, 0.5f, {0, 5, 0});
    Rigid *bob = new Rigid(solver, {0.5f, 0.5f, 0.5f}, 1.0f, 0.5f, {3, 5, 0});
    new Joint(solver, pivot, bob, {0, 0, 0}, {-3, 0, 0}); // spherical (stiffnessAng defaults to 0)
}

// A 3-link spherical chain hanging vertically from a static anchor, the bottom link nudged sideways
// so it sways gently and stays near-vertical (contact-free: link centers stay 1.0 apart, outside the
// 0.91 broadphase reach). Spherical joints (rotation free) → a floppy chain.
static void sceneSphericalChain(Solver *solver)
{
    solver->clear();
    Rigid *prev = new Rigid(solver, {0.5f, 0.5f, 0.5f}, 0.0f, 0.5f, {0, 8, 0});
    for (int i = 1; i <= 3; i++)
    {
        float3 vel = i == 3 ? float3{1.0f, 0, 0} : float3{0, 0, 0};
        Rigid *curr = new Rigid(solver, {0.5f, 0.5f, 0.5f}, 1.0f, 0.5f, {0, 8.0f - i, 0}, vel);
        new Joint(solver, prev, curr, {0, -0.5f, 0}, {0, 0.5f, 0}); // spherical, vertical link
        prev = curr;
    }
}

// A 3-link fixed-joint chain: a horizontal rigid cantilever fixed to a static anchor. The fixed joints
// (stiffnessLin = stiffnessAng = INFINITY) lock relative orientation, so the arm resists gravity's
// torque and settles nearly straight — the angular-row test. Contact-free (centers 1.0 apart, stays
// straight so no folding).
static void sceneFixedChain(Solver *solver)
{
    solver->clear();
    Rigid *prev = new Rigid(solver, {0.5f, 0.5f, 0.5f}, 0.0f, 0.5f, {0, 8, 0});
    for (int i = 1; i <= 3; i++)
    {
        Rigid *curr = new Rigid(solver, {0.5f, 0.5f, 0.5f}, 1.0f, 0.5f, {(float)i, 8, 0});
        new Joint(solver, prev, curr, {0.5f, 0, 0}, {-0.5f, 0, 0}, INFINITY, INFINITY); // fixed
        prev = curr;
    }
}

static void (*scenes[])(Solver *) = {
    sceneEmpty,
    sceneGround,
    sceneDynamicFriction,
    sceneStaticFriction,
    scenePyramid,
    sceneRope,
    sceneHeavyRope,
    sceneSpring,
    sceneSpringsRatio,
    sceneStack,
    sceneStackRatio,
    sceneSoftBody,
    sceneBridge,
    sceneBreakable};

static const char *sceneNames[] = {
    "Empty",
    "Ground",
    "Dynamic Friction",
    "Static Friction",
    "Pyramid",
    "Rope",
    "Heavy Rope",
    "Spring",
    "Spring Ratio",
    "Stack",
    "Stack Ratio",
    "Soft Body",
    "Bridge",
    "Breakable"};

static const int sceneCount = 14;
