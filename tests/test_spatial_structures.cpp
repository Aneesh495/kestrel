#include "Spatial/KestrelOctree.h"
#include "Spatial/KestrelKDTree.h"
#include <iostream>
#include <cassert>
#include <random>

using namespace Kestrel;

void TestOctreeRaycasting() {
    BoundingBox3D world(Vector3(-100,-100,-100), Vector3(100,100,100));
    KestrelOctree octree(world, 6, 4);

    // Insert obstacle box at [10, 0, 0] with width 2
    octree.Insert(42, BoundingBox3D(Vector3(9.0, -1.0, -1.0), Vector3(11.0, 1.0, 1.0)));

    // Raycast directly along X axis from [0, 0, 0]
    RaycastHit hit;
    bool bHit = octree.Raycast(Vector3(0, 0, 0), Vector3(1, 0, 0), 50.0, hit);
    assert(bHit);
    assert(hit.objectId == 42);
    assert(std::abs(hit.distance - 9.0) < 1e-4);
    assert(std::abs(hit.impactPoint.x - 9.0) < 1e-4);

    // Raycast in opposite direction: should miss
    RaycastHit missHit;
    bool bMiss = octree.Raycast(Vector3(0, 0, 0), Vector3(-1, 0, 0), 50.0, missHit);
    assert(!bMiss);

    // Sphere query around obstacle
    std::vector<int> nearby;
    octree.QuerySphere(Vector3(10, 0, 0), 3.0, nearby);
    assert(nearby.size() == 1);
    assert(nearby[0] == 42);

    std::cout << "  [PASS] 3D Octree Raycasting, AABB Intersection, and Sphere Queries\n";
}

void TestKDTreeNearestNeighborStress() {
    std::mt19937 rng(777);
    std::uniform_real_distribution<double> dist(-50.0, 50.0);

    const size_t NUM_POINTS = 5000;
    std::vector<KDPoint> points;
    points.reserve(NUM_POINTS);

    for (size_t i = 0; i < NUM_POINTS; ++i) {
        points.emplace_back(Vector3(dist(rng), dist(rng), dist(rng)), static_cast<int>(i));
    }

    KestrelKDTree kdtree;
    kdtree.Build(points);
    assert(kdtree.Size() == NUM_POINTS);

    // Verify 100 random queries against brute-force linear search
    for (int q = 0; q < 100; ++q) {
        Vector3 query(dist(rng), dist(rng), dist(rng));

        KDPoint kdNearest;
        double kdDistSq = 0.0;
        bool found = kdtree.FindNearest(query, kdNearest, kdDistSq);
        assert(found);

        // Brute-force ground truth
        double bestLinearDistSq = 1e30;
        int bestLinearId = -1;
        for (const auto& pt : points) {
            double dSq = (pt.position - query).NormSq();
            if (dSq < bestLinearDistSq) {
                bestLinearDistSq = dSq;
                bestLinearId = pt.id;
            }
        }

        assert(std::abs(kdDistSq - bestLinearDistSq) < 1e-9);
        assert(kdNearest.id == bestLinearId);
    }

    std::cout << "  [PASS] 3D K-D Tree 5,000-Point Stress Test and Nearest Neighbor Verification\n";
}

int main() {
    std::cout << "=== KESTREL TEST SUITE: SPATIAL ACCELERATION STRUCTURES ===\n";
    TestOctreeRaycasting();
    TestKDTreeNearestNeighborStress();
    std::cout << "ALL SPATIAL STRUCTURE TESTS PASSED.\n\n";
    return 0;
}
