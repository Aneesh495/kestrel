#pragma once

#include "Math/KestrelMathTypes.h"
#include <vector>
#include <memory>
#include <algorithm>

namespace Kestrel {

/**
 * Axis-Aligned Bounding Box (AABB) in 3D Euclidean space.
 */
struct BoundingBox3D {
    Vector3 minPoint{-1.0, -1.0, -1.0};
    Vector3 maxPoint{1.0, 1.0, 1.0};

    constexpr BoundingBox3D() = default;
    constexpr BoundingBox3D(const Vector3& inMin, const Vector3& inMax)
        : minPoint(inMin), maxPoint(inMax) {}

    bool Contains(const Vector3& p) const {
        return p.x >= minPoint.x && p.x <= maxPoint.x &&
               p.y >= minPoint.y && p.y <= maxPoint.y &&
               p.z >= minPoint.z && p.z <= maxPoint.z;
    }

    bool IntersectsRay(const Vector3& origin,
                       const Vector3& invDir,
                       double maxRange,
                       double& outDistance) const {
        double t1 = (minPoint.x - origin.x) * invDir.x;
        double t2 = (maxPoint.x - origin.x) * invDir.x;
        double t3 = (minPoint.y - origin.y) * invDir.y;
        double t4 = (maxPoint.y - origin.y) * invDir.y;
        double t5 = (minPoint.z - origin.z) * invDir.z;
        double t6 = (maxPoint.z - origin.z) * invDir.z;

        double tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
        double tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

        if (tmax < 0 || tmin > tmax || tmin > maxRange) {
            return false;
        }

        outDistance = tmin > 0.0 ? tmin : tmax;
        return true;
    }

    Vector3 Center() const { return (minPoint + maxPoint) * 0.5; }
    Vector3 Extents() const { return (maxPoint - minPoint) * 0.5; }
};

struct RaycastHit {
    bool bHit{false};
    double distance{0.0};
    Vector3 impactPoint{0.0, 0.0, 0.0};
    Vector3 impactNormal{0.0, 0.0, -1.0};
    int objectId{-1};
};

struct SpatialPrimitive {
    int id{0};
    BoundingBox3D bounds;
};

/**
 * High-Throughput 3D Spatial Octree.
 * Accelerates LiDAR sensor raycasts and obstacle proximity detection.
 */
class KestrelOctree {
public:
    KestrelOctree(const BoundingBox3D& worldBounds, int maxDepth = 6, int maxObjectsPerNode = 8);

    void Insert(int objectId, const BoundingBox3D& bounds);
    void Clear();

    bool Raycast(const Vector3& origin,
                 const Vector3& direction,
                 double maxDistance,
                 RaycastHit& outHit) const;

    void QuerySphere(const Vector3& center,
                     double radius,
                     std::vector<int>& outObjectIds) const;

    int GetTotalObjectCount() const { return totalObjects; }

private:
    struct OctreeNode {
        BoundingBox3D bounds;
        int depth{0};
        std::vector<SpatialPrimitive> objects;
        std::array<std::unique_ptr<OctreeNode>, 8> children;
        bool bIsLeaf{true};

        OctreeNode(const BoundingBox3D& inBounds, int inDepth)
            : bounds(inBounds), depth(inDepth) {}

        void Subdivide();
    };

    std::unique_ptr<OctreeNode> root;
    int maxTreeDepth;
    int maxObjectsPerLeaf;
    int totalObjects{0};

    void InsertRecursive(OctreeNode* node, const SpatialPrimitive& prim);
    bool RaycastRecursive(const OctreeNode* node,
                          const Vector3& origin,
                          const Vector3& dir,
                          const Vector3& invDir,
                          double maxDistance,
                          RaycastHit& bestHit) const;
    void QuerySphereRecursive(const OctreeNode* node,
                              const Vector3& center,
                              double radiusSq,
                              std::vector<int>& outObjectIds) const;
};

} // namespace Kestrel
