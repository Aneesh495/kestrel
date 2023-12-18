#pragma once

#include "Math/KestrelMathTypes.h"
#include <vector>
#include <memory>

namespace Kestrel {

struct KDPoint {
    Vector3 position;
    int id{0};

    KDPoint() = default;
    KDPoint(const Vector3& pos, int inId = 0) : position(pos), id(inId) {}
};

/**
 * High-Performance Balanced 3D K-D Tree.
 * Provides O(log N) nearest-neighbor waypoint discovery, collision cluster analysis,
 * and spatial radius queries for autonomous navigation.
 */
class KestrelKDTree {
public:
    KestrelKDTree() = default;

    void Build(const std::vector<KDPoint>& points);
    void Clear();

    bool FindNearest(const Vector3& query, KDPoint& outNearest, double& outDistanceSq) const;
    std::vector<KDPoint> FindKNearest(const Vector3& query, size_t k) const;
    std::vector<KDPoint> FindWithinRadius(const Vector3& query, double radius) const;

    size_t Size() const { return pointCount; }
    bool IsEmpty() const { return pointCount == 0; }

private:
    struct KDNode {
        KDPoint point;
        int axis{0}; // 0 = X, 1 = Y, 2 = Z
        std::unique_ptr<KDNode> left;
        std::unique_ptr<KDNode> right;

        KDNode(const KDPoint& pt, int inAxis) : point(pt), axis(inAxis) {}
    };

    std::unique_ptr<KDNode> root;
    size_t pointCount{0};

    std::unique_ptr<KDNode> BuildRecursive(std::vector<KDPoint>& points, int start, int end, int depth);
    void NearestRecursive(const KDNode* node, const Vector3& query, const KDNode*& bestNode, double& bestDistSq) const;
    void RadiusRecursive(const KDNode* node, const Vector3& query, double radiusSq, std::vector<KDPoint>& results) const;
};

} // namespace Kestrel
