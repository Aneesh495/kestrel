#include "KestrelKDTree.h"
#include <algorithm>
#include <queue>

namespace Kestrel {

void KestrelKDTree::Clear() {
    root.reset();
    pointCount = 0;
}

void KestrelKDTree::Build(const std::vector<KDPoint>& points) {
    Clear();
    if (points.empty()) return;

    std::vector<KDPoint> mutablePoints = points;
    pointCount = mutablePoints.size();
    root = BuildRecursive(mutablePoints, 0, static_cast<int>(mutablePoints.size()), 0);
}

std::unique_ptr<KestrelKDTree::KDNode> KestrelKDTree::BuildRecursive(std::vector<KDPoint>& points,
                                                                     int start,
                                                                     int end,
                                                                     int depth) {
    if (start >= end) return nullptr;

    int axis = depth % 3;
    int mid = start + (end - start) / 2;

    auto comparator = [axis](const KDPoint& a, const KDPoint& b) {
        if (axis == 0) return a.position.x < b.position.x;
        if (axis == 1) return a.position.y < b.position.y;
        return a.position.z < b.position.z;
    };

    std::nth_element(points.begin() + start, points.begin() + mid, points.begin() + end, comparator);

    auto node = std::make_unique<KDNode>(points[mid], axis);
    node->left = BuildRecursive(points, start, mid, depth + 1);
    node->right = BuildRecursive(points, mid + 1, end, depth + 1);

    return node;
}

bool KestrelKDTree::FindNearest(const Vector3& query, KDPoint& outNearest, double& outDistanceSq) const {
    if (!root) return false;

    const KDNode* bestNode = nullptr;
    double bestDistSq = 1e30;

    NearestRecursive(root.get(), query, bestNode, bestDistSq);

    if (bestNode) {
        outNearest = bestNode->point;
        outDistanceSq = bestDistSq;
        return true;
    }
    return false;
}

void KestrelKDTree::NearestRecursive(const KDNode* node,
                                     const Vector3& query,
                                     const KDNode*& bestNode,
                                     double& bestDistSq) const {
    if (!node) return;

    double dSq = (node->point.position - query).NormSq();
    if (dSq < bestDistSq) {
        bestDistSq = dSq;
        bestNode = node;
    }

    double delta = 0.0;
    if (node->axis == 0) delta = query.x - node->point.position.x;
    else if (node->axis == 1) delta = query.y - node->point.position.y;
    else delta = query.z - node->point.position.z;

    const KDNode* first = delta <= 0.0 ? node->left.get() : node->right.get();
    const KDNode* second = delta <= 0.0 ? node->right.get() : node->left.get();

    NearestRecursive(first, query, bestNode, bestDistSq);

    // Check if separating hyperplane distance is smaller than current best radius
    if (delta * delta < bestDistSq) {
        NearestRecursive(second, query, bestNode, bestDistSq);
    }
}

std::vector<KDPoint> KestrelKDTree::FindWithinRadius(const Vector3& query, double radius) const {
    std::vector<KDPoint> results;
    if (!root || radius <= 0.0) return results;

    double radiusSq = radius * radius;
    RadiusRecursive(root.get(), query, radiusSq, results);
    return results;
}

void KestrelKDTree::RadiusRecursive(const KDNode* node,
                                    const Vector3& query,
                                    double radiusSq,
                                    std::vector<KDPoint>& results) const {
    if (!node) return;

    double dSq = (node->point.position - query).NormSq();
    if (dSq <= radiusSq) {
        results.push_back(node->point);
    }

    double delta = 0.0;
    if (node->axis == 0) delta = query.x - node->point.position.x;
    else if (node->axis == 1) delta = query.y - node->point.position.y;
    else delta = query.z - node->point.position.z;

    if (delta - std::sqrt(radiusSq) <= 0.0) {
        RadiusRecursive(node->left.get(), query, radiusSq, results);
    }
    if (delta + std::sqrt(radiusSq) >= 0.0) {
        RadiusRecursive(node->right.get(), query, radiusSq, results);
    }
}

std::vector<KDPoint> KestrelKDTree::FindKNearest(const Vector3& query, size_t k) const {
    std::vector<KDPoint> results;
    if (!root || k == 0) return results;

    struct HeapElement {
        double distSq;
        KDPoint point;
        bool operator<(const HeapElement& o) const {
            return distSq < o.distSq;
        }
    };
    std::priority_queue<HeapElement> pq;

    std::function<void(const KDNode*)> search = [&](const KDNode* node) {
        if (!node) return;

        double dSq = (node->point.position - query).NormSq();
        if (pq.size() < k) {
            pq.push({dSq, node->point});
        } else if (dSq < pq.top().distSq) {
            pq.pop();
            pq.push({dSq, node->point});
        }

        double delta = 0.0;
        if (node->axis == 0) delta = query.x - node->point.position.x;
        else if (node->axis == 1) delta = query.y - node->point.position.y;
        else delta = query.z - node->point.position.z;

        const KDNode* first = delta <= 0.0 ? node->left.get() : node->right.get();
        const KDNode* second = delta <= 0.0 ? node->right.get() : node->left.get();

        search(first);

        double maxDist = pq.size() < k ? 1e30 : pq.top().distSq;
        if (delta * delta < maxDist) {
            search(second);
        }
    };

    search(root.get());

    while (!pq.empty()) {
        results.push_back(pq.top().point);
        pq.pop();
    }
    std::reverse(results.begin(), results.end());
    return results;
}

} // namespace Kestrel
