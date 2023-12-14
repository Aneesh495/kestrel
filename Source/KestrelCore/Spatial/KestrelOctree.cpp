#include "KestrelOctree.h"
#include <limits>

namespace Kestrel {

void KestrelOctree::OctreeNode::Subdivide() {
    Vector3 center = bounds.Center();
    Vector3 min = bounds.minPoint;
    Vector3 max = bounds.maxPoint;

    // Construct 8 octant bounding boxes
    children[0] = std::make_unique<OctreeNode>(BoundingBox3D(Vector3(min.x, min.y, min.z), Vector3(center.x, center.y, center.z)), depth + 1);
    children[1] = std::make_unique<OctreeNode>(BoundingBox3D(Vector3(center.x, min.y, min.z), Vector3(max.x, center.y, center.z)), depth + 1);
    children[2] = std::make_unique<OctreeNode>(BoundingBox3D(Vector3(min.x, center.y, min.z), Vector3(center.x, max.y, center.z)), depth + 1);
    children[3] = std::make_unique<OctreeNode>(BoundingBox3D(Vector3(center.x, center.y, min.z), Vector3(max.x, max.y, center.z)), depth + 1);
    children[4] = std::make_unique<OctreeNode>(BoundingBox3D(Vector3(min.x, min.y, center.z), Vector3(center.x, center.y, max.z)), depth + 1);
    children[5] = std::make_unique<OctreeNode>(BoundingBox3D(Vector3(center.x, min.y, center.z), Vector3(max.x, center.y, max.z)), depth + 1);
    children[6] = std::make_unique<OctreeNode>(BoundingBox3D(Vector3(min.x, center.y, center.z), Vector3(center.x, max.y, max.z)), depth + 1);
    children[7] = std::make_unique<OctreeNode>(BoundingBox3D(Vector3(center.x, center.y, center.z), Vector3(max.x, max.y, max.z)), depth + 1);

    bIsLeaf = false;
}

KestrelOctree::KestrelOctree(const BoundingBox3D& worldBounds, int maxDepth, int maxObjectsPerNode)
    : root(std::make_unique<OctreeNode>(worldBounds, 0))
    , maxTreeDepth(maxDepth)
    , maxObjectsPerLeaf(maxObjectsPerNode)
    , totalObjects(0) {}

void KestrelOctree::Clear() {
    BoundingBox3D oldBounds = root->bounds;
    root = std::make_unique<OctreeNode>(oldBounds, 0);
    totalObjects = 0;
}

void KestrelOctree::Insert(int objectId, const BoundingBox3D& bounds) {
    SpatialPrimitive prim{objectId, bounds};
    InsertRecursive(root.get(), prim);
    totalObjects++;
}

void KestrelOctree::InsertRecursive(OctreeNode* node, const SpatialPrimitive& prim) {
    if (node->bIsLeaf) {
        if (node->objects.size() < static_cast<size_t>(maxObjectsPerLeaf) || node->depth >= maxTreeDepth) {
            node->objects.push_back(prim);
            return;
        }

        // Subdivide leaf node
        node->Subdivide();

        // Re-distribute existing objects into children
        std::vector<SpatialPrimitive> existing = std::move(node->objects);
        node->objects.clear();

        for (const auto& obj : existing) {
            InsertRecursive(node, obj);
        }
    }

    // Insert into all children that intersect the primitive bounds
    bool insertedInChild = false;
    for (int i = 0; i < 8; ++i) {
        const auto& cBounds = node->children[i]->bounds;
        // Simple AABB overlap check
        if (cBounds.minPoint.x <= prim.bounds.maxPoint.x && cBounds.maxPoint.x >= prim.bounds.minPoint.x &&
            cBounds.minPoint.y <= prim.bounds.maxPoint.y && cBounds.maxPoint.y >= prim.bounds.minPoint.y &&
            cBounds.minPoint.z <= prim.bounds.maxPoint.z && cBounds.maxPoint.z >= prim.bounds.minPoint.z) {
            InsertRecursive(node->children[i].get(), prim);
            insertedInChild = true;
        }
    }

    if (!insertedInChild) {
        // Fallback: retain at current node
        node->objects.push_back(prim);
    }
}

bool KestrelOctree::Raycast(const Vector3& origin,
                            const Vector3& direction,
                            double maxDistance,
                            RaycastHit& outHit) const {
    Vector3 normDir = direction.Normalized();
    Vector3 invDir(
        std::abs(normDir.x) > 1e-12 ? 1.0 / normDir.x : 1e12,
        std::abs(normDir.y) > 1e-12 ? 1.0 / normDir.y : 1e12,
        std::abs(normDir.z) > 1e-12 ? 1.0 / normDir.z : 1e12
    );

    RaycastHit bestHit;
    bestHit.bHit = false;
    bestHit.distance = maxDistance;

    if (RaycastRecursive(root.get(), origin, normDir, invDir, maxDistance, bestHit)) {
        outHit = bestHit;
        return true;
    }
    return false;
}

bool KestrelOctree::RaycastRecursive(const OctreeNode* node,
                                     const Vector3& origin,
                                     const Vector3& dir,
                                     const Vector3& invDir,
                                     double maxDistance,
                                     RaycastHit& bestHit) const {
    double boxDist = 0.0;
    if (!node->bounds.IntersectsRay(origin, invDir, bestHit.distance, boxDist)) {
        return false;
    }

    bool hitAny = false;

    // Check objects in this node
    for (const auto& obj : node->objects) {
        double objDist = 0.0;
        if (obj.bounds.IntersectsRay(origin, invDir, bestHit.distance, objDist)) {
            if (objDist < bestHit.distance) {
                bestHit.bHit = true;
                bestHit.distance = objDist;
                bestHit.impactPoint = origin + dir * objDist;
                bestHit.objectId = obj.id;
                // Approximate normal from box center
                Vector3 center = obj.bounds.Center();
                bestHit.impactNormal = (bestHit.impactPoint - center).Normalized();
                hitAny = true;
            }
        }
    }

    // If not a leaf, recurse into children
    if (!node->bIsLeaf) {
        // Sort children by distance to origin along ray for early exit
        struct ChildDist {
            int index;
            double dist;
        };
        std::vector<ChildDist> sortedChildren;
        for (int i = 0; i < 8; ++i) {
            double cDist = 0.0;
            if (node->children[i]->bounds.IntersectsRay(origin, invDir, bestHit.distance, cDist)) {
                sortedChildren.push_back({i, cDist});
            }
        }
        std::sort(sortedChildren.begin(), sortedChildren.end(), [](const ChildDist& a, const ChildDist& b) {
            return a.dist < b.dist;
        });

        for (const auto& cd : sortedChildren) {
            if (cd.dist >= bestHit.distance) break;
            if (RaycastRecursive(node->children[cd.index].get(), origin, dir, invDir, maxDistance, bestHit)) {
                hitAny = true;
            }
        }
    }

    return hitAny;
}

void KestrelOctree::QuerySphere(const Vector3& center,
                                double radius,
                                std::vector<int>& outObjectIds) const {
    QuerySphereRecursive(root.get(), center, radius * radius, outObjectIds);
}

void KestrelOctree::QuerySphereRecursive(const OctreeNode* node,
                                         const Vector3& center,
                                         double radiusSq,
                                         std::vector<int>& outObjectIds) const {
    // Check distance between sphere center and node AABB
    Vector3 closestPoint(
        std::max(node->bounds.minPoint.x, std::min(center.x, node->bounds.maxPoint.x)),
        std::max(node->bounds.minPoint.y, std::min(center.y, node->bounds.maxPoint.y)),
        std::max(node->bounds.minPoint.z, std::min(center.z, node->bounds.maxPoint.z))
    );

    double distSq = (closestPoint - center).NormSq();
    if (distSq > radiusSq) {
        return;
    }

    for (const auto& obj : node->objects) {
        Vector3 objClosest(
            std::max(obj.bounds.minPoint.x, std::min(center.x, obj.bounds.maxPoint.x)),
            std::max(obj.bounds.minPoint.y, std::min(center.y, obj.bounds.maxPoint.y)),
            std::max(obj.bounds.minPoint.z, std::min(center.z, obj.bounds.maxPoint.z))
        );
        if ((objClosest - center).NormSq() <= radiusSq) {
            if (std::find(outObjectIds.begin(), outObjectIds.end(), obj.id) == outObjectIds.end()) {
                outObjectIds.push_back(obj.id);
            }
        }
    }

    if (!node->bIsLeaf) {
        for (int i = 0; i < 8; ++i) {
            QuerySphereRecursive(node->children[i].get(), center, radiusSq, outObjectIds);
        }
    }
}

} // namespace Kestrel
