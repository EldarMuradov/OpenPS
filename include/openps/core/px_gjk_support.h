#ifndef _OPENPS_GJK_
#define _OPENPS_GJK_

#include <openps_decl.h>

#include <extensions/PxGjkQueryExt.h>

namespace openps
{
    using namespace physx;

    using px_gjk_support = PxGjkQuery::Support;
    using px_sphere_support = PxGjkQueryExt::SphereSupport;
    using px_box_support = PxGjkQueryExt::BoxSupport;
    using px_capsule_support = PxGjkQueryExt::CapsuleSupport;
    using px_convex_support = PxGjkQueryExt::ConvexMeshSupport;

    struct px_gjk_query
    {
        bool overlapSphere(const PxVec3& center1, float radius1, const PxVec3& center2, float radius2) noexcept
        {
            px_sphere_support support1, support2;

            support1.radius = radius1;
            support2.radius = radius2;

            return overlapImpl(support1, support2, PxTransform(center1), PxTransform(center2));
        }

        bool overlapBox(const PxVec3& center1, const PxVec3& halfExtents1, const PxVec3& center2, const PxVec3& halfExtents2) noexcept
        {
            px_box_support support1, support2;

            support1.halfExtents = halfExtents1;
            support2.halfExtents = halfExtents2;

            return overlapImpl(support1, support2, PxTransform(center1), PxTransform(center2));
        }

        bool overlapCapsule(const PxVec3& center1, float halfHeight1, float radius1,
            const PxVec3& center2, float halfHeight2, float radius2) noexcept
        {
            px_capsule_support support1, support2;

            support1.halfHeight = halfHeight1;
            support2.halfHeight = halfHeight2;
            support1.radius = radius1;
            support2.radius = radius2;

            return overlapImpl(support1, support2, PxTransform(center1), PxTransform(center2));
        }

        bool raycastShape(const px_gjk_support& supportShape, const PxVec3& shapePose,
            const PxVec3& rayStart, const PxVec3& direction, float maxDist, float& hitDist) noexcept
        {
            PxVec3 normal, point;
            return raycastImpl(supportShape, PxTransform(shapePose),
                rayStart, direction, maxDist, hitDist, normal, point);
        }

    private:
        bool overlapImpl(const PxGjkQuery::Support& a, const PxGjkQuery::Support& b,
            const PxTransform& poseA, const PxTransform& poseB) noexcept;

        bool sweepImpl(const PxGjkQuery::Support& a, const PxGjkQuery::Support& b,
            const PxTransform& poseA, const PxTransform& poseB,
            const PxVec3& unitDir, PxReal maxDist,
            PxReal& hitDist, PxVec3& n, PxVec3& p) noexcept;

        bool raycastImpl(const PxGjkQuery::Support& shape, const PxTransform& pose,
            const PxVec3& rayStart, const PxVec3& unitDir, PxReal maxDist,
            PxReal& hitDist, PxVec3& n, PxVec3& p) noexcept;

        bool proximityInfoImpl(const PxGjkQuery::Support& a, const PxGjkQuery::Support& b,
            const PxTransform& poseA, const PxTransform& poseB,
            PxReal contactDistance, PxReal toleranceLength,
            PxVec3& pointA, PxVec3& pointB, PxVec3& separatingAxis,
            PxReal& separation) noexcept;

        bool generateContactsImpl(
            const PxGjkQuery::Support& a, const PxGjkQuery::Support& b,
            const PxTransform& poseA, const PxTransform& poseB,
            PxReal contactDistance, PxReal toleranceLength,
            PxContactBuffer& contactBuffer) noexcept;
    };
}

#endif