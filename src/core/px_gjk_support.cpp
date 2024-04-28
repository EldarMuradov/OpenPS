#include "core/px_gjk_support.h"

bool openps::px_gjk_query::overlapImpl(const PxGjkQuery::Support& a, const PxGjkQuery::Support& b, const PxTransform& poseA, const PxTransform& poseB) noexcept
{
    return PxGjkQuery::overlap(a, b, poseA, poseB);
}

bool openps::px_gjk_query::sweepImpl(const PxGjkQuery::Support& a, const PxGjkQuery::Support& b, const PxTransform& poseA, const PxTransform& poseB, const PxVec3& unitDir, PxReal maxDist, PxReal& hitDist, PxVec3& n, PxVec3& p) noexcept
{
    return PxGjkQuery::sweep(a, b, poseA, poseB, unitDir, maxDist, hitDist, n, p);
}

bool openps::px_gjk_query::raycastImpl(const PxGjkQuery::Support& shape, const PxTransform& pose, const PxVec3& rayStart, const PxVec3& unitDir, PxReal maxDist, PxReal& hitDist, PxVec3& n, PxVec3& p) noexcept
{
    return PxGjkQuery::raycast(shape, pose, rayStart, unitDir, maxDist, hitDist, n, p);
}

bool openps::px_gjk_query::proximityInfoImpl(const PxGjkQuery::Support& a, const PxGjkQuery::Support& b, const PxTransform& poseA, const PxTransform& poseB, PxReal contactDistance, PxReal toleranceLength, PxVec3& pointA, PxVec3& pointB, PxVec3& separatingAxis, PxReal& separation) noexcept
{
    return PxGjkQuery::proximityInfo(a, b, poseA, poseB, contactDistance, toleranceLength,
        pointA, pointB, separatingAxis, separation);
}

bool openps::px_gjk_query::generateContactsImpl(const PxGjkQuery::Support& a, const PxGjkQuery::Support& b, const PxTransform& poseA, const PxTransform& poseB, PxReal contactDistance, PxReal toleranceLength, PxContactBuffer& contactBuffer) noexcept
{
    return PxGjkQueryExt::generateContacts(a, b, poseA, poseB,
        contactDistance, toleranceLength, contactBuffer);
}