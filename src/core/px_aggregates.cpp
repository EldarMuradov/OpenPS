#include <core/px_aggregates.h>

openps::px_aggregate::px_aggregate(uint8_t nb, bool sc) noexcept : nbActors(nb), selfCollisions(sc)
{
	aggregate = openps::physics_holder::physicsRef->getPhysicsImpl()->createAggregate(nbActors, selfCollisions, physx::PxAggregateFilterHint());
	openps::physics_holder::physicsRef->addAggregate(aggregate);
}

openps::px_aggregate::~px_aggregate()
{
	openps::physics_holder::physicsRef->removeAggregate(aggregate);
	PX_RELEASE(aggregate)
}

void openps::px_aggregate::addActor(physx::PxActor* actor) noexcept
{
	aggregate->addActor(*actor);
}

void openps::px_aggregate::removeActor(physx::PxActor* actor) noexcept
{
	aggregate->removeActor(*actor);
}