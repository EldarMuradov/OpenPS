#pragma once
#include <openps.h>

struct px_aggregate
{
	px_aggregate() = default;

	px_aggregate(uint8_t nb, bool sc = true) noexcept : nbActors(nb), selfCollisions(sc)
	{
		aggregate = openps::physics_holder::physicsRef->getPhysicsImpl()->createAggregate(nbActors, selfCollisions, physx::PxAggregateFilterHint());
		openps::physics_holder::physicsRef->addAggregate(aggregate);
	}

	~px_aggregate() { openps::physics_holder::physicsRef->removeAggregate(aggregate); PX_RELEASE(aggregate) }

	void addActor(physx::PxActor* actor) noexcept { aggregate->addActor(*actor); }
	void removeActor(physx::PxActor* actor) noexcept { aggregate->removeActor(*actor); }

	NODISCARD uint8_t getNbActors() const noexcept { return nbActors; }
	NODISCARD bool isSelfCollision() const noexcept { return selfCollisions; }

private:
	physx::PxAggregate* aggregate = nullptr;

	uint8_t nbActors = 0;
	bool selfCollisions = true;
};