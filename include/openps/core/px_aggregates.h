#ifndef _OPENPS_AGGREGATES_
#define _OPENPS_AGGREGATES_

#include <core/px_physics.h>

namespace openps
{
	struct px_aggregate
	{
		px_aggregate() = default;

		px_aggregate(uint8_t nb, bool sc = true) noexcept;

		~px_aggregate();

		void addActor(physx::PxActor* actor) noexcept;
		void removeActor(physx::PxActor* actor) noexcept;

		NODISCARD const uint8_t getNbActors() const noexcept { return nbActors; }
		NODISCARD const bool isSelfCollision() const noexcept { return selfCollisions; }

	private:
		physx::PxAggregate* aggregate = nullptr;

		uint8_t nbActors = 0;
		bool selfCollisions = true;
	};
}

#endif