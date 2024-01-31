#include <pch.h>
#include <core/px_wrappers.h>
#include "core/px_physics.h"

void openps::collision_contact_callback::onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs)
{
	UNUSED(pairHeader);

	const physx::PxU32 bufferSize = PX_CONTACT_BUFFER_SIZE;
	physx::PxContactPairPoint contacts[bufferSize];
	for (physx::PxU32 i = 0; i < nbPairs; i++)
	{
		const physx::PxContactPair& cp = pairs[i];
		physx::PxU32 nbContacts = pairs[i].extractContacts(contacts, bufferSize);
		for (physx::PxU32 j = 0; j < nbContacts; j++)
		{
			physx::PxVec3 point = contacts[j].position;
			physx::PxVec3 impulse = contacts[j].impulse;
			physx::PxU32 internalFaceIndex0 = contacts[j].internalFaceIndex0;
			physx::PxU32 internalFaceIndex1 = contacts[j].internalFaceIndex1;
			UNUSED(point);
			UNUSED(impulse);
			UNUSED(internalFaceIndex0);
			UNUSED(internalFaceIndex1);
		}

		if (cp.events & physx::PxPairFlag::eNOTIFY_TOUCH_FOUND)
		{
			auto r1 = pairHeader.actors[0]->is<PxRigidActor>();
			auto r2 = pairHeader.actors[1]->is<PxRigidActor>();
			if (!r1 || !r2)
				return;
			PxRigidActor* actor1 = r1;
			PxRigidActor* actor2 = r2;

			auto rb1 = openps::physics_holder::physicsRef->actorsMap[actor1];
			auto rb2 = openps::physics_holder::physicsRef->actorsMap[actor2];

			if (rb1 && rb2)
			{
				rb1->onCollisionEnter(rb2);
				rb2->onCollisionEnter(rb1);
				openps::physics_holder::physicsRef->collisionQueue.emplace(rb1->handle, rb2->handle);
			}
		}
	}
}

void openps::ccd_contact_modification::onCCDContactModify(PxContactModifyPair* const pairs, PxU32 count)
{
	UNUSED(pairs);

	for (size_t i = 0; i < count; i++)
	{
		physx::PxContactModifyPair& cp = pairs[i];

		PxRigidActor* actor1 = (PxRigidActor*)cp.actor[0];
		PxRigidActor* actor2 = (PxRigidActor*)cp.actor[1];

		auto rb1 = openps::physics_holder::physicsRef->actorsMap[actor1];
		auto rb2 = openps::physics_holder::physicsRef->actorsMap[actor2];

		if (rb1 && rb2)
		{
			rb1->onCollisionEnter(rb2);
			rb2->onCollisionEnter(rb1);
		}
	}
}

physx::PxTriangleMesh* openps::createTriangleMesh(PxTriangleMeshDesc desc)
{
	try
	{
		if (desc.triangles.count > 0 && desc.isValid())
		{
			auto cookingParams = physx::PxCookingParams(physics_holder::physicsRef->toleranceScale);
#if PX_GPU_BROAD_PHASE
			cookingParams.buildGPUData = true;
#endif
			cookingParams.suppressTriangleMeshRemapTable = true;
			cookingParams.midphaseDesc = physx::PxMeshMidPhase::eBVH34;
			cookingParams.meshPreprocessParams = physx::PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
			return PxCreateTriangleMesh(cookingParams, desc);
		}
	}
	catch (...)
	{
		logger::log_error("Physics> Failed to create physics triangle mesh");
	}
	return nullptr;
}
