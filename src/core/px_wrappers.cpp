#include <core/px_wrappers.h>
#include <core/px_physics.h>

static void clearColliderFromCollection(const openps::rigidbody* collider,
	physx::PxArray<openps::simulation_event_callback::colliders_pair>& collection) noexcept
{
	const auto c = &collection[0];
	for (uint32_t i = 0; i < collection.size(); ++i)
	{
		const openps::simulation_event_callback::colliders_pair cc = c[i];
		if (cc.first == collider || cc.second == collider)
		{
			collection.remove(i--);
			if (collection.empty())
				break;
		}
	}
}

void openps::simulation_event_callback::clear() noexcept
{
	newCollisions.clear();
	removedCollisions.clear();
	kinematicsToRemoveFlag.clear();

	newTriggerPairs.clear();
	lostTriggerPairs.clear();
}

void openps::simulation_event_callback::sendCollisionEvents()
{
	for (auto& c : removedCollisions)
	{
		c.thisActor->onCollisionExit(c.otherActor);
		c.swapObjects();
		c.thisActor->onCollisionExit(c.otherActor);
		c.swapObjects();
		openps::physics_holder::physicsRef->collisionExitQueue.emplace(c.thisActor->handle, c.otherActor->handle);
	}

	for (auto& c : newCollisions)
	{
		c.thisActor->onCollisionEnter(c.otherActor);
		c.swapObjects();
		c.thisActor->onCollisionEnter(c.otherActor);
		c.swapObjects();
		openps::physics_holder::physicsRef->collisionQueue.emplace(c.thisActor->handle, c.otherActor->handle);
	}
}

void openps::simulation_event_callback::sendTriggerEvents()
{
	for (auto& c : lostTriggerPairs)
	{
		c.first->onTriggerExit(c.second);
		c.second->onTriggerExit(c.first);
		openps::physics_holder::physicsRef->triggerExitQueue.emplace(c.first->handle, c.second->handle);
	}

	for (auto& c : newTriggerPairs)
	{
		c.first->onTriggerEnter(c.second);
		c.second->onTriggerEnter(c.first);
		openps::physics_holder::physicsRef->triggerQueue.emplace(c.first->handle, c.second->handle);
	}
}

void openps::simulation_event_callback::onColliderRemoved(rigidbody* collider)
{
	clearColliderFromCollection(collider, newTriggerPairs);
	clearColliderFromCollection(collider, lostTriggerPairs);
}

void openps::simulation_event_callback::onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs)
{
	const physx::PxU32 bufferSize = PX_CONTACT_BUFFER_SIZE;
	physx::PxContactPairPoint contacts[bufferSize];

	collision collision{};
	PxContactPairExtraDataIterator iter(pairHeader.extraDataStream, pairHeader.extraDataStreamSize);

	for (physx::PxU32 i = 0; i < nbPairs; ++i)
	{
		const physx::PxContactPair& cp = pairs[i];
		physx::PxU32 nbContacts = pairs[i].extractContacts(contacts, bufferSize);

		const bool hasPostVelocities = !cp.flags.isSet(PxContactPairFlag::eACTOR_PAIR_LOST_TOUCH);

		for (physx::PxU32 j = 0; j < nbContacts; ++j)
		{
			physx::PxVec3 point = contacts[j].position;
			physx::PxVec3 impulse = contacts[j].impulse;
			physx::PxU32 internalFaceIndex0 = contacts[j].internalFaceIndex0;
			physx::PxU32 internalFaceIndex1 = contacts[j].internalFaceIndex1;

			collision.impulse += impulse;

			//UNUSED(impulse);
			UNUSED(point);
			UNUSED(internalFaceIndex0);
			UNUSED(internalFaceIndex1);
		}

		collision.thisVelocity = collision.otherVelocity = PxVec3(0.0f);

		if (hasPostVelocities && iter.nextItemSet())
		{
			if (iter.contactPairIndex != i)
				continue;
			if (iter.postSolverVelocity)
			{
				collision.thisVelocity = iter.postSolverVelocity->linearVelocity[0];
				collision.otherVelocity = iter.postSolverVelocity->linearVelocity[1];
			}
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

			if (!rb1 || !rb2)
				return;

			collision.thisActor = rb1;
			collision.otherActor = rb2;

			if (cp.flags & PxContactPairFlag::eACTOR_PAIR_HAS_FIRST_TOUCH)
			{
				newCollisions.pushBack(collision);
			}
		}
		else if (cp.events & physx::PxPairFlag::eNOTIFY_TOUCH_LOST)
		{
			auto r1 = pairHeader.actors[0]->is<PxRigidActor>();
			auto r2 = pairHeader.actors[1]->is<PxRigidActor>();

			if (!r1 || !r2)
				return;

			PxRigidActor* actor1 = r1;
			PxRigidActor* actor2 = r2;

			auto rb1 = openps::physics_holder::physicsRef->actorsMap[actor1];
			auto rb2 = openps::physics_holder::physicsRef->actorsMap[actor2];

			if (!rb1 || !rb2)
				return;

			collision.thisActor = rb1;
			collision.otherActor = rb2;

			if (cp.flags & PxContactPairFlag::eACTOR_PAIR_LOST_TOUCH)
			{
				removedCollisions.pushBack(collision);
			}
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

physx::PxRigidActor* openps::createRigidbodyActor(rigidbody* rb, collider_base* collider, const PxTransform& trs) noexcept
{
	if (!rb || !collider)
		return nullptr;

	const auto physics = openps::physics_holder::physicsRef->getPhysicsImpl();

	rb->material = physics->createMaterial(rb->staticFriction, rb->dynamicFriction, rb->restitution);

	if (rb->type == rigidbody_type::Static)
	{
		PxRigidStatic* actor = physics->createRigidStatic(trs);

		PxShape* shape = PxRigidActorExt::createExclusiveShape(*actor, *collider->createGeometry(), *rb->material);

		uint32_t* h = new uint32_t[1];
		h[0] = (uint32_t)rb->handle;

		actor->userData = h;
		shape->userData = h;

		rb->actor = actor;

		openps::physics_holder::physicsRef->addActor(rb, rb->actor);

		return actor;
	}
	else
	{
		PxRigidDynamic* actor = physics->createRigidDynamic(trs);
		actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW, true);
		actor->setRigidBodyFlag(PxRigidBodyFlag::eRETAIN_ACCELERATIONS, true);
		actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
		actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, true);
		actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD_FRICTION, true);

		PxShape* shape = PxRigidActorExt::createExclusiveShape(*actor, *collider->createGeometry(), *rb->material);

		uint32_t* h = new uint32_t[1];
		h[0] = (uint32_t)rb->handle;

		actor->userData = h;
		shape->userData = h;

		rb->actor = actor;

		openps::physics_holder::physicsRef->addActor(rb, rb->actor);

		return actor;
	}

	return nullptr;
}

void openps::collision::swapObjects() noexcept
{
	if (!thisActor || !otherActor)
		return;
	::std::swap(thisActor, otherActor);
	::std::swap(thisVelocity, otherVelocity);
}

void openps::error_reporter::reportError(PxErrorCode::Enum code, const char* message, const char* file, int line)
{
	if (message)
	{
		std::stringstream stream{};
		stream << message << " in file: " << file << " in line: " << line;
		logger::log_error(stream.str().c_str());
	}
	else
		logger::log_error("PhysX Error!");
}

physx::PxQueryHitType::Enum openps::query_filter::preFilter(const PxFilterData& filterData, const PxShape* shape, const PxRigidActor* actor, PxHitFlags& queryFlags)
{
	if (!shape)
		return PxQueryHitType::eNONE;

	const PxFilterData shapeFilter = shape->getQueryFilterData();
	if ((filterData.word0 & shapeFilter.word0) == 0)
		return PxQueryHitType::eNONE;

	const bool hitTriggers = filterData.word2 != 0;
	if (!hitTriggers && shape->getFlags() & PxShapeFlag::eTRIGGER_SHAPE)
		return PxQueryHitType::eNONE;

	const bool blockSingle = filterData.word1 != 0;
	return blockSingle ? PxQueryHitType::eBLOCK : PxQueryHitType::eTOUCH;
}

physx::PxQueryHitType::Enum openps::query_filter::postFilter(const PxFilterData& filterData, const PxQueryHit& hit, const PxShape* shape, const PxRigidActor* actor)
{
	return PxQueryHitType::eNONE;
}

void* openps::allocator_callback::allocate(size_t size, const char* typeName, const char* filename, int line)
{
	ASSERT(size < GB(1));
	return _aligned_malloc(size, 16);
}

void openps::allocator_callback::deallocate(void* ptr)
{
	_aligned_free(ptr);
}