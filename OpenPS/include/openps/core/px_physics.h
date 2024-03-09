#pragma once

#include <core/px_logger.h>
#include <core/px_wrappers.h>
#include <core/px_structs.h>
#include <ecs/px_rigidbody.h>
#include <memory/ememory.h>

namespace openps
{
	struct rigidbody;

	using namespace physx;

	struct physics_desc
	{
		log_message_func_ptr logMessageFunc;
		log_error_func_ptr logErrorFunc;
	};


	struct collision_handling_data
	{
		uint32_t id1;
		uint32_t id2;
	};

	struct physics
	{
		std::set<rigidbody*> actors;
		std::unordered_map<PxRigidActor*, rigidbody*> actorsMap;
		std::queue<collision_handling_data> collisionQueue;

		// It should be float
		float frameRate = 60.0f;

		PxTolerancesScale toleranceScale;

		physics();
		physics(physics_desc desc);
		physics(const physics&) = default;
		physics(physics&&) = default;
		virtual ~physics() { release(); }

		void update(float dt);

		void addAggregate(PxAggregate* aggregate) noexcept
		{
		
		}

		void removeAggregate(PxAggregate* aggregate) noexcept
		{
		
		}

		void addActor(rigidbody* actor, PxRigidActor* ractor, bool addToScene = true)
		{
			scene->lockWrite();
			if (addToScene)
				scene->addActor(*ractor);

			actors.emplace(actor);
			actorsMap.insert(std::make_pair(ractor, actor));
			scene->unlockWrite();
		}

		void addActor(PxRigidActor* actor)
		{
			scene->lockWrite();
			scene->addActor(*actor);
			scene->unlockWrite();
		}

		void removeActor(rigidbody* actor)
		{
			scene->lockWrite();
			actors.erase(actor);
			actorsMap.erase(actor->getRigidActor());
			scene->removeActor(*actor->getRigidActor());
			scene->unlockWrite();
		}

		void reomoveActor(PxRigidActor* actor)
		{
			scene->lockWrite();
			scene->removeActor(*actor);
			scene->unlockWrite();
		}

		void lockRead() { scene->lockRead(); }
		void unlockRead() { scene->unlockRead(); }

		void lockWrite() { scene->lockWrite(); }
		void unlockWrite() { scene->unlockWrite(); }

		NODISCARD PxPhysics* getPhysicsImpl() const noexcept { return physicsImpl; }

		const raycast_info raycast(rigidbody* rb, const PxVec3& dir, int maxDist = PX_NB_MAX_RAYCAST_DISTANCE, bool hitTriggers = true, uint32_t layerMask = 0, int maxHits = PX_NB_MAX_RAYCAST_HITS) noexcept
		{
			PX_SCENE_QUERY_SETUP(true);
			PxRaycastBuffer buffer;
			bool status = scene->raycast(rb->getPosition(), dir, maxDist, buffer, hitFlags, PxQueryFilterData(), &queryFilter);

			if (status)
			{
				uint32_t nb = buffer.getNbAnyHits();
				uint32_t index = 0;
				if (nb > 1)
					index = 1;
				const auto& hitInfo1 = buffer.getAnyHit(index);

				auto actor = actorsMap[hitInfo1.actor];

				if (actor != rb)
					return
				{
					actor,
					hitInfo1.distance,
					buffer.getNbAnyHits(),
					PxVec3(hitInfo1.position.x, hitInfo1.position.y, hitInfo1.position.z)
				};
				else if (buffer.getNbAnyHits() > 1)
				{
					const auto& hitInfo2 = buffer.getAnyHit(1);

					actor = actorsMap[hitInfo2.actor];

					return
					{
						actor,
						hitInfo2.distance,
						buffer.getNbAnyHits(),
						PxVec3(hitInfo2.position.x, hitInfo2.position.y, hitInfo2.position.z)
					};
				}
			}

			return raycast_info();
		}

		// Checking
		const bool checkBox(const PxVec3& center, const PxVec3& halfExtents, const PxQuat& rotation, bool hitTriggers = false, uint32_t layerMask = 0) noexcept
		{
			PX_SCENE_QUERY_SETUP_CHECK();
			std::vector<uint32_t*> results;
			const PxTransform pose(center, rotation);
			const PxBoxGeometry geometry(halfExtents);

			return scene->overlap(geometry, pose, buffer, filterData, &queryFilter);
		}

		const bool checkSphere(const PxVec3& center, const float radius, bool hitTriggers = false, uint32_t layerMask = 0) noexcept
		{
			PX_SCENE_QUERY_SETUP_CHECK();
			std::vector<uint32_t*> results;
			const PxTransform pose(center);
			const PxSphereGeometry geometry(radius);

			return scene->overlap(geometry, pose, buffer, filterData, &queryFilter);
		}

		const bool checkCapsule(const PxVec3& center, const float radius, const float halfHeight, const PxQuat& rotation, bool hitTriggers = false, uint32_t layerMask = 0) noexcept
		{
			PX_SCENE_QUERY_SETUP_CHECK();
			std::vector<uint32_t*> results;
			const PxTransform pose(center, rotation);
			const PxCapsuleGeometry geometry(radius, halfHeight);
			return scene->overlap(geometry, pose, buffer, filterData, &queryFilter);
		}

		// Overlapping
		const overlap_info overlapCapsule(const PxVec3& center, const float radius, const float halfHeight, const PxQuat& rotation, bool hitTriggers = false, uint32_t layerMask = 0) noexcept
		{
			PX_SCENE_QUERY_SETUP_OVERLAP();
			std::vector<uint32_t*> results;
			const PxTransform pose(center, rotation);
			const PxCapsuleGeometry geometry(radius, halfHeight);
			if (!scene->overlap(geometry, pose, buffer, filterData, &queryFilter))
				return overlap_info(false, results);

			PX_SCENE_QUERY_COLLECT_OVERLAP();

			return overlap_info(true, results);
		}

		const overlap_info overlapBox(const PxVec3& center, const PxVec3& halfExtents, const PxQuat& rotation, bool hitTriggers = false, uint32_t layerMask = 0) noexcept
		{
			PX_SCENE_QUERY_SETUP_OVERLAP();
			std::vector<uint32_t*> results;
			const PxTransform pose(center, rotation);
			const PxBoxGeometry geometry(halfExtents);

			if (!scene->overlap(geometry, pose, buffer, filterData, &queryFilter))
				return overlap_info(false, results);

			PX_SCENE_QUERY_COLLECT_OVERLAP();

			return overlap_info(true, results);
		}

		const overlap_info overlapSphere(const PxVec3& center, const float radius, bool hitTriggers = false, uint32_t layerMask = 0) noexcept
		{
			PX_SCENE_QUERY_SETUP_OVERLAP();
			std::vector<uint32_t*> results;
			const PxTransform pose(center);
			const PxSphereGeometry geometry(radius);

			if (!scene->overlap(geometry, pose, buffer, filterData, &queryFilter))
				return overlap_info(false, results);

			PX_SCENE_QUERY_COLLECT_OVERLAP();

			return overlap_info(true, results);
		}

	private:
		void initialize();
		void release();

	private:
		PxScene* scene = nullptr;

		PxPhysics* physicsImpl = nullptr;

		PxPvd* pvd = nullptr;

		PxCudaContextManager* cudaContextManager = nullptr;

		PxFoundation* foundation = nullptr;

		PxDefaultCpuDispatcher* dispatcher = nullptr;

		PxDefaultAllocator defaultAllocatorCallback;

		allocator_callback allocatorCallback;

		error_reporter errorReporter;

		profiler_callback profilerCallback;

		query_filter queryFilter;
		simulation_filter_callback simulationFilterCallback;
		collision_contact_callback collisionCallback;
		ccd_contact_modification contactModification;

		eallocator allocator;

		uint32_t nbCPUDispatcherThreads = 4;
	};

	struct physics_holder 
	{
		static physics* physicsRef;
	};
}