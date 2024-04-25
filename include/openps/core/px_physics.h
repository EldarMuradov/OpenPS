#pragma once

#include <openps_decl.h>

#include <core/px_logger.h>
#include <core/px_structs.h>
#include <core/px_wrappers.h>

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

	struct physics;

	struct physics_holder
	{
		static inline physics* physicsRef = nullptr;
	};

	inline physx::PxFilterFlags contactReportFilterShader(
		physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
		physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
		physx::PxPairFlags& pairFlags, const void* constantBlock, physx::PxU32 constantBlockSize) noexcept
	{
		UNUSED(constantBlockSize);
		UNUSED(constantBlock);

		if (physx::PxFilterObjectIsTrigger(attributes0) || physx::PxFilterObjectIsTrigger(attributes1))
		{
			pairFlags = physx::PxPairFlag::eTRIGGER_DEFAULT;
			return physx::PxFilterFlag::eDEFAULT;
		}

		pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT;
		pairFlags |= physx::PxPairFlag::eDETECT_CCD_CONTACT;
		pairFlags |= physx::PxPairFlag::eNOTIFY_TOUCH_FOUND;
		pairFlags |= physx::PxPairFlag::eNOTIFY_TOUCH_LOST;
		pairFlags |= physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS;
		pairFlags |= physx::PxPairFlag::ePOST_SOLVER_VELOCITY;
		pairFlags |= physx::PxPairFlag::eNOTIFY_CONTACT_POINTS;
		pairFlags |= physx::PxPairFlag::eSOLVE_CONTACT;
		pairFlags |= physx::PxPairFlag::eDETECT_DISCRETE_CONTACT;

		return physx::PxFilterFlag::eDEFAULT;
	}

	struct physics
	{
		std::set<rigidbody*> actors;
		std::unordered_map<PxRigidActor*, rigidbody*> actorsMap;

		std::queue<collision_handling_data> collisionQueue;
		std::queue<collision_handling_data> collisionExitQueue;

		std::queue<collision_handling_data> triggerQueue;
		std::queue<collision_handling_data> triggerExitQueue;

		// It should be float
		float frameRate = 60.0f;

		PxTolerancesScale toleranceScale;

		physics()
		{
			logger::logErrorFunc = default_log_error;
			logger::logMessageFunc = default_log_message;

			physics_holder::physicsRef = this;

			initialize();
		}

		physics(physics_desc desc)
		{
			if (desc.logErrorFunc)
				logger::logErrorFunc = desc.logErrorFunc;
			if (desc.logMessageFunc)
				logger::logMessageFunc = desc.logMessageFunc;

			physics_holder::physicsRef = this;

			initialize();
		}

		physics(const physics&) = default;
		physics(physics&&) = default;

		virtual ~physics() { release(); }

		void update(float dt)
		{
			const static float stepSize = 1.0f / frameRate;
			static constexpr uint64_t align = 16U;

			const static uint32_t rawMemotySize = 32U;

			scene->lockWrite();

			clearInternalQueues();

			scene->getTaskManager()->startSimulation();

			void* scratchMemBlock = allocator.allocate(MB(rawMemotySize), align, true);

			scene->simulate(stepSize, NULL, scratchMemBlock, MB(rawMemotySize));

			scene->fetchResults(true);

			scene->getTaskManager()->stopSimulation();
			scene->unlockWrite();

			allocator.reset();
		}

		void addAggregate(PxAggregate* aggregate) noexcept
		{
			scene->addAggregate(*aggregate);
		}

		void removeAggregate(PxAggregate* aggregate) noexcept
		{
			scene->removeAggregate(*aggregate);
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

		NODISCARD PxMaterial* getDefaultMaterial() const noexcept { return defaultMaterial; }

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
		void initialize()
		{
			allocator.initialize(MB(256U));

			foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocatorCallback, errorReporter);

			if (!foundation)
				throw std::exception("Failed to create {PxFoundation}. Error in {PhysicsEngine} ctor.");

			pvd = PxCreatePvd(*foundation);

#if PX_ENABLE_PVD
			PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate("localhost", 5425, 10);

			if (transport == NULL)
				throw std::exception("Failed to create {PxPvdTransport}. Error in {PhysicsEngine} ctor.");

			pvd->connect(*transport, PxPvdInstrumentationFlag::eALL);
#endif
			toleranceScale.length = 1.0f;
			toleranceScale.speed = 9.81f;

			physicsImpl = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, toleranceScale, true, pvd);

			if (!PxInitExtensions(*physicsImpl, pvd))
				logger::log_error("Physics> Failed to initialize extensions.");

			dispatcher = PxDefaultCpuDispatcherCreate(nbCPUDispatcherThreads);

			PxSceneDesc sceneDesc(toleranceScale);
			sceneDesc.gravity = gravity;
			sceneDesc.cpuDispatcher = dispatcher;
			sceneDesc.filterShader = contactReportFilterShader;
			sceneDesc.kineKineFilteringMode = physx::PxPairFilteringMode::eKEEP;
			sceneDesc.staticKineFilteringMode = physx::PxPairFilteringMode::eKEEP;
			sceneDesc.simulationEventCallback = &simulationCallback;

			PxCudaContextManagerDesc cudaContextManagerDesc;

			cudaContextManager = PxCreateCudaContextManager(*foundation, cudaContextManagerDesc, &profilerCallback);
			sceneDesc.cudaContextManager = cudaContextManager;

#if PX_GPU_BROAD_PHASE
			sceneDesc.broadPhaseType = physx::PxBroadPhaseType::eGPU;
			sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
			sceneDesc.gpuMaxNumPartitions = 8;
			sceneDesc.gpuDynamicsConfig = PxgDynamicsMemoryConfig();
			sceneDesc.flags |= PxSceneFlag::eREQUIRE_RW_LOCK;
#else
			sceneDesc.broadPhaseType = physx::PxBroadPhaseType::ePABP;
#endif
			sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
			sceneDesc.flags |= PxSceneFlag::eDISABLE_CCD_RESWEEP;

			sceneDesc.frictionType = PxFrictionType::ePATCH;
			sceneDesc.flags |= physx::PxSceneFlag::eENABLE_ACTIVE_ACTORS;
			sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;

			sceneDesc.solverType = PxSolverType::eTGS;

			sceneDesc.flags |= PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
			sceneDesc.flags |= PxSceneFlag::eENABLE_STABILIZATION;
			sceneDesc.filterCallback = &simulationFilterCallback;

			scene = physicsImpl->createScene(sceneDesc);

			defaultMaterial = physicsImpl->createMaterial(0.6f, 0.6f, 0.8f);

#if PX_ENABLE_PVD
			PxPvdSceneClient* client = scene->getScenePvdClient();

			if (client)
			{
				client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
				client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
				client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
			}

			if (pvd->isConnected())
				logger::log_message("Physics> PVD Connection enabled.");
#endif
		}

		void release()
		{
			PxCloseExtensions();

			PX_RELEASE(physicsImpl)
			PX_RELEASE(pvd)
			PX_RELEASE(foundation)
			PX_RELEASE(scene)
			PX_RELEASE(defaultMaterial)
			PX_RELEASE(cudaContextManager)

			allocator.reset(true);
		}

		void processSimulationEventCallbacks() noexcept
		{
			simulationCallback.sendCollisionEvents();
			simulationCallback.sendTriggerEvents();
			simulationCallback.clear();
		}

		void clearInternalQueues() noexcept
		{
			while (collisionQueue.size())
				collisionQueue.pop();

			while (collisionExitQueue.size())
				collisionExitQueue.pop();

			while (triggerQueue.size())
				triggerQueue.pop();

			while (triggerExitQueue.size())
				triggerExitQueue.pop();
		}

	private:
		PxScene* scene = nullptr;

		PxPhysics* physicsImpl = nullptr;

		PxMaterial* defaultMaterial = nullptr;

		PxPvd* pvd = nullptr;

		PxCudaContextManager* cudaContextManager = nullptr;

		PxFoundation* foundation = nullptr;

		PxDefaultCpuDispatcher* dispatcher = nullptr;

		PxDefaultAllocator defaultAllocatorCallback;

		allocator_callback allocatorCallback;

		error_reporter errorReporter;

		profiler_callback profilerCallback;

		query_filter queryFilter;
		uint32_t nbCPUDispatcherThreads = 4;

		simulation_filter_callback simulationFilterCallback;
		simulation_event_callback simulationCallback;

		eallocator allocator;
	};
}