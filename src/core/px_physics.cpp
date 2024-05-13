#include <core/px_physics.h>

namespace openps
{
	static physx::PxFilterFlags contactReportFilterShader(
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
}

openps::physics::physics() noexcept
{
	logger::logErrorFunc = default_log_error;
	logger::logMessageFunc = default_log_message;

	physics_holder::physicsRef = this;

	initialize();
}

openps::physics::physics(const physics_desc& desc) noexcept
{
	if (desc.logErrorFunc)
		logger::logErrorFunc = desc.logErrorFunc;
	if (desc.logMessageFunc)
		logger::logMessageFunc = desc.logMessageFunc;

	physics_holder::physicsRef = this;

	initialize();
}

void openps::physics::update(float dt)
{
	const static float stepSize = 1.0f / (float)frameRate;
	static constexpr uint64_t align = 16U;

	static constexpr uint32_t rawMemotySize = 32U;

	static constexpr uint32_t scratchMemBlockSize = (uint32_t)MB(rawMemotySize);

	physics_lock_write lock{};

	clearInternalQueues();

	scene->getTaskManager()->startSimulation();

	void* scratchMemBlock = allocator.allocate(scratchMemBlockSize, align, true);

	scene->simulate(stepSize, NULL, scratchMemBlock, scratchMemBlockSize);

	scene->fetchResults(true);

	scene->getTaskManager()->stopSimulation();

	allocator.reset();
}

void openps::physics::addAggregate(PxAggregate* aggregate) noexcept
{
	physics_lock_write lock{};
	scene->addAggregate(*aggregate);
}

void openps::physics::removeAggregate(PxAggregate* aggregate) noexcept
{
	physics_lock_write lock{};
	scene->removeAggregate(*aggregate);
}

void openps::physics::addActor(rigidbody* actor, PxRigidActor* ractor, bool addToScene) noexcept
{
	physics_lock_write lock{};
	if (addToScene)
		scene->addActor(*ractor);

	actors.emplace(actor);
	actorsMap.insert(std::make_pair(ractor, actor));
}

void openps::physics::addActor(PxRigidActor* actor) noexcept
{
	physics_lock_write lock{};
	scene->addActor(*actor);
}

void openps::physics::removeActor(rigidbody* actor) noexcept
{
	physics_lock_write lock{};
	actors.erase(actor);
	actorsMap.erase(actor->getRigidActor());
	scene->removeActor(*actor->getRigidActor());
}

void openps::physics::reomoveActor(PxRigidActor* actor) noexcept
{
	physics_lock_write lock{};
	scene->removeActor(*actor);
}

void openps::physics::lockRead() noexcept
{
	scene->lockRead();
}

void openps::physics::unlockRead() noexcept
{
	scene->unlockRead();
}

void openps::physics::lockWrite() noexcept
{
	scene->lockWrite();
}

void openps::physics::unlockWrite() noexcept
{
	scene->unlockWrite();
}

const openps::raycast_info openps::physics::raycast(rigidbody* rb, const PxVec3& dir, float maxDist, bool hitTriggers, uint32_t layerMask) noexcept
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

const bool openps::physics::checkBox(const PxVec3& center, const PxVec3& halfExtents, const PxQuat& rotation, bool hitTriggers, uint32_t layerMask) noexcept
{
	PX_SCENE_QUERY_SETUP_CHECK();
	std::vector<uint32_t*> results;
	const PxTransform pose(center, rotation);
	const PxBoxGeometry geometry(halfExtents);

	return scene->overlap(geometry, pose, buffer, filterData, &queryFilter);
}

const bool openps::physics::checkSphere(const PxVec3& center, const float radius, bool hitTriggers, uint32_t layerMask) noexcept
{
	PX_SCENE_QUERY_SETUP_CHECK();
	std::vector<uint32_t*> results;
	const PxTransform pose(center);
	const PxSphereGeometry geometry(radius);

	return scene->overlap(geometry, pose, buffer, filterData, &queryFilter);
}

const bool openps::physics::checkCapsule(const PxVec3& center, const float radius, const float halfHeight, const PxQuat& rotation, bool hitTriggers, uint32_t layerMask) noexcept
{
	PX_SCENE_QUERY_SETUP_CHECK();
	std::vector<uint32_t*> results;
	const PxTransform pose(center, rotation);
	const PxCapsuleGeometry geometry(radius, halfHeight);
	return scene->overlap(geometry, pose, buffer, filterData, &queryFilter);
}

const openps::overlap_info openps::physics::overlapCapsule(const PxVec3& center, const float radius, const float halfHeight, const PxQuat& rotation, bool hitTriggers, uint32_t layerMask) noexcept
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

const openps::overlap_info openps::physics::overlapBox(const PxVec3& center, const PxVec3& halfExtents, const PxQuat& rotation, bool hitTriggers, uint32_t layerMask) noexcept
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

const openps::overlap_info openps::physics::overlapSphere(const PxVec3& center, const float radius, bool hitTriggers, uint32_t layerMask) noexcept
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

void openps::physics::initialize() noexcept
{
	allocator.initialize(MB(256U));

	foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocatorCallback, errorReporter);

	if (!foundation)
	{
		logger::log_error("Physics> Failed to initialize foundation.");
		return;
	}

	pvd = PxCreatePvd(*foundation);

	if (!pvd)
	{
		logger::log_error("Physics> Failed to initialize PVD.");
		return;
	}

#if PX_ENABLE_PVD
	PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate("localhost", 5425, 10);

	if (!transport)
	{
		logger::log_error("Physics> Failed to initialize PxPvdTransport.");
		return;
	}

	pvd->connect(*transport, PxPvdInstrumentationFlag::eALL);
#endif

	toleranceScale.length = 1.0f;
	toleranceScale.speed = 9.81f;

	physicsImpl = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, toleranceScale, true, pvd);

	if (!physicsImpl)
	{
		logger::log_error("Physics> Failed to initialize PxPhysics.");
		return;
	}

	if (!PxInitExtensions(*physicsImpl, pvd))
	{
		logger::log_error("Physics> Failed to initialize extensions.");
		return;
	}

	dispatcher = PxDefaultCpuDispatcherCreate(nbCPUDispatcherThreads);

	if (!dispatcher)
	{
		logger::log_error("Physics> Failed to initialize PxDefaultCpuDispatcher.");
		return;
	}

	PxCudaContextManagerDesc cudaContextManagerDesc;

	cudaContextManager = PxCreateCudaContextManager(*foundation, cudaContextManagerDesc, &profilerCallback);

	if (!cudaContextManager)
	{
		logger::log_error("Physics> Failed to initialize PxCudaContextManager.");
		return;
	}

	PxSceneDesc sceneDesc(toleranceScale);
	sceneDesc.gravity = gravity;
	sceneDesc.cpuDispatcher = dispatcher;
	sceneDesc.filterShader = contactReportFilterShader;
	//sceneDesc.kineKineFilteringMode = physx::PxPairFilteringMode::eKEEP;
	//sceneDesc.staticKineFilteringMode = physx::PxPairFilteringMode::eKEEP;
	sceneDesc.simulationEventCallback = &simulationCallback;

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

	sceneDesc.solverType = PxSolverType::eTGS;
	sceneDesc.frictionType = PxFrictionType::ePATCH;

	sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
	sceneDesc.flags |= PxSceneFlag::eDISABLE_CCD_RESWEEP;
	sceneDesc.flags |= physx::PxSceneFlag::eENABLE_ACTIVE_ACTORS;
	sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
	sceneDesc.flags |= PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
	sceneDesc.flags |= PxSceneFlag::eENABLE_STABILIZATION;

	sceneDesc.filterCallback = &simulationFilterCallback;

	scene = physicsImpl->createScene(sceneDesc);

	if (!scene)
	{
		logger::log_error("Physics> Failed to initialize PxScene.");
		return;
	}

	defaultMaterial = physicsImpl->createMaterial(0.6f, 0.6f, 0.8f);

	if (!defaultMaterial)
	{
		logger::log_error("Physics> Failed to initialize PxMaterial.");
		return;
	}

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

void openps::physics::release() noexcept
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

void openps::physics::processSimulationEventCallbacks() noexcept
{
	simulationCallback.sendCollisionEvents();
	simulationCallback.sendTriggerEvents();
	simulationCallback.clear();
}

void openps::physics::clearInternalQueues() noexcept
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