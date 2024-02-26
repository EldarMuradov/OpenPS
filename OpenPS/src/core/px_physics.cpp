#include "pch.h"
#include "core/px_physics.h"

namespace openps
{
	std::mutex sync;

	physics* physics_holder::physicsRef;

	static physx::PxFilterFlags contactReportFilterShader(
		PxFilterObjectAttributes attributes0,
		PxFilterData filterData0,
		PxFilterObjectAttributes attributes1,
		PxFilterData filterData1,
		PxPairFlags& pairFlags,
		const void* constantBlock,
		PxU32 constantBlockSize)
	{
		UNUSED(constantBlockSize);
		UNUSED(constantBlock);

		if (physx::PxFilterObjectIsTrigger(attributes0) || physx::PxFilterObjectIsTrigger(attributes1))
		{
			pairFlags = physx::PxPairFlag::eTRIGGER_DEFAULT;
			return physx::PxFilterFlag::eDEFAULT;
		}

		pairFlags |= PxPairFlag::eDETECT_CCD_CONTACT;

		pairFlags = physx::PxPairFlag::eNOTIFY_CONTACT_POINTS | physx::PxPairFlag::eDETECT_DISCRETE_CONTACT;

		if ((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
			pairFlags |= physx::PxPairFlag::eNOTIFY_TOUCH_FOUND;

		pairFlags = physx::PxPairFlag::eSOLVE_CONTACT | physx::PxPairFlag::eDETECT_DISCRETE_CONTACT
			| physx::PxPairFlag::eNOTIFY_TOUCH_FOUND
			| physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS
			| physx::PxPairFlag::eNOTIFY_CONTACT_POINTS | PxPairFlag::eMODIFY_CONTACTS;
		return physx::PxFilterFlag::eNOTIFY;
	}

	physics::physics()
	{
		logger::logErrorFunc = default_log_error;
		logger::logMessageFunc = default_log_message;

		physics_holder::physicsRef = this;

		initialize();
	}

	physics::physics(physics_desc desc)
	{
		if(desc.logErrorFunc)
			logger::logErrorFunc = desc.logErrorFunc;
		if (desc.logMessageFunc)
			logger::logMessageFunc = desc.logMessageFunc;

		physics_holder::physicsRef = this;

		initialize();
	}

	void physics::update(float dt)
	{
		float stepSize = 1.0f / frameRate;

		scene->lockWrite();
		scene->getTaskManager()->startSimulation();

		void* scratchMemBlock = allocator.allocate(MB(16), 16U, true);

#if PX_GPU_BROAD_PHASE
		scene->simulate(stepSize, NULL, scratchMemBlock, MB(16));
#else
		scene->collide(std::max(dt, stepSize), NULL, scratchMemBlock, MB(16));
		scene->fetchCollision(true);
		scene->advance();
#endif

		scene->fetchResults(true);

		scene->flushSimulation();
		scene->getTaskManager()->stopSimulation();
		scene->unlockWrite();

		allocator.reset();
	}
	
	void physics::initialize()
	{
		allocator.initialize(MB(256));

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
		toleranceScale.length = 1.0;
		toleranceScale.speed = 10.0;

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
		sceneDesc.simulationEventCallback = &collisionCallback;

		PxCudaContextManagerDesc cudaContextManagerDesc;

#if PX_GPU_BROAD_PHASE
		sceneDesc.broadPhaseType = physx::PxBroadPhaseType::eGPU;
		sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
		sceneDesc.gpuMaxNumPartitions = 8;
		sceneDesc.gpuDynamicsConfig = PxgDynamicsMemoryConfig();
#else
		sceneDesc.broadPhaseType = physx::PxBroadPhaseType::ePABP;
#endif
		sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
		sceneDesc.flags |= PxSceneFlag::eDISABLE_CCD_RESWEEP;
		sceneDesc.flags |= PxSceneFlag::eREQUIRE_RW_LOCK;
		cudaContextManager = PxCreateCudaContextManager(*foundation, cudaContextManagerDesc, &profilerCallback);
		sceneDesc.cudaContextManager = cudaContextManager;
		sceneDesc.cudaContextManager = cudaContextManager;
		sceneDesc.frictionType = PxFrictionType::eTWO_DIRECTIONAL;
		sceneDesc.flags |= physx::PxSceneFlag::eENABLE_ACTIVE_ACTORS;
		sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
		sceneDesc.staticStructure = PxPruningStructureType::eDYNAMIC_AABB_TREE;
		sceneDesc.solverType = PxSolverType::eTGS;
		sceneDesc.flags |= PxSceneFlag::eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS;
		sceneDesc.flags |= PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
		sceneDesc.flags |= PxSceneFlag::eENABLE_STABILIZATION;
		sceneDesc.filterCallback = &simulationFilterCallback;
		sceneDesc.ccdContactModifyCallback = &contactModification;

		scene = physicsImpl->createScene(sceneDesc);

#if PX_ENABLE_PVD
		PxPvdSceneClient* client = scene->getScenePvdClient();

		if (client)
		{
			client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
			client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
			client->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
		}
#endif

#if PX_ENABLE_PVD
		if (pvd->isConnected())
			logger::log_message("Physics> PVD Connection enabled.");
#endif
	}

	void physics::release()
	{
		PxCloseExtensions();

		PX_RELEASE(physicsImpl)
		PX_RELEASE(pvd)
		PX_RELEASE(foundation)
		PX_RELEASE(scene)
		PX_RELEASE(cudaContextManager)

		allocator.reset(true);
	}
}