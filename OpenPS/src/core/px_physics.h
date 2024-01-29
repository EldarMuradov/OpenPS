#pragma once

#include <core/px_logger.h>
#include <core/px_wrappers.h>

namespace openps
{
	using namespace physx;

	struct physics_desc
	{
		log_message_func_ptr logMessageFunc;
		log_error_func_ptr logErrorFunc;
	};

	struct physics
	{
		physics();
		physics(physics_desc desc);
		physics(const physics&) = default;
		physics(physics&&) = default;
		virtual ~physics() {}

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

		profiler_callback profiler;

		query_filter queryFilter;
		simulation_filter_callback simulationFilterCcallback;

		PxTolerancesScale toleranceScale;

		const uint32_t nbCPUDispatcherThreads = 4;
	};
}