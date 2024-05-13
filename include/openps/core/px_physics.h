#ifndef _OPENPS_PHYSICS_
#define _OPENPS_PHYSICS_

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

	struct physics
	{
		std::set<rigidbody*> actors;
		std::unordered_map<PxRigidActor*, rigidbody*> actorsMap;

		std::queue<collision_handling_data> collisionQueue;
		std::queue<collision_handling_data> collisionExitQueue;

		std::queue<collision_handling_data> triggerQueue;
		std::queue<collision_handling_data> triggerExitQueue;

		uint32_t frameRate = 60U;

		PxTolerancesScale toleranceScale;

		physics() noexcept;

		physics(const physics_desc& desc) noexcept;

		physics(const physics&) = default;
		physics(physics&&) = default;

		virtual ~physics() { release(); }

		void update(float dt);

		void addAggregate(PxAggregate* aggregate) noexcept;

		void removeAggregate(PxAggregate* aggregate) noexcept;

		void addActor(rigidbody* actor, PxRigidActor* ractor, bool addToScene = true) noexcept;

		void addActor(PxRigidActor* actor) noexcept;

		void removeActor(rigidbody* actor) noexcept;

		void reomoveActor(PxRigidActor* actor) noexcept;

		void lockRead() noexcept;
		void unlockRead() noexcept;

		void lockWrite() noexcept;
		void unlockWrite() noexcept;

		NODISCARD PxPhysics* getPhysicsImpl() const noexcept { return physicsImpl; }

		NODISCARD PxMaterial* getDefaultMaterial() const noexcept { return defaultMaterial; }

		const raycast_info raycast(rigidbody* rb, const PxVec3& dir, float maxDist = PX_NB_MAX_RAYCAST_DISTANCE, bool hitTriggers = true, uint32_t layerMask = 0) noexcept;

		// Checking
		const bool checkBox(const PxVec3& center, const PxVec3& halfExtents, const PxQuat& rotation, bool hitTriggers = false, uint32_t layerMask = 0) noexcept;

		const bool checkSphere(const PxVec3& center, const float radius, bool hitTriggers = false, uint32_t layerMask = 0) noexcept;

		const bool checkCapsule(const PxVec3& center, const float radius, const float halfHeight, const PxQuat& rotation, bool hitTriggers = false, uint32_t layerMask = 0) noexcept;

		// Overlapping
		const overlap_info overlapCapsule(const PxVec3& center, const float radius, const float halfHeight, const PxQuat& rotation, bool hitTriggers = false, uint32_t layerMask = 0) noexcept;

		const overlap_info overlapBox(const PxVec3& center, const PxVec3& halfExtents, const PxQuat& rotation, bool hitTriggers = false, uint32_t layerMask = 0) noexcept;

		const overlap_info overlapSphere(const PxVec3& center, const float radius, bool hitTriggers = false, uint32_t layerMask = 0) noexcept;

	private:
		void initialize() noexcept;

		void release() noexcept;

		void processSimulationEventCallbacks() noexcept;

		void clearInternalQueues() noexcept;

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

		simulation_filter_callback simulationFilterCallback;
		simulation_event_callback simulationCallback;

		uint32_t nbCPUDispatcherThreads = 4U;

		eallocator allocator;
	};

	struct physics_lock
	{
		virtual void lock() noexcept = 0;
		virtual void unlock() noexcept = 0;

		virtual ~physics_lock() {}
	};

	struct physics_lock_read : physics_lock
	{
		physics_lock_read() noexcept
		{
			lock();
		}

		virtual ~physics_lock_read()
		{
			unlock();
		}

		virtual void lock() noexcept
		{
			physics_holder::physicsRef->lockRead();
		}

		virtual void unlock() noexcept
		{
			physics_holder::physicsRef->unlockRead();
		}
	};

	struct physics_lock_write : physics_lock
	{
		physics_lock_write() noexcept
		{
			lock();
		}

		virtual ~physics_lock_write()
		{
			unlock();
		}

		virtual void lock() noexcept
		{
			physics_holder::physicsRef->lockWrite();
		}

		virtual void unlock() noexcept
		{
			physics_holder::physicsRef->unlockWrite();
		}
	};
}

#endif