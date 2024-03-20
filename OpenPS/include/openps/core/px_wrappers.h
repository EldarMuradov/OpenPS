#pragma once

#include <core/px_logger.h>

namespace openps
{
	using namespace physx;

	struct allocator_callback : PxAllocatorCallback
	{
		void* allocate(size_t size, const char* typeName, const char* filename, int line) override
		{
			ASSERT(size < GB(1));
			return _aligned_malloc(size, 16);
		}

		void deallocate(void* ptr) override
		{
			_aligned_free(ptr);
		}
	};

	struct simulation_filter_callback : PxSimulationFilterCallback
	{
		PxFilterFlags pairFound(PxU64 pairID,
			PxFilterObjectAttributes attributes0, PxFilterData filterData0, const PxActor* a0, const PxShape* s0,
			PxFilterObjectAttributes attributes1, PxFilterData filterData1, const PxActor* a1, const PxShape* s1,
			PxPairFlags& pairFlags) override
		{
			return PxFilterFlags(PxFilterFlag::eDEFAULT);
		};

		void pairLost(PxU64 pairID,
			PxFilterObjectAttributes attributes0, PxFilterData filterData0,
			PxFilterObjectAttributes attributes1, PxFilterData filterData1,
			bool objectRemoved) override
		{

		};

		bool statusChange(PxU64& pairID, PxPairFlags& pairFlags, PxFilterFlags& filterFlags) override
		{
			return false;
		};
	};

	struct query_filter : public PxQueryFilterCallback
	{
		PxQueryHitType::Enum preFilter(const PxFilterData& filterData, const PxShape* shape, const PxRigidActor* actor, PxHitFlags& queryFlags) override
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

		PxQueryHitType::Enum postFilter(const PxFilterData& filterData, const PxQueryHit& hit, const PxShape* shape, const PxRigidActor* actor) override
		{
			return PxQueryHitType::eNONE;
		}
	};

	struct profiler_callback : PxProfilerCallback
	{
		void* zoneStart(const char* eventName, bool detached, uint64_t contextId) override
		{
			// TODO: log start event
			return nullptr;
		}

		void zoneEnd(void* profilerData, const char* eventName, bool detached, uint64_t contextId) override
		{
			// TODO: log end event
		}
	};

	struct error_reporter : PxErrorCallback
	{
		void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line) override
		{
			if (message)
				logger::log_error(message);
			else
				logger::log_error("PhysX Error!");
		}
	};

	struct simulation_event_callback : PxSimulationEventCallback
	{
		void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count) override { /*std::cout << "onConstraintBreak\n";*/ }
		void onWake(physx::PxActor** actors, physx::PxU32 count) override { /*std::cout << "onWake\n";*/ }
		void onSleep(physx::PxActor** actors, physx::PxU32 count) override { /*std::cout << "onSleep\n";*/ }
		void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count) override { /*std::cout << "onTrigger\n";*/ }
		void onAdvance(const physx::PxRigidBody* const* bodyBuffer, const physx::PxTransform* poseBuffer, const physx::PxU32 count) override { /*std::cout << "onAdvance\n";*/ }
		void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs) override;
	};

	PxTriangleMesh* createTriangleMesh(PxTriangleMeshDesc desc);

	struct ccd_contact_modification : PxCCDContactModifyCallback
	{
		void onCCDContactModify(PxContactModifyPair* const pairs, PxU32 count);
	};

	template<typename HitType>
	class DynamicHitBuffer : public PxHitCallback<HitType>
	{
	private:
		uint32_t _count;
		HitType _buffer[PX_CONTACT_BUFFER_SIZE];

	public:
		DynamicHitBuffer()
			: PxHitCallback<HitType>(_buffer, PX_CONTACT_BUFFER_SIZE)
			, _count(0)
		{
		}

	public:
		PX_INLINE PxU32 getNbAnyHits() const
		{
			return getNbTouches();
		}

		PX_INLINE const HitType& getAnyHit(const PxU32 index) const
		{
			PX_ASSERT(index < getNbTouches() + PxU32(this->hasBlock));
			return index < getNbTouches() ? getTouches()[index] : this->block;
		}

		PX_INLINE PxU32 getNbTouches() const
		{
			return _count;
		}

		PX_INLINE const HitType* getTouches() const
		{
			return _buffer;
		}

		PX_INLINE const HitType& getTouch(const PxU32 index) const
		{
			PX_ASSERT(index < getNbTouches());
			return _buffer[index];
		}

		PX_INLINE PxU32 getMaxNbTouches() const
		{
			return PX_CONTACT_BUFFER_SIZE;
		}

	protected:
		PxAgain processTouches(const HitType* buffer, PxU32 nbHits) override
		{
			nbHits = min(nbHits, PX_CONTACT_BUFFER_SIZE - _count);

			for (PxU32 i = 0; i < nbHits; i++)
				_buffer[_count + i] = buffer[i];

			_count += nbHits;
			return true;
		}

		void finalizeQuery() override
		{
			if (this->hasBlock)
			{
				processTouches(&this->block, 1);
			}
		}
	};
}