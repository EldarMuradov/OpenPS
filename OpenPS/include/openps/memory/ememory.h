#pragma once

NODISCARD inline uint32_t alignTo(uint32_t currentOffset, uint32_t alignment)
{
	uint32_t mask = alignment - 1;
	uint32_t misalignment = currentOffset & mask;
	uint32_t adjustment = (misalignment == 0) ? 0 : (alignment - misalignment);
	return currentOffset + adjustment;
}

NODISCARD inline uint64_t alignTo(uint64_t currentOffset, uint64_t alignment)
{
	uint64_t mask = alignment - 1;
	uint64_t misalignment = currentOffset & mask;
	uint64_t adjustment = (misalignment == 0) ? 0 : (alignment - misalignment);
	return currentOffset + adjustment;
}

NODISCARD inline void* alignTo(void* currentAddress, uint64_t alignment)
{
	uint64_t mask = alignment - 1;
	uint64_t misalignment = (uint64_t)(currentAddress)&mask;
	uint64_t adjustment = (misalignment == 0) ? 0 : (alignment - misalignment);
	return (uint8_t*)currentAddress + adjustment;
}

inline bool rangesOverlap(uint64_t fromA, uint64_t toA, uint64_t fromB, uint64_t toB)
{
	if (toA <= fromB || fromA >= toA)
		return false;
	return true;
}

inline bool rangesOverlap(void* fromA, void* toA, void* fromB, void* toB)
{
	if (toA <= fromB || fromA >= toB)
		return false;
	return true;
}

namespace openps 
{
	struct memory_marker
	{
		uint64_t before;
	};

	struct eallocator
	{
	protected:
		uint8_t* memory = 0;
		uint64_t committedMemory = 0;

		uint64_t current = 0;
		uint64_t sizeLeftCurrent = 0;

		uint64_t sizeLeftTotal = 0;

		uint64_t pageSize = 0;
		uint64_t minimumBlockSize = 0;

		uint64_t reserveSize = 0;

		std::mutex mutex;

	public:
		eallocator() {}
		eallocator(const eallocator&) = delete;
		eallocator(eallocator&&) = default;
		~eallocator() { reset(true); }

		void initialize(uint64_t minimumBlockSize = 0, uint64_t reserveSize = GB(8));

		void ensureFreeSize(uint64_t size);

		NODISCARD void* allocate(uint64_t size, uint64_t alignment = 1, bool clearToZero = false);

		template <typename T>
		NODISCARD T* allocate(uint32_t count = 1, bool clearToZero = false)
		{
			return (T*)allocate(sizeof(T) * count, alignof(T), clearToZero);
		}

		NODISCARD void* getCurrent(uint64_t alignment = 1);

		template <typename T>
		NODISCARD T* getCurrent()
		{
			return (T*)getCurrent(alignof(T));
		}

		void setCurrentTo(void* ptr);

		void reset(bool freeMemory = false);

		NODISCARD memory_marker getMarker();
		void resetToMarker(memory_marker marker);

		NODISCARD uint8_t* base() { return memory; }

	protected:
		void ensureFreeSizeInternal(uint64_t size);
	};
}