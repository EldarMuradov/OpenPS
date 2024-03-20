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
	return !(toA <= fromB || fromA >= toA);
}

inline bool rangesOverlap(void* fromA, void* toA, void* fromB, void* toB)
{
	return !(toA <= fromB || fromA >= toB);
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

		NODISCARD void* getCurrent(uint64_t alignment = 1)  const noexcept
		{
			memory + alignTo(current, alignment);
		}

		template <typename T>
		NODISCARD T* getCurrent() const noexcept
		{
			return (T*)getCurrent(alignof(T));
		}

		void setCurrentTo(void* ptr) noexcept
		{
			current = (uint8_t*)ptr - memory;
			sizeLeftCurrent = committedMemory - current;
			sizeLeftTotal = reserveSize - current;
		}

		void reset(bool freeMemory = false);

		NODISCARD memory_marker getMarker() const noexcept { return { current }; }

		void resetToMarker(memory_marker marker) noexcept
		{
			current = marker.before;
			sizeLeftCurrent = committedMemory - current;
			sizeLeftTotal = reserveSize - current;
		}

		NODISCARD uint8_t* base() const noexcept { return memory; }

	protected:
		void ensureFreeSizeInternal(uint64_t size);
	};
}