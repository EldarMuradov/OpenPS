#include <pch.h>
#include "memory/ememory.h"

namespace openps
{
	void eallocator::initialize(uint64_t minimumBlockSize, uint64_t reserveSize)
	{
		reset(true);

		memory = (uint8_t*)VirtualAlloc(0, reserveSize, MEM_RESERVE, PAGE_READWRITE);

		SYSTEM_INFO systemInfo;
		GetSystemInfo(&systemInfo);

		pageSize = systemInfo.dwPageSize;
		sizeLeftTotal = reserveSize;
		this->minimumBlockSize = minimumBlockSize;
		this->reserveSize = reserveSize;
	}

	void eallocator::ensureFreeSize(uint64_t size)
	{
		mutex.lock();
		ensureFreeSizeInternal(size);
		mutex.unlock();
	}

	void eallocator::ensureFreeSizeInternal(uint64_t size)
	{
		if (sizeLeftCurrent < size)
		{
			uint64_t allocationSize = max(size, minimumBlockSize);
			allocationSize = pageSize * bucketize(allocationSize, pageSize);
			VirtualAlloc(memory + committedMemory, allocationSize, MEM_COMMIT, PAGE_READWRITE);

			sizeLeftTotal += allocationSize;
			sizeLeftCurrent += allocationSize;
			committedMemory += allocationSize;
		}
	}

	NODISCARD void* eallocator::allocate(uint64_t size, uint64_t alignment, bool clearToZero)
	{
		if (size == 0)
			return 0;

		mutex.lock();

		uint64_t mask = alignment - 1;
		uint64_t misalignment = current & mask;
		uint64_t adjustment = (misalignment == 0) ? 0 : (alignment - misalignment);
		current += adjustment;

		sizeLeftCurrent -= adjustment;
		sizeLeftTotal -= adjustment;

		ASSERT(sizeLeftTotal >= size);

		ensureFreeSizeInternal(size);

		uint8_t* result = memory + current;
		current += size;
		sizeLeftCurrent -= size;
		sizeLeftTotal -= size;

		mutex.unlock();

		if (clearToZero)
			memset(result, 0, size);

		return result;
	}

	NODISCARD void* eallocator::getCurrent(uint64_t alignment)
	{
		return memory + alignTo(current, alignment);
	}

	void eallocator::setCurrentTo(void* ptr)
	{
		current = (uint8_t*)ptr - memory;
		sizeLeftCurrent = committedMemory - current;
		sizeLeftTotal = reserveSize - current;
	}

	void eallocator::reset(bool freeMemory)
	{
		if (memory && freeMemory)
		{
			VirtualFree(memory, 0, MEM_RELEASE);
			memory = 0;
			committedMemory = 0;
		}

		resetToMarker(memory_marker{ 0 });
	}

	NODISCARD memory_marker eallocator::getMarker()
	{
		return { current };
	}

	void eallocator::resetToMarker(memory_marker marker)
	{
		current = marker.before;
		sizeLeftCurrent = committedMemory - current;
		sizeLeftTotal = reserveSize - current;
	}
}