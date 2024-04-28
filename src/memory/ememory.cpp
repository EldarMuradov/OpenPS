#include <memory/ememory.h>

void openps::eallocator::initialize(uint64_t minimumBlockSize, uint64_t reserveSize) noexcept
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

void openps::eallocator::ensureFreeSize(uint64_t size) noexcept
{
	::std::unique_lock<::std::mutex> lock{ mutex };
	ensureFreeSizeInternal(size);
}

NODISCARD void* openps::eallocator::allocate(uint64_t size, uint64_t alignment, bool clearToZero) noexcept
{
	if (size == 0)
		return 0;

	::std::unique_lock<::std::mutex> lock{ mutex };

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

	if (clearToZero)
		memset(result, 0, size);

	return result;
}

void openps::eallocator::setCurrentTo(void* ptr) noexcept
{
	current = (uint8_t*)ptr - memory;
	sizeLeftCurrent = committedMemory - current;
	sizeLeftTotal = reserveSize - current;
}

void openps::eallocator::reset(bool freeMemory) noexcept
{
	if (memory && freeMemory)
	{
		VirtualFree(memory, 0, MEM_RELEASE);
		memory = 0;
		committedMemory = 0;
	}

	resetToMarker(memory_marker{ 0 });
}

void openps::eallocator::resetToMarker(memory_marker marker) noexcept
{
	current = marker.before;
	sizeLeftCurrent = committedMemory - current;
	sizeLeftTotal = reserveSize - current;
}

void openps::eallocator::ensureFreeSizeInternal(uint64_t size) noexcept
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