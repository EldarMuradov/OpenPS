#ifndef _OPENPS_STRUCTS_
#define _OPENPS_STRUCTS_

#include <openps_decl.h>

namespace openps
{
	struct rigidbody;

	struct raycast_info
	{
		rigidbody* actor = nullptr;

		float distance = 0.0f;
		uint32_t hitCount = 0;
		physx::PxVec3 position = physx::PxVec3(0.0f);
	};

	struct overlap_info
	{
		bool isOverlapping = false;
		std::vector<uint32_t*> results{};
	};
}

#endif