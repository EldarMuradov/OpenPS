#pragma once

namespace openps
{
	struct rigidbody;

	struct raycast_info
	{
		ref<rigidbody> actor;

		float distance = 0.0f;
		unsigned int hitCount = 0;
		physx::PxVec3 position = physx::PxVec3(0.0f);
	};

	struct overlap_info
	{
		bool isOverlapping = false;
		std::vector<uint32_t*> results{};
	};
}