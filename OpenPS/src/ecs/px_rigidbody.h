#pragma once

namespace openps
{
	struct rigidbody;
	using on_collision_enter_rb_func_ptr = void(*)(rigidbody*);

	enum class rigidbody_type : uint8_t
	{
		None,
		Static,
		Dynamic,
		Kinematic
	};

	enum class force_mode : uint8_t
	{
		None,
		Force,
		Impulse
	};

	using namespace physx;

	struct rigidbody
	{
		// TODO
		const PxVec3 getPosition() const noexcept { return {}; }
		const PxQuat getRotation() const noexcept { return {}; }

		void onCollisionEnter(rigidbody* collision) const
		{
			if (onCollisionFunc)
				onCollisionFunc(collision);
		}

		on_collision_enter_rb_func_ptr onCollisionFunc = nullptr;

		uint32_t handle{};

	private:
		PxMaterial* material = nullptr;

		PxRigidActor* actor = nullptr;

		float restitution = 0.6f;

		uint32_t mass = 1;

		float dynamicFriction = 0.8f;
		float staticFriction = 0.8f;

		physx::PxU32 filterGroup = -1;
		physx::PxU32 filterMask = -1;

		physx::PxRigidDynamicLockFlags rotLockNative;
		physx::PxRigidDynamicLockFlags posLockNative;

		bool useGravity = true;

		rigidbody_type type = rigidbody_type::None;
	};
}