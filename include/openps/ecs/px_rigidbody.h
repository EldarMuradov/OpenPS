#ifndef _OPENPS_RIGIDBODY_
#define _OPENPS_RIGIDBODY_

#include <openps_decl.h>
#include <core/px_logger.h>
#include <ecs/px_colliders.h>

namespace openps
{
	struct rigidbody;
	struct collider_base;

	using on_collision_enter_rb_func_ptr = void(*)(rigidbody*);
	using on_collision_exit_rb_func_ptr = void(*)(rigidbody*);
	using on_collision_stay_rb_func_ptr = void(*)(rigidbody*);

	using on_trigger_enter_rb_func_ptr = void(*)(rigidbody*);
	using on_trigger_exit_rb_func_ptr = void(*)(rigidbody*);
	using on_trigger_stay_rb_func_ptr = void(*)(rigidbody*);

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
		rigidbody() = default;
		rigidbody(uint32_t hndl, rigidbody_type tp) noexcept : handle(hndl), type(tp) { }

		NODISCARD const PxVec3 getPosition() const noexcept;

		void setPosition(const PxVec3& pos) noexcept;
		void setPosition(PxVec3&& pos) noexcept;

		NODISCARD const PxQuat getRotation() const noexcept;

		void setRotation(const PxQuat& rot) noexcept;
		void setRotation(PxQuat&& rot) noexcept;

		NODISCARD const float getMass() const noexcept { return mass; }

		NODISCARD PxRigidActor* getRigidActor() const noexcept { return actor; }

		void setMass(float newMass) noexcept;

		void onCollisionExit(rigidbody* collision) const noexcept;

		void onCollisionStay(rigidbody* collision) const noexcept;

		void onCollisionEnter(rigidbody* collision) const noexcept;

		void onTriggerExit(rigidbody* trigger) const noexcept;

		void onTriggerStay(rigidbody* trigger) const noexcept;

		void onTriggerEnter(rigidbody* trigger) const noexcept;

		on_collision_enter_rb_func_ptr onCollisionEnterFunc = nullptr;
		on_collision_exit_rb_func_ptr onCollisionExitFunc = nullptr;
		on_collision_stay_rb_func_ptr onCollisionStayFunc = nullptr;

		on_trigger_enter_rb_func_ptr onTriggerEnterFunc = nullptr;
		on_trigger_exit_rb_func_ptr onTriggerExitFunc = nullptr;
		on_trigger_stay_rb_func_ptr onTriggerStayFunc = nullptr;

		uint32_t handle{};

	private:
		float mass = 1.0f;
		float restitution = 0.6f;

		float dynamicFriction = 0.8f;
		float staticFriction = 0.8f;

		PxU32 filterGroup = -1;
		PxU32 filterMask = -1;

		PxRigidDynamicLockFlags rotLockNative;
		PxRigidDynamicLockFlags posLockNative;

		rigidbody_type type = rigidbody_type::None;

		bool useGravity = true;

		PxMaterial* material = nullptr;

		PxRigidActor* actor = nullptr;

	private:
		friend PxRigidActor* createRigidbodyActor(rigidbody* rb, collider_base* collider, const PxTransform& trs) noexcept;
	};
}

#endif