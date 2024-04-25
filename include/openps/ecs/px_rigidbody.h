#pragma once

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

		NODISCARD const PxVec3 getPosition() const noexcept { return actor->getGlobalPose().p; }
		void setPosition(const PxVec3& pos) noexcept { actor->setGlobalPose(PxTransform(pos, getRotation())); }
		void setPosition(PxVec3&& pos) noexcept { actor->setGlobalPose(PxTransform(pos, getRotation())); }

		NODISCARD const PxQuat getRotation() const noexcept { return actor->getGlobalPose().q; }
		void setRotation(const PxQuat& rot) noexcept { actor->setGlobalPose(PxTransform(getPosition(), rot)); }
		void setRotation(PxQuat&& rot) noexcept { actor->setGlobalPose(PxTransform(getPosition(), rot)); }

		NODISCARD const float getMass() const noexcept { return mass; }

		NODISCARD PxRigidActor* getRigidActor() const noexcept { return actor; }

		void setMass(float newMass) noexcept
		{
			mass = newMass;
			actor->is<PxRigidDynamic>()->setMass(mass);
		}

		void onCollisionExit(rigidbody* collision) const
		{
			openps::logger::log_message("collision enter");
			if (onCollisionExitFunc)
				onCollisionExitFunc(collision);
		}

		void onCollisionStay(rigidbody* collision) const
		{
			if (onCollisionStayFunc)
				onCollisionStayFunc(collision);
		}

		void onCollisionEnter(rigidbody* collision) const
		{
			openps::logger::log_message("collision enter");
			if (onCollisionEnterFunc)
				onCollisionEnterFunc(collision);
		}

		void onTriggerExit(rigidbody* trigger) const
		{
			openps::logger::log_message("trigger exit");
			if (onTriggerExitFunc)
				onTriggerExitFunc(trigger);
		}

		void onTriggerStay(rigidbody* trigger) const
		{
			if (onTriggerStayFunc)
				onTriggerStayFunc(trigger);
		}

		void onTriggerEnter(rigidbody* trigger) const
		{
			openps::logger::log_message("trigger exit");
			if (onTriggerEnterFunc)
				onTriggerEnterFunc(trigger);
		}

		on_collision_enter_rb_func_ptr onCollisionEnterFunc = nullptr;
		on_collision_exit_rb_func_ptr onCollisionExitFunc = nullptr;
		on_collision_stay_rb_func_ptr onCollisionStayFunc = nullptr;

		on_trigger_enter_rb_func_ptr onTriggerEnterFunc = nullptr;
		on_trigger_exit_rb_func_ptr onTriggerExitFunc = nullptr;
		on_trigger_stay_rb_func_ptr onTriggerStayFunc = nullptr;

		uint32_t handle{};

	private:
		PxMaterial* material = nullptr;

		PxRigidActor* actor = nullptr;

		float restitution = 0.6f;

		float mass = 1;

		float dynamicFriction = 0.8f;
		float staticFriction = 0.8f;

		PxU32 filterGroup = -1;
		PxU32 filterMask = -1;

		PxRigidDynamicLockFlags rotLockNative;
		PxRigidDynamicLockFlags posLockNative;

		bool useGravity = true;

		rigidbody_type type = rigidbody_type::None;

	private:
		friend PxRigidActor* createRigidbodyActor(rigidbody* rb, collider_base* collider, const PxTransform& trs);
	};
}