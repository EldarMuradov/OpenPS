#include <ecs/px_rigidbody.h>
#include <core/px_physics.h>

NODISCARD const physx::PxVec3 openps::rigidbody::getPosition() const noexcept
{
	physics_lock_read lock{};
	return actor->getGlobalPose().p;
}

void openps::rigidbody::setPosition(const PxVec3& pos) noexcept
{
	physics_lock_write lock{};
	actor->setGlobalPose(PxTransform(pos, getRotation()));
}

void openps::rigidbody::setPosition(PxVec3&& pos) noexcept
{
	physics_lock_write lock{};
	actor->setGlobalPose(PxTransform(pos, getRotation()));
}

NODISCARD const physx::PxQuat openps::rigidbody::getRotation() const noexcept
{
	physics_lock_read lock{};
	return actor->getGlobalPose().q;
}

void openps::rigidbody::setRotation(const PxQuat& rot) noexcept
{
	physics_lock_write lock{};
	actor->setGlobalPose(PxTransform(getPosition(), rot));
}

void openps::rigidbody::setRotation(PxQuat&& rot) noexcept
{
	physics_lock_write lock{};
	actor->setGlobalPose(PxTransform(getPosition(), rot));
}

void openps::rigidbody::setMass(float newMass) noexcept
{
	if(auto dyn = actor->is<PxRigidDynamic>())
	{
		physics_lock_write lock{};
		mass = newMass;
		dyn->setMass(mass);
	}
}

void openps::rigidbody::onCollisionExit(rigidbody* collision) const noexcept
{
	openps::logger::log_message("collision exit");
	if (onCollisionExitFunc)
		onCollisionExitFunc(collision);
}

void openps::rigidbody::onCollisionStay(rigidbody* collision) const noexcept
{
	if (onCollisionStayFunc)
		onCollisionStayFunc(collision);
}

void openps::rigidbody::onCollisionEnter(rigidbody* collision) const noexcept
{
	openps::logger::log_message("collision enter");
	if (onCollisionEnterFunc)
		onCollisionEnterFunc(collision);
}

void openps::rigidbody::onTriggerExit(rigidbody* trigger) const noexcept
{
	openps::logger::log_message("trigger exit");
	if (onTriggerExitFunc)
		onTriggerExitFunc(trigger);
}

void openps::rigidbody::onTriggerStay(rigidbody* trigger) const noexcept
{
	if (onTriggerStayFunc)
		onTriggerStayFunc(trigger);
}

void openps::rigidbody::onTriggerEnter(rigidbody* trigger) const noexcept
{
	openps::logger::log_message("trigger enter");
	if (onTriggerEnterFunc)
		onTriggerEnterFunc(trigger);
}
