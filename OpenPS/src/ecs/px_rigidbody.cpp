#include <pch.h>
#include <ecs/px_rigidbody.h>
#include <ecs/px_colliders.h>
#include <core/px_physics.h>

openps::rigidbody::rigidbody(uint32_t hndl, rigidbody_type tp) noexcept : handle(hndl), type(tp)
{

}

physx::PxRigidActor* openps::createRigidbodyActor(rigidbody* rb, collider_base* collider, const PxTransform& trs)
{
	if (!rb)
		return nullptr;

	if (!collider)
		return nullptr;

	const auto physics = openps::physics_holder::physicsRef->getPhysicsImpl();

	rb->material = physics->createMaterial(rb->staticFriction, rb->dynamicFriction, rb->restitution);

	if (rb->type == rigidbody_type::Static)
	{
		PxRigidStatic* actor = physics->createRigidStatic(trs);

		collider->createShape();
		uint32_t* h = new uint32_t[1];
		h[0] = rb->handle;
		collider->getShape()->userData = h;
		actor->attachShape(*collider->getShape());
		actor->userData = h;
		rb->useGravity = false;
		rb->actor = actor;

		openps::physics_holder::physicsRef->addActor(rb, rb->actor);

		return actor;
	}
	else
	{
		PxRigidDynamic* actor = physics->createRigidDynamic(trs);
		actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW, true);
		actor->setRigidBodyFlag(PxRigidBodyFlag::eRETAIN_ACCELERATIONS, true);

#if PX_GPU_BROAD_PHASE
		actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, true);
#else
		actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
		actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD_FRICTION, true);
#endif

		collider->createShape();
		uint32_t* h = new uint32_t[1];
		h[0] = rb->handle;
		collider->getShape()->userData = h;
		actor->attachShape(*collider->getShape());
		actor->userData = h;

		rb->actor = actor;

		openps::physics_holder::physicsRef->addActor(rb, rb->actor);

		return actor;
	}
	
	return nullptr;
}
