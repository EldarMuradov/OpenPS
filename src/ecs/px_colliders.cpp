#include <ecs/px_colliders.h>
#include <core/px_physics.h>

bool openps::plane_collider::createShape()
{
	auto physics = openps::physics_holder::physicsRef;
	plane = PxCreatePlane(*physics->getPhysicsImpl(), PxPlane(position, normal), *physics->getDefaultMaterial());
	physics->addActor(plane);

	return true;
}
