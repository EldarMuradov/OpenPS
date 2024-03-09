#include <pch.h>
#include <openps.h>

#define DEFAULT_SDF_AND_RST 0.5f, 0.5f, 0.6f

static NODISCARD openps::bounding_box calculateBoundingBox(const std::vector<physx::PxVec3>& positions)
{
	openps::bounding_box box;
	if (positions.empty())
		return box;

	box.minCorner = positions[0];
	box.maxCorner = positions[0];

	for (const auto& position : positions)
	{
		box.minCorner = physx::min(box.minCorner, position);
		box.maxCorner = physx::max(box.maxCorner, position);
	}

	return box;
}

static void createMeshFromBoundingBox(const openps::bounding_box& box, std::vector<physx::PxVec3>& vertices, std::vector<uint32_t>& indices)
{
	physx::PxVec3 p0 = box.minCorner;
	physx::PxVec3 p1 = physx::PxVec3(box.maxCorner.x, box.minCorner.y, box.minCorner.z);
	physx::PxVec3 p2 = physx::PxVec3(box.maxCorner.x, box.minCorner.y, box.maxCorner.z);
	physx::PxVec3 p3 = physx::PxVec3(box.minCorner.x, box.minCorner.y, box.maxCorner.z);
	physx::PxVec3 p4 = physx::PxVec3(box.minCorner.x, box.maxCorner.y, box.minCorner.z);
	physx::PxVec3 p5 = physx::PxVec3(box.maxCorner.x, box.maxCorner.y, box.minCorner.z);
	physx::PxVec3 p6 = box.maxCorner;
	physx::PxVec3 p7 = physx::PxVec3(box.minCorner.x, box.maxCorner.y, box.maxCorner.z);

	vertices.push_back({ p0 });
	vertices.push_back({ p1 });
	vertices.push_back({ p2 });
	vertices.push_back({ p3 });
	vertices.push_back({ p4 });
	vertices.push_back({ p5 });
	vertices.push_back({ p6 });
	vertices.push_back({ p7 });

	uint32_t boxIndices[36] =
	{
		0, 1, 2,
		2, 3, 0,
		4, 5, 6,
		6, 7, 4,
		0, 1, 5,
		5, 4, 0,
		2, 3, 7,
		7, 6, 2,
		0, 4, 7,
		7, 3, 0,
		1, 5, 6,
		6, 2, 1
	};

	for (int i = 0; i < 36; i++)
		indices.push_back(boxIndices[i]);
}

void openps::enableShapeInContactTests(physx::PxShape* shape) noexcept
{
	shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
}

void openps::disableShapeInContactTests(physx::PxShape* shape) noexcept
{
	shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
}

void openps::enableShapeInSceneQueryTests(physx::PxShape* shape) noexcept
{
	shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, true);
}

void openps::disableShapeInSceneQueryTests(physx::PxShape* shape) noexcept
{
	shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);
}

void openps::collider_base::release()
{
	PX_RELEASE(shape);
	PX_RELEASE(material)
}

openps::box_collider::~box_collider()
{
}

bool openps::box_collider::createShape()
{
	const auto physics = openps::physics_holder::physicsRef->getPhysicsImpl();

	material = physics->createMaterial(DEFAULT_SDF_AND_RST);
	shape = physics->createShape(physx::PxBoxGeometry(x, y, z), *material);

	enableShapeInSceneQueryTests(shape);
	enableShapeInContactTests(shape);

	return true;
}

openps::sphere_collider::~sphere_collider()
{
}

bool openps::sphere_collider::createShape()
{
	const auto physics = openps::physics_holder::physicsRef->getPhysicsImpl();

	material = physics->createMaterial(DEFAULT_SDF_AND_RST);
	shape = physics->createShape(physx::PxSphereGeometry(radius), *material);

	enableShapeInSceneQueryTests(shape);
	enableShapeInContactTests(shape);

	return true;
}

openps::capsule_collider::~capsule_collider()
{
}

bool openps::capsule_collider::createShape()
{
	const auto physics = openps::physics_holder::physicsRef->getPhysicsImpl();

	material = physics->createMaterial(DEFAULT_SDF_AND_RST);
	shape = physics->createShape(physx::PxCapsuleGeometry(radius, height / 2.0f), *material);
	
	enableShapeInSceneQueryTests(shape);
	enableShapeInContactTests(shape);

	return true;
}

openps::bounding_box_collider::~bounding_box_collider()
{
}

bool openps::bounding_box_collider::createShape()
{
	return false;
}

bool openps::plane_collider::createShape()
{
	material = physics_holder::physicsRef->getPhysicsImpl()->createMaterial(0.5f, 0.5f, 0.6f);
	plane = PxCreatePlane(*physics_holder::physicsRef->getPhysicsImpl(), PxPlane(position, normal), *material);
	physics_holder::physicsRef->addActor(plane);

	return true;
}
