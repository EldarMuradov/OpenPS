#include <ecs/px_colliders.h>
#include <core/px_physics.h>

namespace openps
{
	static NODISCARD openps::bounding_box calculateBoundingBox(const std::vector<physx::PxVec3>& positions) noexcept
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

	static void createMeshFromBoundingBox(const openps::bounding_box& box, std::vector<physx::PxVec3>& vertices, std::vector<uint32_t>& indices) noexcept
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

	void enableShapeVisualization(PxShape* shape) noexcept
	{
		shape->setFlag(PxShapeFlag::eVISUALIZATION, true);
	}

	void disableShapeVisualization(PxShape* shape) noexcept
	{
		shape->setFlag(PxShapeFlag::eVISUALIZATION, false);
	}

	void enableShapeInContactTests(PxShape* shape) noexcept
	{
		shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
	}

	void disableShapeInContactTests(PxShape* shape) noexcept
	{
		shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
	}

	void enableShapeInSceneQueryTests(PxShape* shape) noexcept
	{
		shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, true);
	}

	void disableShapeInSceneQueryTests(PxShape* shape) noexcept
	{
		shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);
	}

	PxGeometry* box_collider::createGeometry()
	{
		geometry = new PxBoxGeometry(x, y, z);

		return geometry;
	}

	PxGeometry* sphere_collider::createGeometry()
	{
		geometry = new PxSphereGeometry(radius);

		return geometry;
	}

	PxGeometry* capsule_collider::createGeometry()
	{
		geometry = new PxCapsuleGeometry(radius, height / 2.0f);

		return geometry;
	}

	PxGeometry* plane_collider::createGeometry()
	{
		geometry = new PxPlaneGeometry();

		return geometry;
	}

	bool plane_collider::createShape()
	{
		auto physics = physics_holder::physicsRef;
		plane = PxCreatePlane(*physics->getPhysicsImpl(), PxPlane(position, normal), *physics->getDefaultMaterial());
		physics->addActor(plane);

		return true;
	}
}