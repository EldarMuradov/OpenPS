#pragma once

#include <openps_decl.h>

namespace openps
{
	using namespace physx;

	enum class collider_type : uint8_t
	{
		None,
		Box,
		Sphere,
		Capsule,
		TriangleMesh,
		ConvexMesh,
		Plane,
		BoundingBox
	};

	template<typename T_>
	using VBT_ = std::enable_if_t<sizeof(T_) == sizeof(physx::PxVec3), bool>;

	template<typename VBT, typename IBT, VBT_<VBT> = true>
	struct triangle_mesh_descriptor
	{
		VBT* vertexBuffer = nullptr;
		size_t vertexBufferStride = sizeof(VBT);
		size_t vertexBufferSize{};

		IBT* indexBuffer = nullptr;
		size_t indexBufferSize{};
	};

	template<typename VBT, VBT_<VBT> = true>
	struct convex_mesh_descriptor
	{
		VBT* vertexBuffer = nullptr;
		size_t vertexBufferStride = sizeof(VBT);
		size_t vertexBufferSize{};
	};

	template<>
	struct convex_mesh_descriptor<PxVec3>
	{
		PxVec3* vertexBuffer = nullptr;
		size_t vertexBufferStride = sizeof(PxVec3);
		size_t vertexBufferSize{};
	};

	template<>
	struct triangle_mesh_descriptor<PxVec3, PxU32>
	{
		PxVec3* vertexBuffer{};
		size_t vertexBufferStride = sizeof(PxVec3);
		size_t vertexBufferSize{};

		PxU32* indexBuffer{};
		size_t indexBufferSize{};
	};

	struct px_triangle_mesh_collider_builder
	{
		template<typename VBT, typename IBT, VBT_<VBT> = true>
		NODISCARD PxTriangleMesh* buildMesh(triangle_mesh_descriptor<VBT, IBT> desc) { return nullptr; }
	};

	struct px_convex_mesh_collider_builder
	{
		template<typename VBT, VBT_<VBT> = true>
		NODISCARD PxConvexMesh* buildMesh(convex_mesh_descriptor<VBT> desc) { return nullptr; }
	};

	inline void enableShapeVisualization(PxShape* shape) noexcept
	{
		shape->setFlag(PxShapeFlag::eVISUALIZATION, true);
	}

	inline void disableShapeVisualization(PxShape* shape) noexcept
	{
		shape->setFlag(PxShapeFlag::eVISUALIZATION, false);
	}

	inline void enableShapeInContactTests(PxShape* shape) noexcept
	{
		shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
	}

	inline void disableShapeInContactTests(PxShape* shape) noexcept
	{
		shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
	}

	inline void enableShapeInSceneQueryTests(PxShape* shape) noexcept
	{
		shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, true);
	}

	inline void disableShapeInSceneQueryTests(PxShape* shape) noexcept
	{
		shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);
	}

	struct bounding_box
	{
		PxVec3 minCorner;
		PxVec3 maxCorner;
	};

	template<typename T_>
	using ColliderType = std::enable_if_t<std::is_base_of_v<struct collider_base, T_>, bool>;

	struct collider_base
	{
		collider_base() = default;
		collider_base(collider_type collType) noexcept : type(collType) {};

		virtual ~collider_base() {};

		NODISCARD PxGeometry* getGeometry() noexcept { return geometry; }

		virtual void release() { RELEASE_PTR(geometry) }

		NODISCARD collider_type getType() const noexcept { return type; }

		virtual PxGeometry* createGeometry() = 0;

		template<typename T, ColliderType<T> = true>
		NODISCARD T* is()
		{
			return static_cast<const T*>(this);
		}

	protected:
		collider_type type = collider_type::None;

		PxGeometry* geometry = nullptr;
	};

	struct box_collider : collider_base
	{
		box_collider() = default;
		box_collider(float x, float y, float z) noexcept
		{
			type = collider_type::Box;
			this->x = x;
			this->y = y;
			this->z = z;
		};

		virtual ~box_collider() {}

		PxGeometry* createGeometry() override
		{
			geometry = new PxBoxGeometry(x, y, z);

			return geometry;
		}

		float x{}, y{}, z{};
	};

	struct sphere_collider : collider_base
	{
		sphere_collider() = default;
		sphere_collider(float r) noexcept
		{
			radius = r;
			type = collider_type::Sphere;
		};

		virtual ~sphere_collider() {}

		PxGeometry* createGeometry() override
		{
			geometry = new PxSphereGeometry(radius);

			return geometry;
		}

		float radius{};
	};

	struct capsule_collider: collider_base
	{
		capsule_collider() = default;
		capsule_collider(float r, float h) noexcept
		{
			type = collider_type::Capsule;
			radius = r;
			height = h;
		};

		virtual ~capsule_collider() {}

		PxGeometry* createGeometry() override
		{
			geometry = new PxCapsuleGeometry(radius, height / 2.0f);

			return geometry;
		}

		float height{}, radius{};
	};

	inline NODISCARD openps::bounding_box calculateBoundingBox(const std::vector<physx::PxVec3>& positions)
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

	inline void createMeshFromBoundingBox(const openps::bounding_box& box, std::vector<physx::PxVec3>& vertices, std::vector<uint32_t>& indices)
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

	struct bounding_box_collider : collider_base
	{
		bounding_box_collider() = default;
		bounding_box_collider(const bounding_box& b) noexcept : box(b)
		{
			type = collider_type::BoundingBox;
		};

		virtual ~bounding_box_collider() {}

		PxGeometry* createGeometry() override { return nullptr; }

		bounding_box box{};
	};

	template<typename VBT, typename IBT, VBT_<VBT> = true>
	struct triangle_mesh_collider : collider_base
	{
		triangle_mesh_collider() = default;
		triangle_mesh_collider(VBT size, triangle_mesh_descriptor<VBT, IBT> desc) noexcept : descriptor(desc), modelSize(size)
		{
			type = collider_type::TriangleMesh;
		};

		virtual ~triangle_mesh_collider() {}

		PxGeometry* createGeometry() override { return nullptr; }

		void release() override
		{
			RELEASE_PTR(descriptor.vertexBuffer)
			RELEASE_PTR(descriptor.indexBuffer)
		}

		triangle_mesh_descriptor<VBT, IBT> descriptor{};

		VBT modelSize{};
	};

	template<typename VBT, VBT_<VBT> = true>
	struct convex_mesh_collider : collider_base
	{
		convex_mesh_collider() = default;
		convex_mesh_collider(VBT size, convex_mesh_descriptor<VBT> desc) noexcept : descriptor(desc), modelSize(size)
		{
			type = collider_type::ConvexMesh;
		};

		virtual ~convex_mesh_collider() {}

		PxGeometry* createGeometry() override { return nullptr; }

		void release() override
		{
			RELEASE_PTR(descriptor.vertexBuffer)
			RELEASE_PTR(descriptor.indexBuffer)
		}

		convex_mesh_descriptor<VBT> descriptor{};

		VBT modelSize{};
	};

	struct plane_collider : collider_base
	{
		plane_collider() = default;
		plane_collider(const PxVec3& pos, const PxVec3& norm = PxVec3(0.f, 1.f, 0.f)) noexcept : position(pos), normal(norm)
		{
			type = collider_type::Plane;
		}

		~plane_collider() {}

		bool createShape();

		PxGeometry* createGeometry() override { return nullptr; };

		void release() override
		{
			PX_RELEASE(plane)
		}

		PxVec3 position{};
		PxVec3 normal{};

		PxRigidStatic* plane = nullptr;
	};
};