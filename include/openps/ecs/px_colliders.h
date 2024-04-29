#ifndef _OPENPS_COLLIDERS_
#define _OPENPS_COLLIDERS_

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

	void enableShapeVisualization(PxShape* shape) noexcept;

	void disableShapeVisualization(PxShape* shape) noexcept;

	void enableShapeInContactTests(PxShape* shape) noexcept;

	void disableShapeInContactTests(PxShape* shape) noexcept;

	void enableShapeInSceneQueryTests(PxShape* shape) noexcept;

	void disableShapeInSceneQueryTests(PxShape* shape) noexcept;

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

		PxGeometry* createGeometry() override;

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

		PxGeometry* createGeometry() override;

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

		PxGeometry* createGeometry() override;

		float height{}, radius{};
	};

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

		PxGeometry* createGeometry() override;

		void release() override
		{
			PX_RELEASE(plane)
		}

		PxVec3 position{};
		PxVec3 normal{};

		PxRigidStatic* plane = nullptr;
	};
};

#endif