#pragma once

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

	template<typename VBT, typename IBT>
	struct triangle_mesh_descriptor
	{
		VBT* vertexBuffer = nullptr;
		size_t vertexBufferStride = sizeof(VBT);
		size_t vertexBufferSize{};

		IBT* indexBuffer = nullptr;
		size_t indexBufferSize{};
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
		template<typename VBT, typename IBT>
		NODISCARD PxTriangleMesh* createMeshShape(triangle_mesh_descriptor<VBT, IBT> desc) { return nullptr; }
	};

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

		NODISCARD PxShape* getShape() const noexcept { return shape; }
		void setShape(PxShape* newShape) noexcept { shape = newShape; }

		void release();

		NODISCARD collider_type getType() const noexcept { return type; }

		virtual bool createShape() = 0;

		template<typename T, ColliderType<T> = true>
		NODISCARD T* is()
		{
			return static_cast<const T*>(this);
		}

	protected:
		collider_type type = collider_type::None;

		PxShape* shape = nullptr;
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

		virtual ~box_collider();

		bool createShape() override;

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

		virtual ~sphere_collider();

		bool createShape() override;

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

		virtual ~capsule_collider();

		bool createShape() override;

		float height{}, radius{};
	};

	struct bounding_box_collider : collider_base
	{
		bounding_box_collider() = default;
		bounding_box_collider(const bounding_box& b) noexcept : box(b)
		{
			type = collider_type::BoundingBox;
		};

		virtual ~bounding_box_collider();

		bool createShape() override;

		bounding_box box{};
	};

	template<typename VBT, typename IBT>
	struct triangle_mesh_collider : collider_base
	{
		triangle_mesh_collider() = default;
		triangle_mesh_collider(float size, triangle_mesh_descriptor<VBT, IBT> desc) noexcept : descriptor(desc), modelSize(size)
		{
			type = collider_type::TriangleMesh;
		};

		virtual ~triangle_mesh_collider() {}

		bool createShape() override { return false; }

		triangle_mesh_descriptor<VBT, IBT> descriptor{};

		float modelSize{};
	};
};