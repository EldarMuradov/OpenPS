#pragma once

#define PX_PHYSX_STATIC_LIB

#define PX_GPU_BROAD_PHASE 1

#define PX_CONTACT_BUFFER_SIZE 64
#define PX_NB_MAX_RAYCAST_HITS 64
#define PX_NB_MAX_RAYCAST_DISTANCE 128

#define PX_ENABLE_RAYCAST_CCD 0

#define NODISCARD [[nodiscard]]

#if _DEBUG
#define PX_ENABLE_PVD 1
#else
#define PX_ENABLE_PVD 0
#endif

#define PX_RELEASE(x)	if(x)	{ x->release(); x = nullptr;}
#define UNUSED(x) (void)(x)

#define RELEASE_PTR(ptr) if(ptr) { delete ptr; ptr = nullptr; }
#define RELEASE_ARRAY_PTR(arrayPtr) if(arrayPtr) { delete[] arrayPtr; arrayPtr = nullptr; } 

#define ASSERT(cond) \
	(void)((!!(cond)) || (std::cout << "Assertion '" << #cond "' failed [" __FILE__ " : " << __LINE__ << "].\n", ::__debugbreak(), 0))

#include <cuda.h>
#include <PxPhysics.h>
#include <PxPhysicsAPI.h>

#include <Windows.h>
#include <windowsx.h>
#include <tchar.h>

#include <mutex>
#include <limits>
#include <array>
#include <string>
#include <vector>
#include <iostream>
#include <memory>

#include <set>
#include <unordered_map>
#include <queue>

#include <functional>
#include <tuple>

static physx::PxVec3 gravity(0.0f, -9.8f, 0.0f);

template <typename T> using ref = std::shared_ptr<T>;
template <typename T> using weakref = std::weak_ptr<T>;

template <typename F_, typename... Args>
using IsCallableFunc = std::enable_if_t<std::is_invocable_v<F_, Args...>, bool>;

template <typename T, typename... Args>
NODISCARD inline ref<T> make_ref(Args&&... args)
{
	return std::make_shared<T>(std::forward<Args>(args)...);
}

template <typename T> struct is_ref : std::false_type {};
template <typename T> struct is_ref<ref<T>> : std::true_type {};

template <typename T> inline constexpr bool is_ref_v = is_ref<T>::value;

#define arraysize(arr) (sizeof(arr) / sizeof((arr)[0]))

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

template <typename T>
NODISCARD constexpr inline auto min(T a, T b)
{
	return (a < b) ? a : b;
}

template <typename T>
NODISCARD constexpr inline auto max(T a, T b)
{
	return (a < b) ? b : a;
}

#define M_PI 3.14159265359f
#define M_PI_OVER_2 (M_PI * 0.5f)
#define M_PI_OVER_180 (M_PI / 180.f)
#define M_180_OVER_PI (180.f / M_PI)
#define SQRT_PI	1.77245385090f
#define INV_PI 0.31830988618379f
#define M_TAU 6.28318530718f
#define INV_TAU 0.159154943091895335f

#define EPSILON 1e-6f

#define deg2rad(deg) ((deg) * M_PI_OVER_180)
#define rad2deg(rad) ((rad) * M_180_OVER_PI)

#define KB(n) (1024ull * (n))
#define MB(n) (1024ull * KB(n))
#define GB(n) (1024ull * MB(n))

#define BYTE_TO_KB(b) ((b) / 1024)
#define BYTE_TO_MB(b) ((b) / (1024 * 1024))
#define BYTE_TO_GB(b) ((b) / (1024 * 1024))

NODISCARD static constexpr float lerp(float l, float u, float t) { return l + t * (u - l); }
NODISCARD static constexpr float inverseLerp(float l, float u, float v) { return (v - l) / (u - l); }
NODISCARD static constexpr float remap(float v, float oldL, float oldU, float newL, float newU) { return lerp(newL, newU, inverseLerp(oldL, oldU, v)); }
NODISCARD static constexpr float clamp(float v, float l, float u) { float r = max(l, v); r = min(u, r); return r; }
NODISCARD static constexpr uint32_t clamp(uint32_t v, uint32_t l, uint32_t u) { uint32_t r = max(l, v); r = min(u, r); return r; }
NODISCARD static constexpr int32_t clamp(int32_t v, int32_t l, int32_t u) { int32_t r = max(l, v); r = min(u, r); return r; }
NODISCARD static constexpr float clamp01(float v) { return clamp(v, 0.f, 1.f); }
NODISCARD static constexpr float saturate(float v) { return clamp01(v); }
NODISCARD static constexpr float smoothstep(float t) { return t * t * (3.f - 2.f * t); }
NODISCARD static constexpr float smoothstep(float l, float u, float v) { return smoothstep(clamp01(inverseLerp(l, u, v))); }
NODISCARD static constexpr uint32_t bucketize(uint32_t problemSize, uint32_t bucketSize) { return (problemSize + bucketSize - 1) / bucketSize; }
NODISCARD static constexpr uint64_t bucketize(uint64_t problemSize, uint64_t bucketSize) { return (problemSize + bucketSize - 1) / bucketSize; }\

#define PX_SCENE_QUERY_SETUP(blockSingle) \
const PxHitFlags hitFlags = PxHitFlag::ePOSITION | PxHitFlag::eNORMAL | PxHitFlag::eMESH_MULTIPLE | PxHitFlag::eUV; \
PxQueryFilterData filterData; \
filterData.flags |= PxQueryFlag::eDYNAMIC | PxQueryFlag::eSTATIC; \
filterData.data.word0 = layerMask; \
filterData.data.word1 = blockSingle ? 1 : 0; \
filterData.data.word2 = hitTriggers ? 1 : 0

#define PX_SCENE_QUERY_SETUP_SWEEP_CAST_ALL() PX_SCENE_QUERY_SETUP(true); \
		PxSweepBufferN<1> buffer

#define PX_SCENE_QUERY_SETUP_SWEEP_CAST() PX_SCENE_QUERY_SETUP(false); \
		DynamicHitBuffer<PxSweepHit> buffer

#define PX_SCENE_QUERY_SETUP_CHECK() PX_SCENE_QUERY_SETUP(false); \
		PxOverlapBufferN<1> buffer

#define PX_SCENE_QUERY_SETUP_OVERLAP() PX_SCENE_QUERY_SETUP(false); \
		DynamicHitBuffer<PxOverlapHit> buffer

#define PX_SCENE_QUERY_COLLECT_OVERLAP() results.clear(); \
		results.resize(buffer.getNbTouches()); \
		size_t resultSize = results.size(); \
		for (int32_t i = 0; i < resultSize; i++) \
		{ \
			auto& hitInfo = results[i]; \
			const auto& hit = buffer.getTouch(i); \
			hitInfo = hit.shape ? static_cast<uint32_t*>(hit.shape->userData) : nullptr; \
		}

namespace physx
{
	NODISCARD static PxVec2 min(const PxVec2& a, const PxVec2& b) noexcept { return PxVec2(std::min(a.x, b.x), std::min(a.y, b.y)); }
	NODISCARD static PxVec3 min(const PxVec3& a, const PxVec3& b) noexcept { return PxVec3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)); }

	NODISCARD static PxVec2 max(const PxVec2& a, const PxVec2& b) noexcept { return PxVec2(std::max(a.x, b.x), std::max(a.y, b.y)); }
	NODISCARD static PxVec3 max(const PxVec3& a, const PxVec3& b) noexcept { return PxVec3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)); }
}