#ifndef _OPENPS_DECLS_
#define _OPENPS_DECLS_

#include <intrin.h>
#include <xmmintrin.h>
#include <Windows.h>
#include <stdio.h>
#include <assert.h>
#include <ctype.h>
#include <math.h>
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
#include <intrin.h>
#include <stddef.h>

#include <cuda.h>
#include <PxPhysics.h>
#include <PxPhysicsAPI.h>

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

#define UNUSED(x) (void)(x)

#define RELEASE_PTR(ptr) if(ptr) { delete ptr; ptr = nullptr; }
#define RELEASE_ARRAY_PTR(arrayPtr) if(arrayPtr) { delete[] arrayPtr; arrayPtr = nullptr; } 

#define ASSERT(cond) \
	(void)((!!(cond)) || (std::cout << "Assertion '" << #cond "' failed [" __FILE__ " : " << __LINE__ << "].\n", ::__debugbreak(), 0))

static inline physx::PxVec3 gravity(0.0f, -9.8f, 0.0f);

template <typename T> using ref = std::shared_ptr<T>;
template <typename T> using weakref = std::weak_ptr<T>;

template <typename F_, typename... Args>
using IsCallableFunc = std::enable_if_t<std::is_invocable_v<F_, Args...>, bool>;

template <typename T, typename... Args>
NODISCARD inline ref<T> make_ref(Args&&... args) noexcept
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
NODISCARD constexpr inline auto min(T a, T b) noexcept
{
	return (a < b) ? a : b;
}

template <typename T>
NODISCARD constexpr inline auto max(T a, T b) noexcept
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

constexpr auto EPSILON = 1e-6f;

#define deg2rad(deg) ((deg) * M_PI_OVER_180)
#define rad2deg(rad) ((rad) * M_180_OVER_PI)

template<typename T>
inline constexpr auto KB(T n) noexcept { return (1024ull * (n)); }

template<typename T>
inline constexpr auto MB(T n) noexcept { return (1024ull * KB(n)); }

template<typename T>
inline constexpr auto GB(T n) noexcept { return (1024ull * MB(n)); }

template<typename T>
inline constexpr auto BYTE_TO_KB(T b) noexcept { return ((b) / 1024); }

template<typename T>
inline constexpr auto BYTE_TO_MB(T b) noexcept { return ((b) / (1024 * 1024)); }

template<typename T>
inline constexpr auto BYTE_TO_GB(T b) noexcept { return ((b) / (1024 * 1024)); }

NODISCARD inline constexpr float lerp(float l, float u, float t) noexcept { return l + t * (u - l); }
NODISCARD inline constexpr float inverseLerp(float l, float u, float v) noexcept { return (v - l) / (u - l); }
NODISCARD inline constexpr float remap(float v, float oldL, float oldU, float newL, float newU) noexcept { return lerp(newL, newU, inverseLerp(oldL, oldU, v)); }
NODISCARD inline constexpr float clamp(float v, float l, float u) noexcept { float r = max(l, v); r = min(u, r); return r; }
NODISCARD inline constexpr uint32_t clamp(uint32_t v, uint32_t l, uint32_t u) noexcept { uint32_t r = max(l, v); r = min(u, r); return r; }
NODISCARD inline constexpr int32_t clamp(int32_t v, int32_t l, int32_t u) noexcept { int32_t r = max(l, v); r = min(u, r); return r; }
NODISCARD inline constexpr float clamp01(float v) noexcept { return clamp(v, 0.f, 1.f); }
NODISCARD inline constexpr float saturate(float v) noexcept { return clamp01(v); }
NODISCARD inline constexpr float smoothstep(float t) noexcept { return t * t * (3.f - 2.f * t); }
NODISCARD inline constexpr float smoothstep(float l, float u, float v) noexcept { return smoothstep(clamp01(inverseLerp(l, u, v))); }
NODISCARD inline constexpr uint32_t bucketize(uint32_t problemSize, uint32_t bucketSize) noexcept { return (problemSize + bucketSize - 1) / bucketSize; }
NODISCARD inline constexpr uint64_t bucketize(uint64_t problemSize, uint64_t bucketSize) noexcept { return (problemSize + bucketSize - 1) / bucketSize; }\

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
		for (uint32_t i = 0; i < resultSize; ++i) \
		{ \
			auto& hitInfo = results[i]; \
			const auto& hit = buffer.getTouch(i); \
			hitInfo = hit.shape ? static_cast<uint32_t*>(hit.shape->userData) : nullptr; \
		}

namespace physx
{
	NODISCARD inline PxVec2 min(const PxVec2& a, const PxVec2& b) noexcept { return PxVec2(std::min(a.x, b.x), std::min(a.y, b.y)); }
	NODISCARD inline PxVec3 min(const PxVec3& a, const PxVec3& b) noexcept { return PxVec3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)); }

	NODISCARD inline PxVec2 max(const PxVec2& a, const PxVec2& b) noexcept { return PxVec2(std::max(a.x, b.x), std::max(a.y, b.y)); }
	NODISCARD inline PxVec3 max(const PxVec3& a, const PxVec3& b) noexcept { return PxVec3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)); }
}

#endif