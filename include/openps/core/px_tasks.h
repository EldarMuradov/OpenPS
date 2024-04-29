#ifndef _OPENPS_TASKS_
#define _OPENPS_TASKS_

#include <openps_decl.h>

#include <core/px_logger.h>
#include <core/px_physics.h>
#include <core/px_structs.h>
#include <core/px_wrappers.h>
#include <core/px_aggregates.h>

#include <ecs/px_rigidbody.h>
#include <ecs/px_colliders.h>

namespace openps
{
	namespace helper
	{
		template <int... Is>
		struct index {};

		template <int N, int... Is>
		struct gen_seq : gen_seq<N - 1, N - 1, Is...> {};

		template <int... Is>
		struct gen_seq<0, Is...> : index<Is...> {};
	}

	template<typename Func, typename... Args>
	struct light_cpu_task : physx::PxLightCpuTask
	{
		light_cpu_task() = default;

		template<IsCallableFunc<Func, Args...> = true>
		light_cpu_task(std::function<Func(Args...)> f, Args&&... ars) noexcept
			: func(f),
			args(std::forward<Args>(ars)...) {}
		~light_cpu_task() {}

		virtual const char* getName() const { return "PhysX Task"; }
		virtual void run() { call(args); }

	private:
		template <typename... Args, int... Is>
		void call(std::tuple<Args...>& tup, helper::index<Is...>) noexcept
		{
			func(std::get<Is>(tup)...);
		}

		template <typename... Args>
		void call(std::tuple<Args...>& tup) noexcept
		{
			call(tup, helper::gen_seq<sizeof...(Args)>{});
		}

	protected:
		std::tuple<Args...> args;
		std::function<Func(Args...)> func;
	};
}

#endif