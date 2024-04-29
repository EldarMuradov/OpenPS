#ifndef _OPENPS_LOGGER_
#define _OPENPS_LOGGER_

#include <iostream>

using log_message_func_ptr = void(*)(const char*);
using log_error_func_ptr = void(*)(const char*);

namespace openps
{
	inline void default_log_message(const char* message) noexcept { std::cout << "[OpenPS] " << message << "\n"; }
	inline void default_log_error(const char* message) noexcept { std::cerr << "[OpenPS] Error " << message << "\n"; }

	class logger
	{
		logger() = delete;
		logger(const logger&) = delete;
		logger(logger&&) = delete;

		static inline log_message_func_ptr logMessageFunc;
		static inline log_error_func_ptr logErrorFunc;

		friend struct physics;

	public:

		static void log_message(const char* message) { logMessageFunc(message); }
		static void log_error(const char* message) { logErrorFunc(message); }
	};
}

#endif