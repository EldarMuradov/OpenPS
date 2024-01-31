#pragma once
#include <iostream>

using log_message_func_ptr = void(*)(const char*);
using log_error_func_ptr = void(*)(const char*);

namespace openps
{
	static void default_log_message(const char* message) { std::cout << "[OpenPS] " << message << "\n"; }
	static void default_log_error(const char* message) { std::cerr << "[OpenPS] Error " << message << "\n"; }

	class logger
	{
		static log_message_func_ptr logMessageFunc;
		static log_error_func_ptr logErrorFunc;

		friend struct physics;

	public:
		logger() = delete;
		logger(const logger&) = delete;
		logger(logger&&) = delete;

		static void log_message(const char* message) { logMessageFunc(message); }
		static void log_error(const char* message) { logErrorFunc(message); }
	};
}