#include <pch.h>
#include <openps.h>
#include <core/px_logger.h>

namespace openps
{
	log_message_func_ptr logger::logMessageFunc;
	log_error_func_ptr logger::logErrorFunc;
}