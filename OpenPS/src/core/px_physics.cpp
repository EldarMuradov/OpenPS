#include "pch.h"
#include "core/px_physics.h"

namespace openps
{
	physics::physics()
	{
		logger::logErrorFunc = default_log_error;
		logger::logMessageFunc = default_log_message;
	}

	physics::physics(physics_desc desc)
	{
		if(desc.logErrorFunc)
			logger::logErrorFunc = desc.logErrorFunc;
		if (desc.logMessageFunc)
			logger::logMessageFunc = desc.logMessageFunc;
	}
}