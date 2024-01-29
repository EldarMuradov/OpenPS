#include <pch.h>

#include <openps.h>

namespace 
{
    ref<openps::physics> physics;
}

static void test_log_message(const char* message) { std::cout << message << "\n"; }
static void test_log_error(const char* message) { std::cerr << message << "\n"; }

int main()
{
    physics = std::make_shared<openps::physics>(openps::physics_desc{ test_log_message, test_log_error });
    openps::logger::log_message("Started successfuly");

    return 0;
}