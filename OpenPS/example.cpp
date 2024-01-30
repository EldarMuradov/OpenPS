#include <pch.h>
#include <openps.h>
#include <chrono>

namespace
{
    ref<openps::physics> physics;
}

static void test_log_message(const char* message) { std::cout << message << "\n"; }
static void test_log_error(const char* message) { std::cerr << message << "\n"; }

static bool initialize()
{
    openps::physics_desc desc{};
    desc.logErrorFunc = test_log_error;
    desc.logMessageFunc = test_log_message;

    physics = make_ref<openps::physics>(desc);
    openps::logger::log_message("Started successfuly");

    return true;
}

static void update(float dt)
{
    openps::logger::log_message("Update");
    physics->update(dt);
}

static void release()
{
    physics.reset();
}

int main()
{
    if (!initialize())
    {
        openps::logger::log_error("Initialization failed!");
        return -1;
    }
    
    try 
    {
        auto start = std::chrono::high_resolution_clock::now();
        float elapsed_time = 0.0f;
        while (true)
        {
            update(elapsed_time);

            auto end = std::chrono::high_resolution_clock::now();
            elapsed_time = std::chrono::duration<float, std::milli>(end - start).count();
        }

        release();
    }
    catch (...)
    {
        openps::logger::log_error("Runtime Error!");
        return -1;
    }

    return 0;
}