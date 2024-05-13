#include <openps.h>
#include <chrono>

namespace
{
    ref<openps::physics> physics;

    openps::rigidbody* rb1 = nullptr;
    openps::rigidbody* rb2 = nullptr;

    openps::collider_base* coll1 = nullptr;
    openps::collider_base* coll2 = nullptr;
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

    rb1 = new openps::rigidbody(1, openps::rigidbody_type::Static);
    coll1 = new openps::box_collider(1, 1, 1);
    openps::createRigidbodyActor(rb1, coll1, physx::PxTransform(physx::PxVec3(0)));

    rb2 = new openps::rigidbody(2, openps::rigidbody_type::Dynamic);
    coll2 = new openps::sphere_collider(1);
    openps::createRigidbodyActor(rb2, coll2, physx::PxTransform(physx::PxVec3(0, 50, 0)));

    return true;
}

static bool update(float dt)
{
    openps::logger::log_message("Update");
    physics->update(dt);
    
    {
        openps::physics_lock_read lock{};
        PX_UNUSED(lock);

        const auto pos1 = rb1->getPosition();
        openps::logger::log_message(std::to_string(pos1.y).c_str());

        const auto pos2 = rb2->getPosition();
        openps::logger::log_message(std::to_string(pos2.y).c_str());
    }

    return true;
}

static void release()
{
    physics.reset();
}

int main(int argc, char* argv[])
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
            bool successStep = update(elapsed_time);

            auto end = std::chrono::high_resolution_clock::now();
            elapsed_time = std::chrono::duration<float, std::milli>(end - start).count();

            if (!successStep)
                break;
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