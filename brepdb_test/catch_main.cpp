// Provides the Catch2 test runner entry point.
// Kept in a separate TU so the rest of the test sources stay header-only.
#define CATCH_CONFIG_RUNNER
#include <catch2/catch_session.hpp>

int main(int argc, char* argv[])
{
    return Catch::Session().run(argc, argv);
}
