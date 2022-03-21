include(FetchContent)
FetchContent_Declare(
	googletest
	URL https://github.com/google/googletest/archive/054a986a8513149e8374fc669a5fe40117ca6b41.zip
)

FetchContent_MakeAvailable(googletest)

enable_testing()
add_executable(
	cpp-pathfinding-tests
	EXCLUDE_FROM_ALL
	astar_tests.cpp
)

target_link_libraries(cpp-pathfinding-tests gtest_main libpathfinding)
include(GoogleTest)
gtest_discover_tests(cpp-pathfinding-tests)
