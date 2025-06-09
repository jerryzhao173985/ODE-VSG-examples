cmake_minimum_required(VERSION 3.10)
project(ODE_VSG_examples)
enable_testing()

option(USE_VSG "Build with VulkanSceneGraph" ON)

add_executable(wheeled_simulation wheeled_simulation.cpp)
# Include headers
target_include_directories(wheeled_simulation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ODE REQUIRED ode)

target_link_libraries(wheeled_simulation PRIVATE ${ODE_LIBRARIES})
target_include_directories(wheeled_simulation PRIVATE ${ODE_INCLUDE_DIRS})

if(USE_VSG)
    find_package(vsg REQUIRED)
    target_link_libraries(wheeled_simulation PRIVATE vsg::vsg)
    target_compile_definitions(wheeled_simulation PRIVATE USE_VSG)
endif()

# Test program (no VSG needed)
add_executable(headless_test tests/headless_test.cpp)
target_include_directories(headless_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(headless_test PRIVATE ${ODE_LIBRARIES})
add_test(NAME headless COMMAND headless_test)

# Synchronization test using a dummy rendering module
add_executable(sync_test tests/sync_test.cpp)
target_include_directories(sync_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sync_test PRIVATE ${ODE_LIBRARIES})
add_test(NAME sync COMMAND sync_test)
