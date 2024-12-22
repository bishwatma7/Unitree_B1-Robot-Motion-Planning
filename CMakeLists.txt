cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk)

include_directories(include)

# add_compile_options(-std=c++11)
add_compile_options(-std=c++14)

# check arch and os
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")

if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()

if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

link_directories(lib/cpp/${ARCH})
set(EXTRA_LIBS -pthread libunitree_legged_sdk.so lcm)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ../lib/cpp/${ARCH})  # .lib .a
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib/python)  # .so .dylib
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../py/bin)   #  .exe .dll

# one pc one process

add_executable(example_dance example/example_dance.cpp)
target_link_libraries(example_dance ${EXTRA_LIBS})

add_executable(test1 example/test1.cpp)
target_link_libraries(test1 ${EXTRA_LIBS})

add_executable(test2 example/test2.cpp)
target_link_libraries(test2 ${EXTRA_LIBS})

add_executable(modeTesting example/modeTesting.cpp)
target_link_libraries(modeTesting ${EXTRA_LIBS})

add_executable(walk_down_walkBack example/walk_down_walkBack.cpp)
target_link_libraries(walk_down_walkBack ${EXTRA_LIBS})

add_executable(walk_down example/walk_down.cpp)
target_link_libraries(walk_down ${EXTRA_LIBS})

add_executable(stand_up_walkBack example/stand_up_walkBack.cpp)
target_link_libraries(stand_up_walkBack  ${EXTRA_LIBS})

add_executable(stairs example/stairs.cpp)
target_link_libraries(stairs  ${EXTRA_LIBS})

add_executable(demostart example/demostart.cpp)
target_link_libraries(demostart  ${EXTRA_LIBS})

add_executable(demomiddle example/demomiddle.cpp)
target_link_libraries(demomiddle  ${EXTRA_LIBS})




