cmake_minimum_required(VERSION 3.2)

project(Sched-Thread-Milp)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_CXX_COMPILER STREQUAL "GNU")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.0")
    message(FATAL_ERROR "Insufficient gcc version")
  endif()
endif()

set(MAJOR 1)
set(MINOR 0)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_TEST_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/test)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(GUROBI REQUIRED)

if(GUROBI_FOUND)
  message(STATUS "GUROBI FOUND")
else(GUROBI_FOUND)
  message(FATAL "GUROBI NOT FOUND")
endif(GUROBI_FOUND)

include_directories(${GUROBI_INCLUDE_DIRS})

set(CORE_BINARY stm)

add_executable(${CORE_BINARY}
  src/main.cpp
  src/parser.cpp
  src/problem.cpp
  src/solver.cpp)

target_compile_definitions(${CORE_BINARY} PUBLIC -D_GLIBCXX_USE_CXX11_ABI=0)

target_link_libraries(${CORE_BINARY} ${GUROBI_LIBRARIES})

set(ENABLE_TESTING 1)
include(CTest)

add_subdirectory(test)
