cmake_minimum_required(VERSION 3.14)

project(astarTests LANGUAGES C CXX)

include(../cmake/project-is-top-level.cmake)
include(../cmake/folders.cmake)

# ---- Dependencies ----

if(PROJECT_IS_TOP_LEVEL)
  find_package(astar REQUIRED)
  enable_testing()
endif()


function(test_bench_generator TEST_BENCH_NAME IS_TEST ADD_TO_TEST)
  if (IS_TEST)
    add_executable("${TEST_BENCH_NAME}" "source/test/${TEST_BENCH_NAME}.cpp" source/generator/generator.cpp source/generator/generator.hpp)
  else()
    add_executable("${TEST_BENCH_NAME}" "source/benchmark/${TEST_BENCH_NAME}.cpp" source/generator/generator.cpp source/generator/generator.hpp)
  endif()


  if (IS_TEST)
    target_link_libraries("${TEST_BENCH_NAME}" PRIVATE gtest)
  else()
    target_link_libraries("${TEST_BENCH_NAME}" PRIVATE benchmark::benchmark)
  endif()

  target_link_libraries("${TEST_BENCH_NAME}" PRIVATE astar::astar)
  target_link_libraries("${TEST_BENCH_NAME}" PRIVATE raylib)
  target_link_libraries("${TEST_BENCH_NAME}" PRIVATE FastNoise2)
  #target_link_libraries("${TEST_BENCH_NAME}" PRIVATE spdlog::spdlog nlohmann_json::nlohmann_json)

  #if (OpenMP_FOUND OR OpenMP_CXX_FOUND)
  #  target_link_libraries("${TEST_BENCH_NAME}" PRIVATE OpenMP::OpenMP_CXX)
  #endif()

  set_target_properties("${TEST_BENCH_NAME}"
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    PDB_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
  )

  #if(NOT CMAKE_BUILD_TYPE MATCHES Debug AND NOT CMAKE_BUILD_TYPE MATCHES Coverage)
  #  add_test(NAME "${TEST_BENCH_NAME}" COMMAND $<TARGET_FILE:${TEST_BENCH_NAME}>)
  #elseif()
  #  message(STATUS "Disable ${BENCH_NAME}, Performance benchmark test only run on Release/RelWithDebInfo/MinSizeRel")
  #endif()

  if (ADD_TO_TEST)
      add_test(NAME "${TEST_BENCH_NAME}" COMMAND $<TARGET_FILE:${TEST_BENCH_NAME}>)
  endif()
  target_compile_features("${TEST_BENCH_NAME}" PRIVATE cxx_std_20)
endfunction()

# ---- Tests ----

if(NOT WIN32)
  include(../cmake/lib/gtest.cmake)
  include(../cmake/lib/benchmark.cmake)
  #include(../cmake/lib/openmp.cmake)
  include(../cmake/lib/raygui.cmake)

  include(../cmake/lib/raylib.cmake)
  include(../cmake/lib/fast_noise2.cmake)
  #include(../cmake/lib/spdlog.cmake)
  #include(../cmake/lib/json.cmake)
  include(../cmake/utile/ccache.cmake)

  include_directories(source)

  test_bench_generator(astar_test true true)
  test_bench_generator(astar_bench false true)
  test_bench_generator(path_finder false false)
endif()

# ---- End-of-file commands ----

add_folders(Test)
