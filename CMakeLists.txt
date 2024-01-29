cmake_minimum_required(VERSION 3.14)

include(cmake/prelude.cmake)

project(
    astar
    VERSION 0.1.0
    DESCRIPTION "astar"
    HOMEPAGE_URL "bensuperpc.org"
    LANGUAGES NONE
)

include(cmake/project-is-top-level.cmake)
include(cmake/variables.cmake)

# ---- Declare library ----

add_library(astar_astar INTERFACE)
add_library(astar::astar ALIAS astar_astar)

set_property(
    TARGET astar_astar PROPERTY
    EXPORT_NAME astar
)

target_include_directories(
    astar_astar ${warning_guard}
    INTERFACE
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
)

target_compile_features(astar_astar INTERFACE cxx_std_20)

# ---- Install rules ----

if(NOT CMAKE_SKIP_INSTALL_RULES)
  include(cmake/install-rules.cmake)
endif()

# ---- Examples ----

if(PROJECT_IS_TOP_LEVEL)
  option(BUILD_EXAMPLES "Build examples tree." "${astar_DEVELOPER_MODE}")
  if(BUILD_EXAMPLES)
    add_subdirectory(example)
  endif()
endif()

# ---- Developer mode ----

if(NOT astar_DEVELOPER_MODE)
  return()
elseif(NOT PROJECT_IS_TOP_LEVEL)
  message(
      AUTHOR_WARNING
      "Developer mode is intended for developers of astar"
  )
endif()

include(cmake/dev-mode.cmake)
