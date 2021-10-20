ign_get_libsources_and_unittests(sources test_sources)

set (aabb_tree_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/aabb_tree/AABB.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/aabb_tree/AABB.cc)
set(sources ${sources} ${aabb_tree_SRC})

ign_add_component(mujocolib
  SOURCES ${sources}
  GET_TARGET_NAME mujocolib_target
  CXX_STANDARD 17)

target_link_libraries(${mujocolib_target}
  PUBLIC IgnMuJoCo::IgnMuJoCo
  PRIVATE
    ignition-common${IGN_COMMON_VER}::requested
    ignition-math${IGN_MATH_VER}::eigen3
    TINYXML2::TINYXML2
)

 ign_build_tests(
  TYPE UNIT_mujocolib
  SOURCES ${test_sources}
  LIB_DEPS
    ignition-plugin${IGN_PLUGIN_VER}::loader
    ignition-common${IGN_COMMON_VER}::requested
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    ${mujocolib_target}
  TEST_LIST tests)
