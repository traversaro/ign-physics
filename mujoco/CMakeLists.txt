
# This component expresses custom features of the mujoco plugin, which can
# expose native mujoco data types.
ign_add_component(mujoco INTERFACE
  DEPENDS_ON_COMPONENTS sdf heightmap mesh
  GET_TARGET_NAME features)

target_link_libraries(${features} INTERFACE IgnMuJoCo::IgnMuJoCo)


install(
  DIRECTORY include/
  DESTINATION "${IGN_INCLUDE_INSTALL_DIR_FULL}")

ign_get_libsources_and_unittests(sources test_sources)

# TODO(MXG): Think about an ign_add_plugin(~) macro for ign-cmake
set(engine_name mujoco-plugin)
ign_add_component(${engine_name}
  SOURCES ${sources}
  DEPENDS_ON_COMPONENTS mujoco
  GET_TARGET_NAME mujoco_plugin)

target_link_libraries(${mujoco_plugin}
  PUBLIC
    ${features}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-heightmap
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-math${IGN_MATH_VER}::eigen3
  PRIVATE
    # We need to link this, even when the profiler isn't used to get headers.
    ignition-common${IGN_COMMON_VER}::profiler
)

# Note that plugins are currently being installed in 2 places: /lib and the engine-plugins dir
install(TARGETS ${mujoco_plugin} DESTINATION ${IGNITION_PHYSICS_ENGINE_INSTALL_DIR})

# The library created by `ign_add_component` includes the ign-physics version
# (i.e. libignition-physics1-name-plugin.so), but for portability,
# we also install an unversioned symlink into the same versioned folder.
set(versioned ${CMAKE_SHARED_LIBRARY_PREFIX}${mujoco_plugin}${CMAKE_SHARED_LIBRARY_SUFFIX})
set(unversioned ${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME_NO_VERSION_LOWER}-${engine_name}${CMAKE_SHARED_LIBRARY_SUFFIX})
if (WIN32)
  # disable MSVC inherit via dominance warning
  target_compile_options(${mujoco_plugin} PUBLIC "/wd4250")
  INSTALL(CODE "EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E copy
      ${IGNITION_PHYSICS_ENGINE_INSTALL_DIR}\/${versioned}
      ${IGNITION_PHYSICS_ENGINE_INSTALL_DIR}\/${unversioned})")
else()
  EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${versioned} ${unversioned})
  INSTALL(FILES ${PROJECT_BINARY_DIR}/${unversioned} DESTINATION ${IGNITION_PHYSICS_ENGINE_INSTALL_DIR})
endif()

# Testing
ign_build_tests(
  TYPE UNIT
  SOURCES ${test_sources}
  LIB_DEPS
    ${features}
    ignition-plugin${IGN_PLUGIN_VER}::loader
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-heightmap
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
  TEST_LIST tests)

foreach(test ${tests})

  target_compile_definitions(${test} PRIVATE
    "mujoco_plugin_LIB=\"$<TARGET_FILE:${mujoco_plugin}>\""
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/worlds/\""
    "IGNITION_PHYSICS_RESOURCE_DIR=\"${IGNITION_PHYSICS_RESOURCE_DIR}\"")

  # Helps when we want to build a single test after making changes to mujoco_plugin
  add_dependencies(${test} ${mujoco_plugin})
endforeach()

foreach(test UNIT_FindFeatures_TEST UNIT_RequestFeatures_TEST)
  if(TARGET ${test})
    target_compile_definitions(${test} PRIVATE
      "mujoco_plugin_LIB=\"$<TARGET_FILE:${mujoco_plugin}>\"")
  endif()
endforeach()
