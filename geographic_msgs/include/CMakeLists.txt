target_include_directories(validation
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Need to access the target from the build tree to link it in tests.
unset(_cpp_typesupport_target)
rosidl_get_typesupport_target(_cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp
)
target_link_libraries(validation
  INTERFACE
    ${_cpp_typesupport_target}
)
