set(CAT_LIB_NAME cat_user_entity)


add_library(${CAT_LIB_NAME}
  abstract_handle.cpp
  tf_scenegraph_object.cpp
  camera_node.cpp
  manipulator_node.cpp
  user_entity.cpp
  abstract_interaction_tool.cpp
  #haptic_interaction_tool.cpp
  hydra_interaction_tool.cpp
)

add_dependencies(${CAT_LIB_NAME} interaction_cursor_msgs)

target_link_libraries(${CAT_LIB_NAME}
  ${catkin_LIBRARIES}
)

