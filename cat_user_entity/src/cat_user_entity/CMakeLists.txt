include_directories(.)

ADD_DEFINITIONS(-fPIC)


rosbuild_add_library(cat_user_entity

abstract_handle.cpp
tf_scenegraph_object.cpp
camera_node.cpp
manipulator_node.cpp
user_entity.cpp
abstract_interaction_tool.cpp
#haptic_interaction_tool.cpp
hydra_interaction_tool.cpp

)
