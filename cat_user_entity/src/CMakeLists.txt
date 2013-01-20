
add_subdirectory(cat_user_entity)

add_executable(user_test
               main.cpp
)
add_dependencies(user_test ${PROJECT_NAME})

target_link_libraries(user_test
  cat_user_entity
  ${catkin_LIBRARIES}
)


