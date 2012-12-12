
add_subdirectory(cat_user_entity)

rosbuild_add_executable(user_test
                        main.cpp
)

target_link_libraries(user_test
cat_user_entity
)
