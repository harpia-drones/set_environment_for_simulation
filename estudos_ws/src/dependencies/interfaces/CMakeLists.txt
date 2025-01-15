cmake_minimum_required(VERSION 3.8)
project(${PROJECT_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)


######## Uncomment and modify the section bellow ########

# set(msg_files
#   "msg/Velocity.msg"
#   "msg/Time.msg"
# )

# set(srv_files
#   "srv/Interface_1.srv"
#   "srv/Interface_1.srv"
# )

# set(action_files
#   "srv/Interface_1.action"
#   "srv/Interface_2.action"
#   ect...
# )

# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
#   ${srv_files}
#   ${action_files}
# )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
