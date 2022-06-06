cmake_minimum_required(VERSION 3.10.2)
project(greenseeker_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  GreenSeekerPacket.msg
)

# add_service_files(
#   FILES
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
