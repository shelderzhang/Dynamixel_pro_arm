#Specify minimum cmake requirements
cmake_minimum_required(VERSION 2.8.12)


project(Dynamixel_pro_arm)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
include_directories("${PROJECT_SOURCE_DIR}/include")
set(SRC_LIST 
  #         "indirect_address.cpp"
 			 "dynamixel_arm_main.cpp"
             "dynamixel_arm_controller.cpp"
             "group_bulk_read.cpp"
             "group_bulk_write.cpp"
             "group_sync_read.cpp"
             "group_sync_write.cpp"
             "packet_handler.cpp"
             "port_handler.cpp"
             "port_handler_linux.cpp"
             "protocol1_packet_handler.cpp"
             "protocol2_packet_handler.cpp")
#aux_source_directory(. SRC_LIST)
add_executable(dynamixel_arm ${SRC_LIST})



