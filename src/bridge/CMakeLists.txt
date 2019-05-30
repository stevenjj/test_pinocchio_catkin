FILE(GLOB sources *.cpp)

add_library(rviz_bridge_files SHARED ${sources})
target_link_libraries (rviz_bridge_files ${catkin_LIBRARIES})

