add_library(base_controller_nodelet base_controller_nodelet.cpp)
add_dependencies(base_controller_nodelet autorally_msgs_gencpp)
target_link_libraries(base_controller_nodelet ${catkin_LIBRARIES} ${Boost_LIBRARIES} RingBuffer)

install(TARGETS
  base_controller_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
