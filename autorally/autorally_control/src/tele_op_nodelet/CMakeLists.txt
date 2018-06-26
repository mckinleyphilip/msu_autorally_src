add_library(tele_op_nodelet tele_op_nodelet.cpp)
add_dependencies(tele_op_nodelet autorally_msgs_gencpp)
target_link_libraries(tele_op_nodelet ${catkin_LIBRARIES} ${Boost_LIBRARIES} RingBuffer)

install(TARGETS
  tele_op_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
