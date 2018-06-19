add_library(tele_op_PID_speed_controller tele_op_PID_speed_controller.cpp)
add_dependencies(tele_op_PID_speed_controller autorally_msgs_gencpp)
target_link_libraries(tele_op_PID_speed_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES} RingBuffer)

install(TARGETS
  tele_op_PID_speed_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
