execute_process(COMMAND "/home/nazha/sim_ws/build/tuw_geometry/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/nazha/sim_ws/build/tuw_geometry/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
