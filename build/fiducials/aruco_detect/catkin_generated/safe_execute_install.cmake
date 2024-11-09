execute_process(COMMAND "/home/foscar/Auto-Race-/build/fiducials/aruco_detect/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/foscar/Auto-Race-/build/fiducials/aruco_detect/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
