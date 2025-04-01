cmake_minimum_required(VERSION 3.16)
project(dvrk_model VERSION 2.3.0)

find_package (
  catkin QUIET
  )

if (catkin_FOUND)
  catkin_package ()

else (catkin_FOUND)

  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)

    set (_to_be_installed meshes urdf ros2/rviz ros2/launch ros2/ros2_control ros2/config)

    foreach (_dir ${_to_be_installed})
      if (EXISTS "${dvrk_model_SOURCE_DIR}/${_dir}")
        install (DIRECTORY
          ${_dir}
          DESTINATION share/${PROJECT_NAME})
      endif ()
    endforeach ()

    ament_package ()
  endif (ament_cmake_FOUND)

endif (catkin_FOUND)
