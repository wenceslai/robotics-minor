cmake_minimum_required(VERSION 3.5)
project(robot_model)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	  set(CMAKE_CXX_STANDARD 14)
  endif()

  # Find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(gazebo_ros REQUIRED)

  # Include directories for your own header files, if any
  include_directories(
	    include
    )

    # Declare the executable (if your package has nodes)
    # add_executable(robot_model_node src/robot_model_node.cpp)

    # Link libraries and other dependencies
    # target_link_libraries(robot_model_node
    #   rclcpp
    #   gazebo_ros
    # )

    # Install launch files, configurations, and other resources
    install(DIRECTORY launch
	      DESTINATION share/${PROJECT_NAME}/
      )

      install(DIRECTORY meshes
	        DESTINATION share/${PROJECT_NAME}/
	)

	install(DIRECTORY model
		  DESTINATION share/${PROJECT_NAME}/
	  )

	  # Install Python scripts if needed
	  # install(PROGRAMS
	  #   scripts/your_python_script.py
	  #   DESTINATION lib/${PROJECT_NAME}
	  # )

	  # Export the package so it can be used by others
	  ament_package()
