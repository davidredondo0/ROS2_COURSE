# CMake generated Testfile for 
# Source directory: /ros2_ws/src/aerostack2/as2_aerial_platforms/as2_platform_gazebo/tests
# Build directory: /ros2_ws/build/as2_platform_gazebo/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(as2_platform_gazebo_as2_platform_gazebo_gtest "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/ros2_ws/build/as2_platform_gazebo/test_results/as2_platform_gazebo/as2_platform_gazebo_as2_platform_gazebo_gtest.gtest.xml" "--package-name" "as2_platform_gazebo" "--output-file" "/ros2_ws/build/as2_platform_gazebo/ament_cmake_gtest/as2_platform_gazebo_as2_platform_gazebo_gtest.txt" "--command" "/ros2_ws/build/as2_platform_gazebo/tests/as2_platform_gazebo_as2_platform_gazebo_gtest" "--gtest_output=xml:/ros2_ws/build/as2_platform_gazebo/test_results/as2_platform_gazebo/as2_platform_gazebo_as2_platform_gazebo_gtest.gtest.xml")
set_tests_properties(as2_platform_gazebo_as2_platform_gazebo_gtest PROPERTIES  LABELS "gtest" REQUIRED_FILES "/ros2_ws/build/as2_platform_gazebo/tests/as2_platform_gazebo_as2_platform_gazebo_gtest" TIMEOUT "60" WORKING_DIRECTORY "/ros2_ws/build/as2_platform_gazebo/tests" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/ros2_ws/src/aerostack2/as2_aerial_platforms/as2_platform_gazebo/tests/CMakeLists.txt;24;ament_add_gtest;/ros2_ws/src/aerostack2/as2_aerial_platforms/as2_platform_gazebo/tests/CMakeLists.txt;0;")
subdirs("../gtest")
