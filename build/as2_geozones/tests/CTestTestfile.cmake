# CMake generated Testfile for 
# Source directory: /ros2_ws/src/aerostack2/as2_utilities/as2_geozones/tests
# Build directory: /ros2_ws/build/as2_geozones/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(as2_geozones_as2_geozones_gtest "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/ros2_ws/build/as2_geozones/test_results/as2_geozones/as2_geozones_as2_geozones_gtest.gtest.xml" "--package-name" "as2_geozones" "--output-file" "/ros2_ws/build/as2_geozones/ament_cmake_gtest/as2_geozones_as2_geozones_gtest.txt" "--command" "/ros2_ws/build/as2_geozones/tests/as2_geozones_as2_geozones_gtest" "--gtest_output=xml:/ros2_ws/build/as2_geozones/test_results/as2_geozones/as2_geozones_as2_geozones_gtest.gtest.xml")
set_tests_properties(as2_geozones_as2_geozones_gtest PROPERTIES  LABELS "gtest" REQUIRED_FILES "/ros2_ws/build/as2_geozones/tests/as2_geozones_as2_geozones_gtest" TIMEOUT "60" WORKING_DIRECTORY "/ros2_ws/build/as2_geozones/tests" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/ros2_ws/src/aerostack2/as2_utilities/as2_geozones/tests/CMakeLists.txt;23;ament_add_gtest;/ros2_ws/src/aerostack2/as2_utilities/as2_geozones/tests/CMakeLists.txt;0;")
subdirs("../gtest")
