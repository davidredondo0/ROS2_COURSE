# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.22.1)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(nlopt-populate NONE)



include(ExternalProject)
ExternalProject_Add(nlopt-populate
                     "UPDATE_DISCONNECTED" "False" "URL" "https://github.com/stevengj/NLopt/archive/09b3c2a6da71cabcb98d2c8facc6b83d2321ed71.zip"
                    SOURCE_DIR          "/ros2_ws/build/as2_behaviors_trajectory_generation/_deps/nlopt-src"
                    BINARY_DIR          "/ros2_ws/build/as2_behaviors_trajectory_generation/_deps/nlopt-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)


