
if(NOT "/ros2_ws/build/as2_behaviors_trajectory_generation/_deps/dynamic_trajectory_generator-subbuild/dynamic_trajectory_generator-populate-prefix/src/dynamic_trajectory_generator-populate-stamp/dynamic_trajectory_generator-populate-gitinfo.txt" IS_NEWER_THAN "/ros2_ws/build/as2_behaviors_trajectory_generation/_deps/dynamic_trajectory_generator-subbuild/dynamic_trajectory_generator-populate-prefix/src/dynamic_trajectory_generator-populate-stamp/dynamic_trajectory_generator-populate-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/ros2_ws/build/as2_behaviors_trajectory_generation/_deps/dynamic_trajectory_generator-subbuild/dynamic_trajectory_generator-populate-prefix/src/dynamic_trajectory_generator-populate-stamp/dynamic_trajectory_generator-populate-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/ros2_ws/build/as2_behaviors_trajectory_generation/_deps/dynamic_trajectory_generator-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/ros2_ws/build/as2_behaviors_trajectory_generation/_deps/dynamic_trajectory_generator-src'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git"  clone --no-checkout --config "advice.detachedHead=false" "https://github.com/miferco97/dynamic_trajectory_generator.git" "dynamic_trajectory_generator-src"
    WORKING_DIRECTORY "/ros2_ws/build/as2_behaviors_trajectory_generation/_deps"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/miferco97/dynamic_trajectory_generator.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git"  checkout master --
  WORKING_DIRECTORY "/ros2_ws/build/as2_behaviors_trajectory_generation/_deps/dynamic_trajectory_generator-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'master'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git"  submodule update --recursive --init 
    WORKING_DIRECTORY "/ros2_ws/build/as2_behaviors_trajectory_generation/_deps/dynamic_trajectory_generator-src"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/ros2_ws/build/as2_behaviors_trajectory_generation/_deps/dynamic_trajectory_generator-src'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "/ros2_ws/build/as2_behaviors_trajectory_generation/_deps/dynamic_trajectory_generator-subbuild/dynamic_trajectory_generator-populate-prefix/src/dynamic_trajectory_generator-populate-stamp/dynamic_trajectory_generator-populate-gitinfo.txt"
    "/ros2_ws/build/as2_behaviors_trajectory_generation/_deps/dynamic_trajectory_generator-subbuild/dynamic_trajectory_generator-populate-prefix/src/dynamic_trajectory_generator-populate-stamp/dynamic_trajectory_generator-populate-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/ros2_ws/build/as2_behaviors_trajectory_generation/_deps/dynamic_trajectory_generator-subbuild/dynamic_trajectory_generator-populate-prefix/src/dynamic_trajectory_generator-populate-stamp/dynamic_trajectory_generator-populate-gitclone-lastrun.txt'")
endif()

