file(REMOVE_RECURSE
  "libpid_controller.pdb"
  "libpid_controller.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/pid_controller.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
