file(REMOVE_RECURSE
  "libpath_planner_common.pdb"
  "libpath_planner_common.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/path_planner_common.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
