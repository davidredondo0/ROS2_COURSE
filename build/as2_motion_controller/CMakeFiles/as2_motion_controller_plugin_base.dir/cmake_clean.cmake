file(REMOVE_RECURSE
  "libas2_motion_controller_plugin_base.pdb"
  "libas2_motion_controller_plugin_base.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/as2_motion_controller_plugin_base.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
