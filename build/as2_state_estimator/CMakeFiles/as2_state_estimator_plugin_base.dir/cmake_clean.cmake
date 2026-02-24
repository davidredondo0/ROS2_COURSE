file(REMOVE_RECURSE
  "libas2_state_estimator_plugin_base.pdb"
  "libas2_state_estimator_plugin_base.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/as2_state_estimator_plugin_base.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
