file(REMOVE_RECURSE
  "libas2_map_server_base.pdb"
  "libas2_map_server_base.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/as2_map_server_base.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
