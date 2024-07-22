add_library(hdmap_utils)
add_library(hdmap::utils ALIAS hdmap_utils)

target_sources(hdmap_utils PRIVATE
  admap_visual.cc
  admap_convertion.cc
  color_map.cc
  math.cc
)
target_include_directories(hdmap_utils PUBLIC
  ${ADMAP_INCLUDE_DIRS}
)
target_link_libraries(hdmap_utils
  ad_map_access
  ad_physics
)