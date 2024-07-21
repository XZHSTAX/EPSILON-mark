add_library(hdmap_road_network)
add_library(hdmap::road_network ALIAS hdmap_road_network)

# 将源文件与特定目标（如可执行文件、库文件等）关联的指令
# 不必将源文件列表直接写在 add_executable 或 add_library 的参数中
target_sources(hdmap_road_network PRIVATE
  lane.cc
  lane_map.cc
  road_network.cc
)
target_include_directories(hdmap_road_network PUBLIC
  ${ADMAP_INCLUDE_DIRS}
  ./road_network/
)