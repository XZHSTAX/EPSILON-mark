/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "lane_map.h"

namespace hdmap {
// 静态成员变量，储存了整个地图的所有车道信息
// 其是LaneMapType类型的，即std::unordered_map<LaneId, std::shared_ptr<Lane>>
// 即 LaneId为键，std::shared_ptr<Lane>为值
// TODO: 这里很重要
LaneMapType LaneMap::lane_map_;

// 静态方法，返回本类中的静态成员变量lane_map_的指针，使得其他类可以修改该值。
LaneMapType* LaneMap::mutable_lane_map() { return &lane_map_; }

std::shared_ptr<Lane> LaneMap::GetLane(const LaneId& id) {
  if (lane_map_.find(id) != lane_map_.end())
    return lane_map_[id];
  else
    return nullptr;
}
}  // namespace hdmap
