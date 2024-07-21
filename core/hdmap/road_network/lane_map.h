/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <memory>
#include <unordered_map>

#include "lane.h"
#include "road_network.h"

namespace hdmap {

// LaneMapType 是一个哈希表，用于存储 LaneId 和 Lane 的映射关系；
using LaneMapType =
    std::unordered_map<LaneId, std::shared_ptr<Lane>, LaneIdHasher>;

class LaneMap {
 public:
  static std::shared_ptr<Lane> GetLane(const LaneId &id);

 private:
  static LaneMapType lane_map_;          // 哈希表无序映射
  static LaneMapType *mutable_lane_map();

  LaneMap() = delete;
  LaneMap(const LaneMap &) = delete;
  LaneMap &operator=(const LaneMap &) = delete;

  friend class HdMapImpl;
};

}  // namespace hdmap
