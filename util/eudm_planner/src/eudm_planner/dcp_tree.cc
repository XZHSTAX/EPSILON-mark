/**
 * @file behavior_tree.cc
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-07-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "eudm_planner/dcp_tree.h"

namespace planning {
DcpTree::DcpTree(const int& tree_height, const decimal_t& layer_time)
    : tree_height_(tree_height), layer_time_(layer_time) {
  last_layer_time_ = layer_time_;
  GenerateActionScript();
}

DcpTree::DcpTree(const int& tree_height, const decimal_t& layer_time,
                 const decimal_t& last_layer_time)
    : tree_height_(tree_height),
      layer_time_(layer_time),
      last_layer_time_(last_layer_time) {
  GenerateActionScript();
}

ErrorType DcpTree::UpdateScript() { return GenerateActionScript(); }

  /**
   * @brief 在给定的序列后面添加n个相同的动作，并且返回新的序列
   * @param seq_in ：输入序列
   * @param a      ：要添加的动作
   * @param n      ：重复次数
   * @return std::vector<DcpAction>
  */
std::vector<DcpTree::DcpAction> DcpTree::AppendActionSequence(
    const std::vector<DcpAction>& seq_in, const DcpAction& a,
    const int& n) const {
  std::vector<DcpAction> seq = seq_in;
  for (int i = 0; i < n; ++i) {
    seq.push_back(a);
  }
  return seq;
}

/**
 * @brief 生成动作序列
 * @return ErrorType
 * 把生成的动作序列存储在action_script_中
 * action_script_是一个vector，其中的每个元素都是一个std::vector<DcpAction>，表示一个动作序列
 * 里面vector的每个元素都是一个DcpAction，表示一个动作
*/
ErrorType DcpTree::GenerateActionScript() {
  action_script_.clear();
  std::vector<DcpAction> ongoing_action_seq;
  // 根节点的横向动作不变，遍历所有纵向动作
  // 对于每个不同的根节点，向下探索所有可能的动作序列

  for (int lon = 0; lon < static_cast<int>(DcpLonAction::MAX_COUNT); lon++) {
    ongoing_action_seq.clear();
    ongoing_action_seq.push_back(
        DcpAction(DcpLonAction(lon), ongoing_action_.lat, ongoing_action_.t));

    for (int h = 1; h < tree_height_; ++h) {
      for (int lat = 0; lat < static_cast<int>(DcpLatAction::MAX_COUNT);
           lat++) {
        // 如果要添加的新动作的lat与ongoing_action的lat不同，则让余下的动作都变为新动作
        if (lat != static_cast<int>(ongoing_action_.lat)) {
          auto actions = AppendActionSequence(
              ongoing_action_seq,
              DcpAction(DcpLonAction(lon), DcpLatAction(lat), layer_time_),
              tree_height_ - h);
          action_script_.push_back(actions);
        }
      }
      // 如果要添加新动作与ongoing_action的lat相同，则拓展ongoing_action_seq 1次
      ongoing_action_seq.push_back(
          DcpAction(DcpLonAction(lon), ongoing_action_.lat, layer_time_));
    }
    action_script_.push_back(ongoing_action_seq);
  }
  // override the last layer time
  for (auto& action_seq : action_script_) {
    action_seq.back().t = last_layer_time_;
  }
  return kSuccess;
}

decimal_t DcpTree::planning_horizon() const {
  if (action_script_.empty()) return 0.0;
  decimal_t planning_horizon = 0.0;
  for (const auto& a : action_script_[0]) {
    planning_horizon += a.t;
  }
  return planning_horizon;
}

}  // namespace planning
