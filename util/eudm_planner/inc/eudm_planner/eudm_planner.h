#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_

#include <algorithm>
#include <memory>
#include <string>
#include <thread>

#include "common/basics/basics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/mobil/mobil_model.h"
#include "common/state/state.h"
#include "eudm_config.pb.h"
#include "eudm_planner/dcp_tree.h"
#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/map_interface.h"
#include "forward_simulator/onlane_forward_simulation.h"

namespace planning {

class EudmPlanner : public Planner {
 public:
  using State = common::State;
  using Lane = common::Lane;
  using Behavior = common::SemanticBehavior;
  using LateralBehavior = common::LateralBehavior;
  using LongitudinalBehavior = common::LongitudinalBehavior;
  using DcpAction = DcpTree::DcpAction;
  using DcpLonAction = DcpTree::DcpLonAction;
  using DcpLatAction = DcpTree::DcpLatAction;
  using Cfg = planning::eudm::Config;
  using LaneChangeInfo = planning::eudm::LaneChangeInfo;

  enum class LatSimMode {
    kAlwaysLaneKeep = 0,
    kKeepThenChange,
    kAlwaysLaneChange,
    kChangeThenCancel
  };

  // * In forward simulation, ego agent and other agent have different
  // * properties, thus we design two different structure for them
  struct ForwardSimEgoAgent {
    // * constant
    decimal_t lat_range;

    // * update on scenario-level
    OnLaneForwardSimulation::Param sim_param; // 自车前向模拟所需的参数，包括idm模型参数

    LatSimMode seq_lat_mode;        // 横向动作模式，被分为4种，一直保持，保持后变道，一直变道，变道后取消
    common::LateralBehavior lat_behavior_longterm{LateralBehavior::kUndefined}; // 长期目标动作
    common::LateralBehavior seq_lat_behavior; // 自车的横向动作，由动作序列确定。如果动作以变道结束，则为变道；否则为保持
    bool is_cancel_behavior;                  // 横向动作是否被取消，如果被取消，则为true(被取消，即为横向动作后有车道保持动作)
    decimal_t operation_at_seconds{0.0};

    // * update on layer-level
    common::LongitudinalBehavior lon_behavior{LongitudinalBehavior::kMaintain};
    common::LateralBehavior lat_behavior{LateralBehavior::kUndefined};

    common::Lane current_lane;
    common::StateTransformer current_stf;
    common::Lane target_lane;
    common::StateTransformer target_stf;
    common::Lane longterm_lane;
    common::StateTransformer longterm_stf;

    Vec2i target_gap_ids;  // optional, gap ids are fixed in each layer, but gap
                           // is changing in each step

    // * update on step-level
    common::Vehicle vehicle;
  };

  struct ForwardSimAgent {
    int id = kInvalidAgentId;
    common::Vehicle vehicle;

    // * lon
    OnLaneForwardSimulation::Param sim_param;

    // * lat
    common::ProbDistOfLatBehaviors lat_probs;
    common::LateralBehavior lat_behavior{LateralBehavior::kUndefined};

    common::Lane lane;
    common::StateTransformer stf;

    // * other
    decimal_t lat_range;
  };

  struct ForwardSimAgentSet {
    std::unordered_map<int, ForwardSimAgent> forward_sim_agents;
  };

  struct EfficiencyCost {
    decimal_t ego_to_desired_vel = 0.0;
    decimal_t leading_to_desired_vel = 0.0;
    decimal_t ave() const {
      return (ego_to_desired_vel + leading_to_desired_vel) / 2.0;
    }
  };

  struct SafetyCost {
    decimal_t rss = 0.0;
    decimal_t occu_lane = 0.0;
    decimal_t ave() const { return (rss + occu_lane) / 2.0; }
  };

  struct NavigationCost {
    decimal_t lane_change_preference = 0.0;
    decimal_t ave() const { return lane_change_preference; }
  };

  struct CostStructure {
    // * associate cost with micro action using this index
    int valid_sample_index_ub;
    // * efficiency
    EfficiencyCost efficiency;
    // * safety
    SafetyCost safety;
    // * navigation
    NavigationCost navigation;
    decimal_t weight = 1.0;
    decimal_t ave() const {
      return (efficiency.ave() + safety.ave() + navigation.ave()) * weight;
    }

    friend std::ostream& operator<<(std::ostream& os,
                                    const CostStructure& cost) {
      os << std::fixed;
      os << std::fixed;
      os << std::setprecision(3);
      os << "(efficiency: "
         << "ego (" << cost.efficiency.ego_to_desired_vel << ") + leading ("
         << cost.efficiency.leading_to_desired_vel << "), safety: ("
         << cost.safety.rss << "," << cost.safety.occu_lane
         << "), navigation: " << cost.navigation.lane_change_preference << ")";
      return os;
    }
  };

  std::string Name() override;

  ErrorType Init(const std::string config) override;

  ErrorType RunOnce() override;

  void set_map_interface(EudmPlannerMapItf* itf);
  /**
   * @brief set desired velocity
   */
  void set_desired_velocity(const decimal_t desired_vel);

  void set_lane_change_info(const LaneChangeInfo& lc_info);

  ErrorType RunEudm();

  Behavior behavior() const;

  std::vector<DcpAction> winner_action_seq() const {
    return winner_action_seq_;
  }

  decimal_t desired_velocity() const;

  vec_E<vec_E<common::Vehicle>> forward_trajs() const { return forward_trajs_; }

  int winner_id() const;

  decimal_t time_cost() const;

  std::vector<bool> sim_res() const {
    std::vector<bool> ret;
    for (auto& r : sim_res_) {
      if (r == 0) {
        ret.push_back(false);
      } else {
        ret.push_back(true);
      }
    }
    return ret;
  }

  std::vector<bool> risky_res() const {
    std::vector<bool> ret;
    for (auto& r : risky_res_) {
      if (r == 0) {
        ret.push_back(false);
      } else {
        ret.push_back(true);
      }
    }
    return ret;
  }
  std::vector<std::string> sim_info() const { return sim_info_; }
  std::vector<decimal_t> final_cost() const { return final_cost_; }
  std::vector<std::vector<CostStructure>> progress_cost() const {
    return progress_cost_;
  }
  std::vector<CostStructure> tail_cost() const { return tail_cost_; }
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors() const {
    return forward_lat_behaviors_;
  }
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors() const {
    return forward_lon_behaviors_;
  }
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs()
      const {
    return surround_trajs_;
  }
  common::State plan_state() { return ego_vehicle_.state(); }
  std::vector<std::vector<DcpAction>> action_script() {
    return dcp_tree_ptr_->action_script();
  }

  const Cfg& cfg() const { return cfg_; }

  EudmPlannerMapItf* map_itf() const;

  void UpdateDcpTree(const DcpAction& ongoing_action);

  ErrorType ClassifyActionSeq(const std::vector<DcpAction>& action_seq,
                              decimal_t* operation_at_seconds,
                              common::LateralBehavior* lat_behavior,
                              bool* is_cancel_operation) const;

 private:
  ErrorType ReadConfig(const std::string config_path);

  ErrorType GetSimParam(const planning::eudm::ForwardSimDetail& cfg,
                        OnLaneForwardSimulation::Param* sim_param);

  ErrorType GetPotentialLaneIds(const int source_lane_id,
                                const LateralBehavior& beh,
                                std::vector<int>* candidate_lane_ids) const;
  ErrorType UpdateEgoLaneId(const int new_ego_lane_id);

  ErrorType JudgeBehaviorByLaneId(const int ego_lane_id_by_pos,
                                  LateralBehavior* behavior_by_lane_id);

  ErrorType UpdateEgoBehavior(const LateralBehavior& behavior_by_lane_id);

  ErrorType TranslateDcpActionToLonLatBehavior(const DcpAction& action,
                                               LateralBehavior* lat,
                                               LongitudinalBehavior* lon) const;

  ErrorType GetSurroundingForwardSimAgents(
      const common::SemanticVehicleSet& surrounding_semantic_vehicles,
      ForwardSimAgentSet* forward_sim_agents) const;

  // * simulation control loop
  ErrorType SimulateActionSequence(
      const common::Vehicle& ego_vehicle,
      const ForwardSimAgentSet& surrounding_fsagents,
      const std::vector<DcpAction>& action_seq, const int& seq_id);

  ErrorType SimulateScenario(
      const common::Vehicle& ego_vehicle,
      const ForwardSimAgentSet& surrounding_fsagents,
      const std::vector<DcpAction>& action_seq, const int& seq_id,
      const int& sub_seq_id, std::vector<int>* sub_sim_res,
      std::vector<int>* sub_risky_res, std::vector<std::string>* sub_sim_info,
      std::vector<std::vector<CostStructure>>* sub_progress_cost,
      std::vector<CostStructure>* sub_tail_cost,
      vec_E<vec_E<common::Vehicle>>* sub_forward_trajs,
      std::vector<std::vector<LateralBehavior>>* sub_forward_lat_behaviors,
      std::vector<std::vector<LongitudinalBehavior>>* sub_forward_lon_behaviors,
      vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>*
          sub_surround_trajs);

  ErrorType SimulateSingleAction(
      const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent_this_layer,
      const ForwardSimAgentSet& surrounding_fsagents_this_layer,
      vec_E<common::Vehicle>* ego_traj_multisteps,
      std::unordered_map<int, vec_E<common::Vehicle>>*
          surround_trajs_multisteps);

  // * evaluation functions
  ErrorType CostFunction(
      const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent,
      const ForwardSimAgentSet& other_fsagent,
      const vec_E<common::Vehicle>& ego_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
      bool verbose, CostStructure* cost, bool* is_risky,
      std::set<int>* risky_ids);

  ErrorType StrictSafetyCheck(
      const vec_E<common::Vehicle>& ego_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
      bool* is_safe, int* collided_id);

  ErrorType EvaluateSafetyStatus(const vec_E<common::Vehicle>& traj_a,
                                 const vec_E<common::Vehicle>& traj_b,
                                 decimal_t* cost, bool* is_rss_safe,
                                 int* risky_id);
  ErrorType EvaluateSinglePolicyTrajs(
      const std::vector<CostStructure>& progress_cost,
      const CostStructure& tail_cost, const std::vector<DcpAction>& action_seq,
      decimal_t* score);

  ErrorType EvaluateMultiThreadSimResults(int* winner_id,
                                          decimal_t* winner_cost);

  // * simulation util functions
  ErrorType UpdateSimSetupForScenario(const std::vector<DcpAction>& action_seq,
                                      ForwardSimEgoAgent* ego_fsagent) const;

  ErrorType UpdateSimSetupForLayer(const DcpAction& action,
                                   const ForwardSimAgentSet& other_fsagent,
                                   ForwardSimEgoAgent* ego_fsagent) const;

  ErrorType UpdateEgoBehaviorsUsingAction(
      const DcpAction& action, ForwardSimEgoAgent* ego_fsagent) const;

  bool CheckIfLateralActionFinished(const common::State& cur_state,
                                    const int& action_ref_lane_id,
                                    const LateralBehavior& lat_behavior,
                                    int* current_lane_id) const;

  ErrorType UpdateLateralActionSequence(
      const int cur_idx, std::vector<DcpAction>* action_seq) const;

  ErrorType PrepareMultiThreadContainers(const int n_sequence);

  ErrorType GetSimTimeSteps(const DcpAction& action,
                            std::vector<decimal_t>* dt_steps) const;

  ErrorType EgoAgentForwardSim(const ForwardSimEgoAgent& ego_fsagent,
                               const common::VehicleSet& all_sim_vehicles,
                               const decimal_t& sim_time_step,
                               common::State* state_out) const;

  ErrorType SurroundingAgentForwardSim(
      const ForwardSimAgent& fsagent,
      const common::VehicleSet& all_sim_vehicles,
      const decimal_t& sim_time_step, common::State* state_out) const;

  // * map
  EudmPlannerMapItf* map_itf_{nullptr};
  // * action
  DcpTree* dcp_tree_ptr_;
  // * setup
  Cfg cfg_;
  LaneChangeInfo lc_info_;
  decimal_t desired_velocity_{5.0};        // eudm的期望速度，实际上是从launch文件中读取的
  decimal_t sim_time_total_ = 0.0;
  std::set<int> pre_deleted_seq_ids_;      // 预删除动作序列的id的集合；记录不合理的动作序列的id，比如先向左变道又向右变道
  int ego_lane_id_{kInvalidLaneId};
  std::vector<int> potential_lcl_lane_ids_;
  std::vector<int> potential_lcr_lane_ids_;
  std::vector<int> potential_lk_lane_ids_;
  common::Lane rss_lane_;
  common::StateTransformer rss_stf_;
  common::RssChecker::RssConfig rss_config_;
  common::RssChecker::RssConfig rss_config_strict_as_front_;
  common::RssChecker::RssConfig rss_config_strict_as_rear_;

  OnLaneForwardSimulation::Param ego_sim_param_;    // 自车模拟所用的参数
  OnLaneForwardSimulation::Param agent_sim_param_;

  decimal_t time_stamp_;
  int ego_id_;
  common::Vehicle ego_vehicle_;

  // * result 储存前向模拟相关结果
  int winner_id_ = 0;
  decimal_t winner_score_ = 0.0;
  std::vector<DcpAction> winner_action_seq_;
  std::vector<int> sim_res_; // 记录前向模拟结果，0代表成功，1代表失败；为向量，表示每个动作序列的模拟结果
  std::vector<int> risky_res_; // 记录风险评估结果，0表示危险，1表示安全;实际上由CostFunction给出；感觉只是记录了，但是没有采取相应的措施？
  std::vector<std::string> sim_info_;
  std::vector<decimal_t> final_cost_;
  std::vector<std::vector<CostStructure>> progress_cost_; // 表示动作序列的cost；每个动作序列对应一个CostStructure结构体类型的向量，向量中每个元素表示动作序列中每个动作的cost
  std::vector<CostStructure> tail_cost_;                  // 尾部cost，但是该值没有计算
  vec_E<vec_E<common::Vehicle>> forward_trajs_;           // 自车前向模拟轨迹，每个轨迹点是common::Vehicle，除了车辆信息，还记录了位置、速度等信息
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors_;      // 主车的横向动作序列
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors_; // 主车的纵向动作序列，每个动作序列里的纵向决策是一样的
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_;
  decimal_t time_cost_ = 0.0;
};

}  // namespace planning

#endif  // _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_