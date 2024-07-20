/**
 * @file eudm_server_ros.cc
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-07-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "eudm_planner/eudm_server_ros.h"

namespace planning {

EudmPlannerServer::EudmPlannerServer(ros::NodeHandle nh, int ego_id)
    : nh_(nh), work_rate_(20.0), ego_id_(ego_id) {
  p_visualizer_ = new EudmPlannerVisualizer(nh, &bp_manager_, ego_id);
  p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
      config_.kInputBufferSize);
  task_.user_perferred_behavior = 0; // 初始化用户期望行为，为无行为
}

EudmPlannerServer::EudmPlannerServer(ros::NodeHandle nh, double work_rate,
                                     int ego_id)
    : nh_(nh), work_rate_(work_rate), ego_id_(ego_id) {
  p_visualizer_ = new EudmPlannerVisualizer(nh, &bp_manager_, ego_id);
  p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
      config_.kInputBufferSize);
  task_.user_perferred_behavior = 0;
}

void EudmPlannerServer::PushSemanticMap(const SemanticMapManager &smm) {
  if (p_input_smm_buff_) p_input_smm_buff_->try_enqueue(smm);
}

void EudmPlannerServer::PublishData() {
  p_visualizer_->PublishDataWithStamp(ros::Time::now());
}

void EudmPlannerServer::Init(const std::string &bp_config_path) {
  bp_manager_.Init(bp_config_path, work_rate_);
  joy_sub_ = nh_.subscribe("/joy", 10, &EudmPlannerServer::JoyCallback, this);
  nh_.param("use_sim_state", use_sim_state_, true);
  p_visualizer_->Init();
  p_visualizer_->set_use_sim_state(use_sim_state_);
}

void EudmPlannerServer::JoyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
  int msg_id;
  if (std::string("").compare(msg->header.frame_id) == 0) {
    msg_id = 0;
  } else {
    msg_id = std::stoi(msg->header.frame_id);
  }
  if (msg_id != ego_id_) return;
  // ~ buttons[2] --> 1 -->  lcl
  // ~ buttons[1] --> 1 -->  lcr
  // ~ buttons[3] --> 1 -->  +1m/s
  // ~ buttons[0] --> 1 -->  -1m/s
  if (msg->buttons[0] == 0 && msg->buttons[1] == 0 && msg->buttons[2] == 0 &&
      msg->buttons[3] == 0 && msg->buttons[4] == 0 && msg->buttons[5] == 0 &&
      msg->buttons[6] == 0)
    return;

  if (msg->buttons[2] == 1) {           // lcl
    if (task_.user_perferred_behavior != -1) {
      task_.user_perferred_behavior = -1;
    } else {
      task_.user_perferred_behavior = 0;
    }
  } else if (msg->buttons[1] == 1) {   // lcr
    if (task_.user_perferred_behavior != 1) {
      task_.user_perferred_behavior = 1;
    } else {
      task_.user_perferred_behavior = 0;
    }
  } else if (msg->buttons[3] == 1) {  // +1m/s
    task_.user_desired_vel = task_.user_desired_vel + 1.0;
  } else if (msg->buttons[0] == 1) {  // -1m/s
    task_.user_desired_vel = std::max(task_.user_desired_vel - 1.0, 0.0);
  } else if (msg->buttons[4] == 1) {
    task_.lc_info.forbid_lane_change_left = !task_.lc_info.forbid_lane_change_left;
  } else if (msg->buttons[5] == 1) {
    task_.lc_info.forbid_lane_change_right = !task_.lc_info.forbid_lane_change_right;
  } else if (msg->buttons[6] == 1) {
    task_.is_under_ctrl = !task_.is_under_ctrl;
  }
}

void EudmPlannerServer::Start() {
  // detach()函数用于将新创建的线程与当前线程分离，使其能够独立运行。
  std::thread(&EudmPlannerServer::MainThread, this).detach();
  task_.is_under_ctrl = true;
}

void EudmPlannerServer::MainThread() {
  using namespace std::chrono;
  system_clock::time_point current_start_time{system_clock::now()};
  system_clock::time_point next_start_time{current_start_time};
  const milliseconds interval{static_cast<int>(1000.0 / work_rate_)};
  while (true) {
    current_start_time = system_clock::now();
    next_start_time = current_start_time + interval;
    PlanCycleCallback();
    std::this_thread::sleep_until(next_start_time);
  }
}

void EudmPlannerServer::PlanCycleCallback() {
  if (p_input_smm_buff_ == nullptr) return; // 判断p_input_smm_buff_是否为空指针

  // 从p_input_smm_buff_中不断读取数据，存入smm_中，直到p_input_smm_buff_中没有数据为止
  bool has_updated_map = false;
  while (p_input_smm_buff_->try_dequeue(smm_)) {
    has_updated_map = true;
  }
  // 如果p_input_smm_buff_中没有新数据，则返回
  if (!has_updated_map) return;
  // 此行代码的效果是创建一个std::shared_ptr智能指针map_ptr，指向一个新分配的SemanticMapManager对象，该对象是通过拷贝smm_而创建的。
  auto map_ptr =
      std::make_shared<semantic_map_manager::SemanticMapManager>(smm_);
  
  decimal_t replan_duration = 1.0 / work_rate_; // 一次规划时长
  double stamp =
      std::floor(smm_.time_stamp() / replan_duration) * replan_duration;

  if (bp_manager_.Run(stamp, map_ptr, task_) == kSuccess) {
    // 如果行为规划成功，则把规划的动作使用set_ego_behavior，设置到smm中
    common::SemanticBehavior behavior;
    bp_manager_.ConstructBehavior(&behavior);
    smm_.set_ego_behavior(behavior);
  }

  if (has_callback_binded_) {
    private_callback_fn_(smm_);
  }

  PublishData();
}

void EudmPlannerServer::BindBehaviorUpdateCallback(
    std::function<int(const SemanticMapManager &)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

void EudmPlannerServer::set_user_desired_velocity(const decimal_t desired_vel) {
  task_.user_desired_vel = desired_vel;
}

decimal_t EudmPlannerServer::user_desired_velocity() const {
  return task_.user_desired_vel;
}

}  // namespace planning
