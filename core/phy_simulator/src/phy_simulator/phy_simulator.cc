#include "phy_simulator/phy_simulator.h"

namespace phy_simulator {

PhySimulation::PhySimulation() {
  p_arena_loader_ = new ArenaLoader();
  if (!GetDataFromArenaLoader()) assert(false);
}

PhySimulation::PhySimulation(const std::string &vehicle_set_path,
                             const std::string &map_path,
                             const std::string &lane_net_path) {
  std::cout << "[PhySimulation] Constructing..." << std::endl;
  p_arena_loader_ = new ArenaLoader();

  p_arena_loader_->set_vehicle_set_path(vehicle_set_path);
  p_arena_loader_->set_map_path(map_path);
  p_arena_loader_->set_lane_net_path(lane_net_path);

  GetDataFromArenaLoader();         // 解析路径的文件，并存入对应的成员变量
  SetupVehicleModelForVehicleSet();
}

bool PhySimulation::GetDataFromArenaLoader() {
  std::cout << "[PhySimulation] Parsing simulation info..." << std::endl;
  p_arena_loader_->ParseVehicleSet(&vehicle_set_);
  p_arena_loader_->ParseMapInfo(&obstacle_set_);
  // p_arena_loader_->ParseLaneNetInfo(&lane_net_);
  p_arena_loader_->ParseLaneNetInfoFromXodr(&lane_net_);
  return true;
}

bool PhySimulation::AddTemporaryObstacleToMap(const common::Point &pt,
                                              const double &s) {
  common::PolygonObstacle obs;
  obs.polygon.points.push_back(common::Point(pt.x - s / 2, pt.y - s / 2));
  obs.polygon.points.push_back(common::Point(pt.x - s / 2, pt.y + s / 2));
  obs.polygon.points.push_back(common::Point(pt.x + s / 2, pt.y + s / 2));
  obs.polygon.points.push_back(common::Point(pt.x + s / 2, pt.y - s / 2));
  obs.type = 1;

  obs.id = temp_obs_idx_offset_ + temp_obstacle_cnt_;
  obstacle_set_.obs_polygon.insert(
      std::pair<int, common::PolygonObstacle>(obs.id, obs));

  temp_obstacle_set_.insert(
      std::pair<int, common::PolygonObstacle>(obs.id, obs));

  ++temp_obstacle_cnt_;
  return true;
}

bool PhySimulation::SetupVehicleModelForVehicleSet() {
  for (const auto &p : vehicle_set_.vehicles) {
    simulator::VehicleModel vehicle_model(
        p.second.param().wheel_base(), p.second.param().max_steering_angle());
    vehicle_model.set_state(p.second.state());
    vehicle_model_set_.insert(
        std::pair<int, simulator::VehicleModel>(p.first, vehicle_model));

    vehicle_ids_.push_back(p.first);
  }
  return true;
}
/** // ATTENTION 控制的前向模拟，更新vehicle_set_
 * @note 根据signal_set和dt向车辆施加控制信号，并更新所有车辆的状态，更新vehicle_set_
*/
bool PhySimulation::UpdateSimulatorUsingSignalSet(
    const common::VehicleControlSignalSet &signal_set, const decimal_t &dt) {
  TicToc updata_vehicle_time;
  UpdateVehicleStates(signal_set, dt);
  return true;
}
// 根据signal_set和dt向车辆施加控制信号，并更新车辆状态，更新vehicle_set_
bool PhySimulation::UpdateVehicleStates(
    const common::VehicleControlSignalSet &signal_set, const decimal_t &dt) {
  // 检查控制信号数量是否与车辆数量一致
  if (signal_set.signal_set.size() != vehicle_set_.vehicles.size()) {
    std::cerr << "[PhySimulation] ERROR - Signal number error." << std::endl;
    std::cerr << "[PhySimulation] signal_set num: "
              << signal_set.signal_set.size()
              << ", vehicle_set num: " << vehicle_set_.vehicles.size()
              << std::endl;
    assert(false);
  }
  // 遍历所有车辆，根据控制信号更新车辆状态
  for (auto iter = vehicle_set_.vehicles.begin();
       iter != vehicle_set_.vehicles.end(); ++iter) {
    int id = iter->first;
    common::VehicleControlSignal signal = signal_set.signal_set.at(id);
    decimal_t steer_rate = signal.steer_rate;
    decimal_t acc = signal.acc;

    auto model_iter = vehicle_model_set_.find(id);

    if (!signal.is_openloop) {
      // 如果该车是闭环模拟，则施加控制（转向角和加速度），并通过车辆模型更新车辆状态
      model_iter->second.set_control(
          simulator::VehicleModel::Control(steer_rate, acc));
      model_iter->second.Step(dt);
    } else {
      // 如果该车是开环模拟，则直接根据给出的下一个期望状态更新车辆状态
      model_iter->second.set_state(signal.state);
    }
    iter->second.set_state(model_iter->second.state());
  }
  return true;
}

bool PhySimulation::RemoveTemporaryObstacle(const common::Point &pt,
                                            const double &s) {
  for (auto it = temp_obstacle_set_.begin(); it != temp_obstacle_set_.end();) {
    decimal_t sum_x = 0, sum_y = 0;
    for (const auto &p : it->second.polygon.points) {
      sum_x += p.x;
      sum_y += p.y;
    }
    decimal_t center_x = sum_x / it->second.polygon.points.size();
    decimal_t center_y = sum_y / it->second.polygon.points.size();
    decimal_t dx = center_x - pt.x;
    decimal_t dy = center_y - pt.y;
    decimal_t d = std::hypot(dx, dy);

    if (d < s) {
      auto it_obs_set = obstacle_set_.obs_polygon.find(it->first);
      if (it_obs_set != obstacle_set_.obs_polygon.end()) {
        obstacle_set_.obs_polygon.erase(it_obs_set);
      }
      it = temp_obstacle_set_.erase(it);
    } else {
      ++it;
    }
  }
  return true;
}

}  // namespace phy_simulator