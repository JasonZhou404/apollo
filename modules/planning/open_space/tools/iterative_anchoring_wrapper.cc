/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/
#include <ctime>
#include "cyber/common/file.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"
#include "modules/planning/open_space/trajectory_smoother/iterative_anchoring_smoother.h"
namespace apollo {
namespace planning {

class IAObstacleContainer {
 public:
  IAObstacleContainer() = default;
  void AddVirtualObstacle(double* obstacle_x, double* obstacle_y,
                          int vertice_num) {
    std::vector<common::math::Vec2d> obstacle_vertices;
    for (int i = 0; i < vertice_num; i++) {
      common::math::Vec2d vertice(obstacle_x[i], obstacle_y[i]);
      obstacle_vertices.push_back(vertice);
    }
    obstacles_list_.emplace_back(obstacle_vertices);
  }

  const std::vector<std::vector<common::math::Vec2d>>&
  GetObstaclesVerticesVec() {
    return obstacles_list_;
  }

 private:
  std::vector<std::vector<common::math::Vec2d>> obstacles_list_;
};

class IAResultContainer {
 public:
  IAResultContainer() = default;
  void LoadResult() {
    hybrid_a_x_ = std::move(hybrid_a_result_.x);
    hybrid_a_y_ = std::move(hybrid_a_result_.y);
    hybrid_a_phi_ = std::move(hybrid_a_result_.phi);
    hybrid_a_v_ = std::move(hybrid_a_result_.v);
    hybrid_a_a_ = std::move(hybrid_a_result_.a);
    hybrid_a_kappa_ = std::move(hybrid_a_result_.steer);
    common::VehicleParam vehicle_param_ =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    double wheel_base = vehicle_param_.wheel_base();
    std::for_each(
        hybrid_a_kappa_.begin(), hybrid_a_kappa_.end(),
        [wheel_base](double kappa) { kappa = std::tan(kappa) / wheel_base; });
    for (const auto& traj_point : iterative_anchoring_result_) {
      iterative_achoring_x_.push_back(traj_point.path_point().x());
      iterative_achoring_y_.push_back(traj_point.path_point().y());
      iterative_achoring_phi_.push_back(traj_point.path_point().theta());
      iterative_achoring_v_.push_back(traj_point.v());
      iterative_achoring_a_.push_back(traj_point.a());
      iterative_achoring_kappa_.push_back(traj_point.path_point().kappa());
    }
  }
  std::vector<double>* get_hybrid_a_x() { return &hybrid_a_x_; }
  std::vector<double>* get_hybrid_a_y() { return &hybrid_a_y_; }
  std::vector<double>* get_hybrid_a_phi() { return &hybrid_a_phi_; }
  std::vector<double>* get_hybrid_a_v() { return &hybrid_a_v_; }
  std::vector<double>* get_hybrid_a_a() { return &hybrid_a_a_; }
  std::vector<double>* get_hybrid_a_kappa() { return &hybrid_a_kappa_; }
  HybridAStartResult* PrepareHybridAResult() { return &hybrid_a_result_; }
  double* get_hybrid_a_time() { return &hybrid_a_time_; }

  std::vector<double>* get_iterative_achoring_x() {
    return &iterative_achoring_x_;
  }
  std::vector<double>* get_iterative_achoring_y() {
    return &iterative_achoring_y_;
  }
  std::vector<double>* get_iterative_achoring_phi() {
    return &iterative_achoring_phi_;
  }
  std::vector<double>* get_iterative_achoring_v() {
    return &iterative_achoring_v_;
  }
  std::vector<double>* get_iterative_achoring_a() {
    return &iterative_achoring_a_;
  }
  std::vector<double>* get_iterative_achoring_kappa() {
    return &iterative_achoring_kappa_;
  }
  DiscretizedTrajectory* PrepareIterativeAnchoringResult() {
    return &iterative_anchoring_result_;
  }
  double* get_iterative_total_time() { return &iterative_total_time_; }
  double* get_path_smooth_time() { return &path_smooth_time_; }
  double* get_speed_opt_time() { return &speed_opt_time_; }

 private:
  std::vector<double> hybrid_a_x_;
  std::vector<double> hybrid_a_y_;
  std::vector<double> hybrid_a_phi_;
  std::vector<double> hybrid_a_v_;
  std::vector<double> hybrid_a_a_;
  std::vector<double> hybrid_a_kappa_;
  HybridAStartResult hybrid_a_result_;
  double hybrid_a_time_;

  std::vector<double> iterative_achoring_x_;
  std::vector<double> iterative_achoring_y_;
  std::vector<double> iterative_achoring_phi_;
  std::vector<double> iterative_achoring_v_;
  std::vector<double> iterative_achoring_a_;
  std::vector<double> iterative_achoring_kappa_;
  DiscretizedTrajectory iterative_anchoring_result_;
  double iterative_total_time_;
  double path_smooth_time_;
  double speed_opt_time_;
};

extern "C" {
HybridAStar* CreateHybridAPtr() {
  apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;
  CHECK(apollo::cyber::common::GetProtoFromFile(
      FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
      << "Failed to load open space config file "
      << FLAGS_planner_open_space_config_filename;
  return new HybridAStar(planner_open_space_config_);
}

IterativeAnchoringSmoother* CreateIterativeAnchoringPtr() {
  apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;
  CHECK(apollo::cyber::common::GetProtoFromFile(
      FLAGS_planner_open_space_config_filename, &planner_open_space_config_))
      << "Failed to load open space config file "
      << FLAGS_planner_open_space_config_filename;
  return new IterativeAnchoringSmoother(planner_open_space_config_);
}

IAObstacleContainer* IACreateObstaclesPtr() {
  return new IAObstacleContainer();
}

IAResultContainer* IACreateResultPtr() { return new IAResultContainer(); }

void IAAddVirtualObstacle(IAObstacleContainer* obstacles_ptr,
                          double* obstacle_x, double* obstacle_y,
                          int vertice_num) {
  obstacles_ptr->AddVirtualObstacle(obstacle_x, obstacle_y, vertice_num);
}

void LoadHybridAstarResultInEigen(HybridAStartResult* result,
                                  Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS) {
  // load Warm Start result(horizon is timestep number minus one)
  size_t horizon = result->x.size() - 1;
  xWS->resize(4, horizon + 1);
  uWS->resize(2, horizon);
  Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->x.data(), horizon + 1);
  Eigen::VectorXd y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->y.data(), horizon + 1);
  Eigen::VectorXd phi = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->phi.data(), horizon + 1);
  Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->v.data(), horizon + 1);
  Eigen::VectorXd steer = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->steer.data(), horizon);
  Eigen::VectorXd a =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(result->a.data(), horizon);
  xWS->row(0) = std::move(x);
  xWS->row(1) = std::move(y);
  xWS->row(2) = std::move(phi);
  xWS->row(3) = std::move(v);
  uWS->row(0) = std::move(steer);
  uWS->row(1) = std::move(a);
}

void CombineTrajectories(const std::vector<DiscretizedTrajectory>& traj_vec,
                         DiscretizedTrajectory* combined_traj) {
  combined_traj->clear();
  for (size_t i = 0; i < traj_vec.size(); ++i) {
    std::copy(traj_vec[i].begin(), traj_vec[i].end() - 1,
              std::back_inserter(*combined_traj));
  }
  combined_traj->push_back(traj_vec[traj_vec.size() - 1].back());
}

bool IterativeAnchoringSmoothing(
    IterativeAnchoringSmoother* IterativeAnchoring_ptr,
    const Eigen::MatrixXd& xWS, const double init_a, const double init_v,
    const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
    DiscretizedTrajectory* smoothed_trajectory, double* epoch_time,
    double* path_smooth_time, double* speed_opt_time) {
  const auto start_timestamp = std::chrono::system_clock::now();
  if (!IterativeAnchoring_ptr->TmpSmooth(
          xWS, init_a, init_v, obstacles_vertices_vec, smoothed_trajectory,
          path_smooth_time, speed_opt_time)) {
    return false;
  }
  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> time_diff = end_timestamp - start_timestamp;
  *epoch_time = time_diff.count() * 1000;
  return true;
}

bool IterativeAnchoringPlan(HybridAStar* hybridA_ptr,
                            IterativeAnchoringSmoother* IterativeAnchoring_ptr,
                            IAObstacleContainer* obstacles_ptr,
                            IAResultContainer* result_ptr, double sx, double sy,
                            double sphi, double ex, double ey, double ephi,
                            double* XYbounds) {
  HybridAStartResult hybrid_astar_result;
  std::vector<double> XYbounds_(XYbounds, XYbounds + 4);

  double hybrid_a_time = 0.0;
  const auto start_timestamp = std::chrono::system_clock::now();
  if (!hybridA_ptr->Plan(sx, sy, sphi, ex, ey, ephi, XYbounds_,
                         obstacles_ptr->GetObstaclesVerticesVec(),
                         &hybrid_astar_result)) {
    AINFO << "Hybrid A Star fails";
    return false;
  }
  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> time_diff = end_timestamp - start_timestamp;
  hybrid_a_time = time_diff.count() * 1000;

  std::vector<HybridAStartResult> partition_trajectories;
  if (!hybridA_ptr->TrajectoryPartition(hybrid_astar_result,
                                        &partition_trajectories)) {
    return false;
  }
  size_t size = partition_trajectories.size();
  std::vector<Eigen::MatrixXd> xWS_vec;
  std::vector<Eigen::MatrixXd> uWS_vec;
  std::vector<DiscretizedTrajectory> traj_vec;
  traj_vec.resize(size);
  xWS_vec.resize(size);
  uWS_vec.resize(size);

  double total_time = 0.0;
  double path_time = 0.0;
  double speed_time = 0.0;
  for (size_t i = 0; i < size; ++i) {
    LoadHybridAstarResultInEigen(&partition_trajectories[i], &xWS_vec[i],
                                 &uWS_vec[i]);
    double epoch_time = 0.0;
    double path_smooth_time = 0.0;
    double speed_opt_time = 0.0;
    if (!IterativeAnchoringSmoothing(
            IterativeAnchoring_ptr, xWS_vec[i], 0.0, 0.0,
            obstacles_ptr->GetObstaclesVerticesVec(), &traj_vec[i], &epoch_time,
            &path_smooth_time, &speed_opt_time)) {
      return false;
    }
    total_time += epoch_time;
    path_time += path_smooth_time;
    speed_time += speed_opt_time;
  }

  // combine trajectories
  DiscretizedTrajectory combined_traj;
  CombineTrajectories(traj_vec, &combined_traj);

  *(result_ptr->PrepareHybridAResult()) = hybrid_astar_result;
  *(result_ptr->PrepareIterativeAnchoringResult()) = combined_traj;
  *(result_ptr->get_hybrid_a_time()) = hybrid_a_time;
  *(result_ptr->get_iterative_total_time()) = total_time;
  *(result_ptr->get_path_smooth_time()) = path_time;
  *(result_ptr->get_speed_opt_time()) = speed_time;
  return true;
}

void IAGetResult(IAResultContainer* result_ptr, double* x, double* y,
                 double* phi, double* v, double* a, double* kappa,
                 double* opt_x, double* opt_y, double* opt_phi, double* opt_v,
                 double* opt_a, double* opt_kappa, size_t* hybrid_a_output_size,
                 size_t* iterative_anchoring_output_size, double* hybrid_a_time,
                 double* iterative_total_time, double* path_time,
                 double* speed_time) {
  result_ptr->LoadResult();
  size_t size_by_hybrid_astar = result_ptr->get_hybrid_a_x()->size();
  size_t size_by_iterative_anchoring =
      result_ptr->get_iterative_achoring_x()->size();
  hybrid_a_output_size[0] = size_by_hybrid_astar;
  iterative_anchoring_output_size[0] = size_by_iterative_anchoring;

  for (size_t i = 0; i < size_by_hybrid_astar; ++i) {
    x[i] = result_ptr->get_hybrid_a_x()->at(i);
    y[i] = result_ptr->get_hybrid_a_y()->at(i);
    phi[i] = result_ptr->get_hybrid_a_phi()->at(i);
    v[i] = result_ptr->get_hybrid_a_v()->at(i);
  }
  for (size_t i = 0; i + 1 < size_by_hybrid_astar; ++i) {
    a[i] = result_ptr->get_hybrid_a_a()->at(i);
    kappa[i] = result_ptr->get_hybrid_a_kappa()->at(i);
  }

  for (size_t i = 0; i < size_by_iterative_anchoring; ++i) {
    opt_x[i] = result_ptr->get_iterative_achoring_x()->at(i);
    opt_y[i] = result_ptr->get_iterative_achoring_y()->at(i);
    opt_phi[i] = result_ptr->get_iterative_achoring_phi()->at(i);
    opt_v[i] = result_ptr->get_iterative_achoring_v()->at(i);
  }
  for (size_t i = 0; i + 1 < size_by_iterative_anchoring; ++i) {
    opt_a[i] = result_ptr->get_iterative_achoring_a()->at(i);
    opt_kappa[i] = result_ptr->get_iterative_achoring_kappa()->at(i);
  }

  hybrid_a_time[0] = *(result_ptr->get_hybrid_a_time());
  iterative_total_time[0] = *(result_ptr->get_iterative_total_time());
  path_time[0] = *(result_ptr->get_path_smooth_time());
  speed_time[0] = *(result_ptr->get_speed_opt_time());
}
};

}  // namespace planning
}  // namespace apollo
