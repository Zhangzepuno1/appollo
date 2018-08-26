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

#include "modules/prediction/evaluator/vehicle/dnn_evaluator.h"

#include <cmath>
#include <limits>
#include <numeric>

#include "modules/common/math/math_utils.h"
#include "modules/common/util/file.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/map/proto/map_lane.pb.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_util.h"
#include "modules/prediction/common/validation_checker.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using HDMapLane = apollo::hdmap::Lane;

namespace {

double ComputeMean(const std::vector<double>& nums, size_t start, size_t end) {
  int count = 0;
  double sum = 0.0;
  for (size_t i = start; i <= end && i < nums.size(); i++) {
    sum += nums[i];
    ++count;
  }
  return (count == 0) ? 0.0 : sum / count;
}

}  // namespace

DNNEvaluator::DNNEvaluator() {
  // TODO(kechxu) implement load model
}

void DNNEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  Clear();
  CHECK_NOTNULL(obstacle_ptr);
  CHECK_LE(LANE_FEATURE_SIZE, 4 * FLAGS_max_num_lane_point);

  int id = obstacle_ptr->id();
  if (!obstacle_ptr->latest_feature().IsInitialized()) {
    AERROR << "Obstacle [" << id << "] has no latest feature.";
    return;
  }

  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);
  if (!latest_feature_ptr->has_lane() ||
      !latest_feature_ptr->lane().has_lane_graph()) {
    ADEBUG << "Obstacle [" << id << "] has no lane graph.";
    return;
  }

  double speed = latest_feature_ptr->speed();

  LaneGraph* lane_graph_ptr =
      latest_feature_ptr->mutable_lane()->mutable_lane_graph();
  CHECK_NOTNULL(lane_graph_ptr);
  if (lane_graph_ptr->lane_sequence_size() == 0) {
    AERROR << "Obstacle [" << id << "] has no lane sequences.";
    return;
  }

  for (int i = 0; i < lane_graph_ptr->lane_sequence_size(); ++i) {
    LaneSequence* lane_sequence_ptr = lane_graph_ptr->mutable_lane_sequence(i);
    CHECK(lane_sequence_ptr != nullptr);
    std::vector<double> feature_values;
    ExtractFeatureValues(obstacle_ptr, lane_sequence_ptr, &feature_values);
    double probability = ComputeProbability(feature_values);

    double centripetal_acc_probability =
        ValidationChecker::ProbabilityByCentripedalAcceleration(
            *lane_sequence_ptr, speed);
    probability *= centripetal_acc_probability;
    lane_sequence_ptr->set_probability(probability);
  }

  if (FLAGS_prediction_offline_mode) {
    FeatureOutput::Insert(*latest_feature_ptr);
  }
}

void DNNEvaluator::Clear() {
  obstacle_feature_values_map_.clear();
}

void DNNEvaluator::ExtractFeatureValues(
    Obstacle* obstacle_ptr,
    LaneSequence* lane_sequence_ptr,
    std::vector<double>* feature_values) {
  int id = obstacle_ptr->id();
  std::vector<double> obstacle_feature_values;

  auto it = obstacle_feature_values_map_.find(id);
  if (it == obstacle_feature_values_map_.end()) {
    SetObstacleFeatureValues(obstacle_ptr, &obstacle_feature_values);
    obstacle_feature_values_map_[id] = obstacle_feature_values;
  } else {
    obstacle_feature_values = it->second;
  }

  if (obstacle_feature_values.size() != OBSTACLE_FEATURE_SIZE) {
    ADEBUG << "Obstacle [" << id << "] has fewer than "
           << "expected obstacle feature_values "
           << obstacle_feature_values.size() << ".";
    return;
  }

  std::vector<double> lane_feature_values;
  SetLaneFeatureValues(obstacle_ptr, lane_sequence_ptr, &lane_feature_values);
  if (lane_feature_values.size() != LANE_FEATURE_SIZE) {
    ADEBUG << "Obstacle [" << id << "] has fewer than "
           << "expected lane feature_values" << lane_feature_values.size()
           << ".";
    return;
  }

  std::vector<double> interaction_feature_values;
  SetInteractionFeatureValues(
      obstacle_ptr, lane_sequence_ptr, &interaction_feature_values);
  if (interaction_feature_values.size() != INTERACTION_FEATURE_SIZE) {
    ADEBUG << "Obstacle [" << id << "] has fewer than "
           << "expected interaction feature_values"
           << interaction_feature_values.size() << ".";
    return;
  }

  feature_values->insert(feature_values->end(),
                         obstacle_feature_values.begin(),
                         obstacle_feature_values.end());
  feature_values->insert(feature_values->end(),
                         lane_feature_values.begin(),
                         lane_feature_values.end());
  feature_values->insert(feature_values->end(),
                         interaction_feature_values.begin(),
                         interaction_feature_values.end());

  if (FLAGS_prediction_offline_mode) {
    SaveOfflineFeatures(lane_sequence_ptr, *feature_values);
  }
}

void DNNEvaluator::SetObstacleFeatureValues(
    Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
  feature_values->clear();
  feature_values->reserve(OBSTACLE_FEATURE_SIZE);

  std::vector<double> thetas;
  std::vector<double> lane_ls;
  std::vector<double> dist_lbs;
  std::vector<double> dist_rbs;
  std::vector<int> lane_types;
  std::vector<double> speeds;
  std::vector<double> timestamps;

  double duration = obstacle_ptr->timestamp() - FLAGS_prediction_duration;
  int count = 0;
  for (std::size_t i = 0; i < obstacle_ptr->history_size(); ++i) {
    const Feature& feature = obstacle_ptr->feature(i);
    if (!feature.IsInitialized()) {
      continue;
    }
    if (feature.timestamp() < duration) {
      break;
    }
    if (feature.has_lane() && feature.lane().has_lane_feature()) {
      thetas.push_back(feature.lane().lane_feature().angle_diff());
      lane_ls.push_back(feature.lane().lane_feature().lane_l());
      dist_lbs.push_back(feature.lane().lane_feature().dist_to_left_boundary());
      dist_rbs.push_back(
          feature.lane().lane_feature().dist_to_right_boundary());
      lane_types.push_back(feature.lane().lane_feature().lane_turn_type());
      timestamps.push_back(feature.timestamp());
      speeds.push_back(feature.speed());
      ++count;
    }
  }
  if (count <= 0) {
    return;
  }
  int curr_size = 5;
  int hist_size = obstacle_ptr->history_size();
  double theta_mean = ComputeMean(thetas, 0, hist_size - 1);
  double theta_filtered = ComputeMean(thetas, 0, curr_size - 1);
  double lane_l_mean = ComputeMean(lane_ls, 0, hist_size - 1);
  double lane_l_filtered = ComputeMean(lane_ls, 0, curr_size - 1);
  double speed_mean = ComputeMean(speeds, 0, hist_size - 1);

  double time_diff = timestamps.front() - timestamps.back();
  double dist_lb_rate = (timestamps.size() > 1)
                            ? (dist_lbs.front() - dist_lbs.back()) / time_diff
                            : 0.0;
  double dist_rb_rate = (timestamps.size() > 1)
                            ? (dist_rbs.front() - dist_rbs.back()) / time_diff
                            : 0.0;

  double delta_t = 0.0;
  if (timestamps.size() > 1) {
    delta_t =
        (timestamps.front() - timestamps.back()) / (timestamps.size() - 1);
  }
  double angle_curr = ComputeMean(thetas, 0, curr_size - 1);
  double angle_prev = ComputeMean(thetas, curr_size, 2 * curr_size - 1);
  double angle_diff =
      (hist_size >= 2 * curr_size) ? angle_curr - angle_prev : 0.0;

  double lane_l_curr = ComputeMean(lane_ls, 0, curr_size - 1);
  double lane_l_prev = ComputeMean(lane_ls, curr_size, 2 * curr_size - 1);
  double lane_l_diff =
      (hist_size >= 2 * curr_size) ? lane_l_curr - lane_l_prev : 0.0;

  double angle_diff_rate = 0.0;
  double lane_l_diff_rate = 0.0;
  if (delta_t > std::numeric_limits<double>::epsilon()) {
    angle_diff_rate = angle_diff / (delta_t * curr_size);
    lane_l_diff_rate = lane_l_diff / (delta_t * curr_size);
  }

  double acc = 0.0;
  if (static_cast<int>(speeds.size()) >= 3 * curr_size &&
      delta_t > std::numeric_limits<double>::epsilon()) {
    double speed_1 = ComputeMean(speeds, 0, curr_size - 1);
    double speed_2 = ComputeMean(speeds, curr_size, 2 * curr_size - 1);
    double speed_3 = ComputeMean(speeds, 2 * curr_size, 3 * curr_size - 1);
    acc = (speed_1 - 2 * speed_2 + speed_3) /
          (curr_size * curr_size * delta_t * delta_t);
  }

  double dist_lb_rate_curr = 0.0;
  if (hist_size >= 2 * curr_size &&
      delta_t > std::numeric_limits<double>::epsilon()) {
    double dist_lb_curr = ComputeMean(dist_lbs, 0, curr_size - 1);
    double dist_lb_prev = ComputeMean(dist_lbs, curr_size, 2 * curr_size - 1);
    dist_lb_rate_curr = (dist_lb_curr - dist_lb_prev) / (curr_size * delta_t);
  }

  double dist_rb_rate_curr = 0.0;
  if (hist_size >= 2 * curr_size &&
      delta_t > std::numeric_limits<double>::epsilon()) {
    double dist_rb_curr = ComputeMean(dist_rbs, 0, curr_size - 1);
    double dist_rb_prev = ComputeMean(dist_rbs, curr_size, 2 * curr_size - 1);
    dist_rb_rate_curr = (dist_rb_curr - dist_rb_prev) / (curr_size * delta_t);
  }

  // setup obstacle feature values
  feature_values->push_back(theta_filtered);
  feature_values->push_back(theta_mean);
  feature_values->push_back(theta_filtered - theta_mean);
  feature_values->push_back(angle_diff);
  feature_values->push_back(angle_diff_rate);

  feature_values->push_back(lane_l_filtered);
  feature_values->push_back(lane_l_mean);
  feature_values->push_back(lane_l_filtered - lane_l_mean);
  feature_values->push_back(lane_l_diff);
  feature_values->push_back(lane_l_diff_rate);

  feature_values->push_back(speed_mean);
  feature_values->push_back(acc);

  feature_values->push_back(dist_lbs.front());
  feature_values->push_back(dist_lb_rate);
  feature_values->push_back(dist_lb_rate_curr);

  feature_values->push_back(dist_rbs.front());
  feature_values->push_back(dist_rb_rate);
  feature_values->push_back(dist_rb_rate_curr);

  feature_values->push_back(lane_types.front() == 0 ? 1.0 : 0.0);
  feature_values->push_back(lane_types.front() == 1 ? 1.0 : 0.0);
  feature_values->push_back(lane_types.front() == 2 ? 1.0 : 0.0);
  feature_values->push_back(lane_types.front() == 3 ? 1.0 : 0.0);
}

void DNNEvaluator::SetLaneFeatureValues(Obstacle* obstacle_ptr,
    LaneSequence* lane_sequence_ptr, std::vector<double>* feature_values) {
  feature_values->clear();
  feature_values->reserve(LANE_FEATURE_SIZE);
  const Feature& feature = obstacle_ptr->latest_feature();
  if (!feature.IsInitialized()) {
    ADEBUG << "Obstacle [" << obstacle_ptr->id() << "] has no latest feature.";
    return;
  } else if (!feature.has_position()) {
    ADEBUG << "Obstacle [" << obstacle_ptr->id() << "] has no position.";
    return;
  }

  double heading = feature.velocity_heading();
  for (int i = 0; i < lane_sequence_ptr->lane_segment_size(); ++i) {
    if (feature_values->size() >= LANE_FEATURE_SIZE) {
      break;
    }
    const LaneSegment& lane_segment = lane_sequence_ptr->lane_segment(i);
    for (int j = 0; j < lane_segment.lane_point_size(); ++j) {
      if (feature_values->size() >= LANE_FEATURE_SIZE) {
        break;
      }
      const LanePoint& lane_point = lane_segment.lane_point(j);
      if (!lane_point.has_position()) {
        AERROR << "Lane point has no position.";
        continue;
      }
      double diff_x = lane_point.position().x() - feature.position().x();
      double diff_y = lane_point.position().y() - feature.position().y();
      double angle = std::atan2(diff_x, diff_y);
      feature_values->push_back(std::sin(angle - heading));
      feature_values->push_back(lane_point.relative_l());
      feature_values->push_back(lane_point.heading());
      feature_values->push_back(lane_point.angle_diff());
    }
  }

  std::size_t size = feature_values->size();
  while (size >= 4 && size < LANE_FEATURE_SIZE) {
    double heading_diff = feature_values->operator[](size - 4);
    double lane_l_diff = feature_values->operator[](size - 3);
    double heading = feature_values->operator[](size - 2);
    double angle_diff = feature_values->operator[](size - 1);
    feature_values->push_back(heading_diff);
    feature_values->push_back(lane_l_diff);
    feature_values->push_back(heading);
    feature_values->push_back(angle_diff);
    size = feature_values->size();
  }
}

void DNNEvaluator::SetInteractionFeatureValues(Obstacle* obstacle_ptr,
    LaneSequence* lane_sequence_ptr, std::vector<double>* feature_values) {
  // forward / backward: relative_s, relative_l, speed, length
  feature_values->clear();
  // Initialize forward and backward obstacles
  NearbyObstacle forward_obstacle;
  NearbyObstacle backward_obstacle;
  forward_obstacle.set_s(1000.0);  // TODO(kechxu) move to gflags
  forward_obstacle.set_l(10.0);  // TODO(kechxu) move to gflags
  backward_obstacle.set_s(-1000.0);  // TODO(kechxu) move to gflags
  backward_obstacle.set_l(10.0);  // TODO(kechxu) move to gflags

  for (const auto& nearby_obstacle : lane_sequence_ptr->nearby_obstacle()) {
    if (nearby_obstacle.s() < 0.0) {
      if (nearby_obstacle.s() > backward_obstacle.s()) {
        backward_obstacle.set_id(nearby_obstacle.id());
        backward_obstacle.set_s(nearby_obstacle.s());
        backward_obstacle.set_l(nearby_obstacle.l());
      }
    } else {
      if (nearby_obstacle.s() < forward_obstacle.s()) {
        forward_obstacle.set_id(nearby_obstacle.id());
        forward_obstacle.set_s(nearby_obstacle.s());
        forward_obstacle.set_l(nearby_obstacle.l());
      }
    }
  }

  ObstaclesContainer* obstacles_container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::PERCEPTION_OBSTACLES));
  // Set feature values for forward obstacle
  feature_values->push_back(forward_obstacle.s());
  feature_values->push_back(forward_obstacle.l());
  if (!forward_obstacle.has_id()) {  // no forward obstacle
    feature_values->push_back(100.0);
    feature_values->push_back(0.0);
  } else {
    Obstacle* forward_obs_ptr =
        obstacles_container->GetObstacle(forward_obstacle.id());
    const Feature& feature = forward_obs_ptr->latest_feature();
    feature_values->push_back(feature.length());
    feature_values->push_back(feature.speed());
  }

  // Set feature values for backward obstacle
  feature_values->push_back(backward_obstacle.s());
  feature_values->push_back(backward_obstacle.l());
  if (!backward_obstacle.has_id()) {  // no forward obstacle
    feature_values->push_back(100.0);
    feature_values->push_back(0.0);
  } else {
    Obstacle* backward_obs_ptr =
        obstacles_container->GetObstacle(backward_obstacle.id());
    const Feature& feature = backward_obs_ptr->latest_feature();
    feature_values->push_back(feature.length());
    feature_values->push_back(feature.speed());
  }
}

double DNNEvaluator::ComputeProbability(
    const std::vector<double>& feature_values) {
  // TODO(kechxu) implement when model is trained
  return 0.1;
}

void DNNEvaluator::SaveOfflineFeatures(
    LaneSequence* sequence, const std::vector<double>& feature_values) {
  for (double feature_value : feature_values) {
    sequence->mutable_features()->add_mlp_features(feature_value);
  }
}

}  // namespace prediction
}  // namespace apollo
