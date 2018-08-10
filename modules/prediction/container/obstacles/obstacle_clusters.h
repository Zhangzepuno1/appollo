/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PREDICTION_CONTAINER_OBSTACLES_OBSTACLE_CLUSTERS_H_
#define MODULES_PREDICTION_CONTAINER_OBSTACLES_OBSTACLE_CLUSTERS_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/macro.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/prediction/proto/lane_graph.pb.h"

namespace apollo {
namespace prediction {

class ObstacleClusters {
 public:
  /**
   * @brief Remove all lane graphs
   */
  static void Init();

  /**
   * @brief Obtain a lane graph given a lane info and s
   * @param lane start s
   * @param lane total length
   * @param lane info
   * @return a corresponding lane graph
   */
  static const LaneGraph& GetLaneGraph(
      const double start_s, const double length,
      std::shared_ptr<const apollo::hdmap::LaneInfo> lane_info_ptr);

   /**
   * @brief Add an obstacle into clusters
   * @param obstacle id
   * @param lane id
   * @param lane s
   * @param lane l
   */
  static void AddObstacle(
      const int obstacle_id,
      const std::string& lane_id,
      const double lane_s,
      const double lane_l);

  /**
   * @brief Sort lane obstacles by lane s
   */
  static void SortObstacles();

  /**
   * @brief Get the forward nearest obstacle on lane sequence at s
   * @param Lane sequence
   * @param s offset in the first lane of the lane sequence
   * @param the forward obstacle on lane
   * @return If the forward obstacle is found
   */
  bool ForwardNearbyObstacle(
      const LaneSequence& lane_sequence,
      const int obstacle_id,
      const double obstacle_s,
      NearbyObstacle* const nearby_obstacle_ptr);

  /**
   * @brief Get the backward nearest obstacle on lane sequence at s
   * @param Lane sequence
   * @param s offset in the first lane of the lane sequence
   * @param the forward obstacle on lane
   * @return If the backward obstacle is found
   */
  bool BackwardNearbyObstacle(
    const LaneSequence& lane_sequence,
    const int obstacle_id,
    const double obstacle_s,
    NearbyObstacle* const nearby_obstacle_ptr);

 private:
  ObstacleClusters() = delete;

  static void Clear();

 private:
  static std::unordered_map<std::string, LaneGraph> lane_graphs_;
  // The obstacles on each lane are sorted by s
  static std::unordered_map<std::string,
                            std::vector<LaneObstacle>> lane_obstacles_;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_CONTAINER_OBSTACLES_OBSTACLE_CLUSTERS_H_
