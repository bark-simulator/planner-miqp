// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef DYNAMIC_OCCUPANCY_HPP_
#define DYNAMIC_OCCUPANCY_HPP_

#include <boost/functional/hash.hpp>
#include "bark/geometry/polygon.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"
#include "bark/world/objects/agent.hpp"

namespace bark {
namespace models {
namespace behavior {

typedef enum OccupancyType : int {
  PREDICTION = 0,
  RULE_NO_RIGHT_PASSING = 1,
  RULE_SAFE_DISTANCE = 2,
} OccupancyType;

struct DynamicOccupancy {
  explicit DynamicOccupancy(int TrajLength,
                            const bark::world::objects::AgentId id,
                            const bark::geometry::Polygon& shape,
                            OccupancyType type, bool is_soft)
      : id(id), shape(shape), type(type), is_soft(is_soft) {
    int l = static_cast<int>(
        bark::models::dynamic::StateDefinition::MIN_STATE_SIZE);
    this->prediction.resize(TrajLength, l);
  }

  static std::size_t GetHash(const bark::world::objects::AgentId id,
                             const OccupancyType type) {
    // calculate hash using id and type
    std::size_t occupancy_hash = boost::hash_value(id);
    boost::hash_combine(occupancy_hash, type);
    return occupancy_hash;
  }

  Trajectory GetPrediction() const { return prediction; };
  bark::world::objects::AgentId GetAgentId() const { return id; };
  bark::geometry::Polygon GetShape() const { return shape; };
  OccupancyType GetOccupancyType() const { return type; };
  bool GetIsSoft() const { return is_soft; };

  bark::world::objects::AgentId id;
  bark::models::dynamic::Trajectory prediction;
  bark::geometry::Polygon shape;
  OccupancyType type;
  bool is_soft;
};

typedef std::shared_ptr<DynamicOccupancy> DynamicOccupancyPtr;

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // DYNAMIC_OCCUPANCY_HPP_
