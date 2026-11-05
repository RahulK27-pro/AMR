#include <explore/costmap_tools.h>
#include <explore/frontier_search.h>

#include <geometry_msgs/msg/point.hpp>
#include <mutex>

#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_exploration
{
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

FrontierSearch::FrontierSearch(nav2_costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size,
                               double wall_bonus_scale, double momentum_scale,
                               double local_radius,
                               rclcpp::Logger logger)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size)
  , wall_bonus_scale_(wall_bonus_scale)
  , momentum_scale_(momentum_scale)
  , local_radius_(local_radius)
  , robot_heading_x_(1.0)
  , robot_heading_y_(0.0)
  , robot_pos_x_(0.0)
  , robot_pos_y_(0.0)
  , logger_(logger)
{
}

std::vector<Frontier>
FrontierSearch::searchFrom(geometry_msgs::msg::Point position)
{
  // Cache robot world position for scoring
  robot_pos_x_ = position.x;
  robot_pos_y_ = position.y;
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    RCLCPP_ERROR(logger_, "[FrontierSearch] Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_) ||
      nearestTraversableCell(clear, pos, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    RCLCPP_WARN(logger_, "[FrontierSearch] Could not find nearby clear cell to start search");
  }
  visited_flag[bfs.front()] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_)) {
      // add to queue all traversable, unvisited cells (cost < INSCRIBED_INFLATED_OBSTACLE)
      if (map_[nbr] < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
          map_[nbr] != NO_INFORMATION && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, traversable neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        if (new_frontier.size * costmap_->getResolution() >=
            min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  // set costs of frontiers
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::msg::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                               pow((double(reference_y) - double(wy)), 2.0));
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;

  // Compute safe_target in confirmed FREE_SPACE with obstacle clearance
  output.safe_target = output.middle;
  unsigned int mx, my;
  if (costmap_->worldToMap(output.middle.x, output.middle.y, mx, my)) {
    unsigned int middle_idx = costmap_->getIndex(mx, my);
    unsigned int clear_idx;
    if (nearestCell(clear_idx, middle_idx, FREE_SPACE, *costmap_) ||
        nearestTraversableCell(clear_idx, middle_idx, *costmap_)) {
      unsigned int cx, cy;
      costmap_->indexToCells(clear_idx, cx, cy);
      double wx, wy;
      costmap_->mapToWorld(cx, cy, wx, wy);
      output.safe_target.x = wx;
      output.safe_target.y = wy;

      // Inset only if robot is comfortably far away (d > 0.80m)
      // to avoid shrinking the goal inside the robot's goal tolerance footprint
      double dx = reference_x - wx;
      double dy = reference_y - wy;
      double d = std::sqrt(dx * dx + dy * dy);
      if (d > 0.80) {
        double step = 0.20;
        double cand_x = wx + (dx / d) * step;
        double cand_y = wy + (dy / d) * step;
        unsigned int cand_mx, cand_my;
        if (costmap_->worldToMap(cand_x, cand_y, cand_mx, cand_my)) {
          unsigned char cost = costmap_->getCost(cand_mx, cand_my);
          if (cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE && cost != NO_INFORMATION) {
            output.safe_target.x = cand_x;
            output.safe_target.y = cand_y;
          }
        }
      }
    }
  }

  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is traversable floor (cost < INSCRIBED_INFLATED_OBSTACLE)
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return true;
    }
  }

  return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  // 1. Distance in meters from robot to frontier
  double distance_m = frontier.min_distance;

  // 2. Physical length of frontier in meters
  double size_m = frontier.size * costmap_->getResolution();

  // 3. True metric cost:
  // Steep distance penalty ensures nearby room frontiers strictly beat distant ones.
  double dist_penalty = potential_scale_ * (distance_m * 2.0 + std::pow(distance_m, 1.4));

  // Cap size reward to a maximum (1.2m) so a huge doorway or hall opening in another room
  // cannot overpower a small 0.3-0.5m corner in the current room.
  double capped_size = std::min(size_m, 1.2);
  double gain_reward = gain_scale_ * capped_size;
  double base_cost = dist_penalty - gain_reward;

  // 4. Section/Room-First Local Priority:
  // If the frontier is within the local room radius (default 3.5m),
  // give a substantial negative bonus (-100.0) so it decisively beats ANY distant frontier!
  // This guarantees that the robot completes the current section/room before moving on.
  if (distance_m <= local_radius_) {
    base_cost -= 100.0;
  }

  // 5. Wall proximity bonus: frontier hugging a wall scores lower cost
  double wall_b = (wall_bonus_scale_ > 0.0) ? wallProximityBonus(frontier) * wall_bonus_scale_ : 0.0;

  // 6. Momentum bonus: frontier ahead of robot scores lower cost
  double mom_b = (momentum_scale_ > 0.0) ? momentumBonus(frontier) * momentum_scale_ : 0.0;

  return base_cost - wall_b - mom_b;
}

double FrontierSearch::wallProximityBonus(const Frontier& frontier)
{
  // Convert frontier centroid to map coordinates
  unsigned int cx, cy;
  if (!costmap_->worldToMap(frontier.centroid.x, frontier.centroid.y, cx, cy)) {
    return 0.0;
  }

  int lethal_count = 0;
  int total_count = 0;

  // Scan a square region of radius wall_check_radius_ around the centroid
  int min_x = std::max(0, (int)cx - wall_check_radius_);
  int max_x = std::min((int)size_x_ - 1, (int)cx + wall_check_radius_);
  int min_y = std::max(0, (int)cy - wall_check_radius_);
  int max_y = std::min((int)size_y_ - 1, (int)cy + wall_check_radius_);

  for (int ix = min_x; ix <= max_x; ++ix) {
    for (int iy = min_y; iy <= max_y; ++iy) {
      unsigned int idx = costmap_->getIndex(ix, iy);
      ++total_count;
      if (map_[idx] == LETHAL_OBSTACLE) {
        ++lethal_count;
      }
    }
  }

  if (total_count == 0) return 0.0;
  // Return fraction of lethal cells, capped at 1.0
  return std::min(1.0, (double)lethal_count / (double)total_count * 4.0);
}

double FrontierSearch::momentumBonus(const Frontier& frontier)
{
  // Do not penalize heading turns for local room frontiers
  if (frontier.min_distance <= local_radius_) {
    return 0.0;
  }

  // Direction vector from robot to frontier centroid
  double dx = frontier.centroid.x - robot_pos_x_;
  double dy = frontier.centroid.y - robot_pos_y_;
  double dist = std::sqrt(dx * dx + dy * dy);
  if (dist < 1e-6) return 0.0;

  // Normalise
  dx /= dist;
  dy /= dist;

  // Dot product with robot heading (cosine similarity: 1.0 = same direction)
  double dot = dx * robot_heading_x_ + dy * robot_heading_y_;

  // Mild penalty for 180-degree reversal between distant global corridors,
  // without locking out any direction:
  if (dot < 0.0) {
    return -2.0;
  }

  // Otherwise, scale [0, 1] for positive dot product
  return dot;
}
}  // namespace frontier_exploration
