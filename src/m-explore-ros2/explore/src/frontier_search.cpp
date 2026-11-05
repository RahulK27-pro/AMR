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
        if (new_frontier.size > 0 &&
            new_frontier.size * costmap_->getResolution() >= min_frontier_size_) {
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

  // Compute safe_target in confirmed FREE_SPACE with guaranteed obstacle clearance
  output.safe_target = output.middle;
  unsigned int mx, my;
  bool found_safe = false;

  if (costmap_->worldToMap(output.middle.x, output.middle.y, mx, my)) {
    unsigned int middle_idx = costmap_->getIndex(mx, my);
    unsigned int clear_idx;
    if (nearestCell(clear_idx, middle_idx, FREE_SPACE, *costmap_) ||
        nearestTraversableCell(clear_idx, middle_idx, *costmap_)) {
      unsigned int cx, cy;
      costmap_->indexToCells(clear_idx, cx, cy);
      double wx, wy;
      costmap_->mapToWorld(cx, cy, wx, wy);

      double dx = reference_x - wx;
      double dy = reference_y - wy;
      double d = std::sqrt(dx * dx + dy * dy);

      if (d > 0.15) {
        // Search along line towards robot for a point with at least 0.18m physical obstacle clearance.
        // Step distances back from frontier towards the robot's known-free location:
        const double step_candidates[] = {0.35, 0.50, 0.70, 0.20, 0.10, 0.0};
        int r_cells = static_cast<int>(0.18 / costmap_->getResolution());
        unsigned int size_x = costmap_->getSizeInCellsX();
        unsigned int size_y = costmap_->getSizeInCellsY();

        for (double s : step_candidates) {
          // Do not step past the robot
          if (s > d - 0.15) continue;

          double cand_x = wx + (dx / d) * s;
          double cand_y = wy + (dy / d) * s;
          unsigned int cand_mx, cand_my;
          if (!costmap_->worldToMap(cand_x, cand_y, cand_mx, cand_my)) continue;

          unsigned char center_cost = costmap_->getCost(cand_mx, cand_my);
          if (center_cost == NO_INFORMATION ||
              center_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            continue;
          }

          // Check circular footprint clearance from physical obstacles
          bool clear_of_obstacles = true;
          for (int rx = -r_cells; rx <= r_cells && clear_of_obstacles; ++rx) {
            for (int ry = -r_cells; ry <= r_cells; ++ry) {
              if (rx * rx + ry * ry > r_cells * r_cells) continue;
              int nx = static_cast<int>(cand_mx) + rx;
              int ny = static_cast<int>(cand_my) + ry;
              if (nx < 0 || nx >= static_cast<int>(size_x) || ny < 0 || ny >= static_cast<int>(size_y)) {
                clear_of_obstacles = false;
                break;
              }
              unsigned char cell_cost = costmap_->getCost(nx, ny);
              // Only physical obstacles (LETHAL or INSCRIBED) cause collisions.
              // NO_INFORMATION (255) is unknown space, not a physical obstacle.
              if (cell_cost != nav2_costmap_2d::NO_INFORMATION &&
                  cell_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                clear_of_obstacles = false;
                break;
              }
            }
          }

          if (clear_of_obstacles) {
            output.safe_target.x = cand_x;
            output.safe_target.y = cand_y;
            found_safe = true;
            break;
          }
        }
      }
    }
  }

  // If no safe target with open clearance could be found (e.g. inside an unreachable narrow slit),
  // invalidate the frontier so the robot never drives into walls or narrow crevices.
  if (!found_safe) {
    output.size = 0;
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

  // 3. Physical distance penalty:
  // Nearest-Frontier principle: travel distance across the warehouse
  // heavily penalizes jumping to distant rooms.
  double dist_penalty = potential_scale_ * (distance_m + 0.2 * std::pow(distance_m, 1.2));

  // 4. Information gain reward (moderated):
  // Gives preference to larger openings without overwhelming proximity.
  double gain_reward = gain_scale_ * std::min(size_m, 3.0);

  double base_cost = dist_penalty - gain_reward;

  // 5. STRICT LOCAL ROOM / ZONE CLUSTERING:
  // If frontier is within current room/local radius (distance <= local_radius_):
  // Apply a decisive cluster bonus (-60.0) so the robot MUST exhaust all local frontiers
  // in the current room before venturing out into other rooms.
  // Within the room, closer frontiers score lower cost (nearest-frontier traveling sweep).
  if (distance_m <= local_radius_) {
    double local_proximity = (1.0 - (distance_m / local_radius_));
    base_cost -= (60.0 + 10.0 * local_proximity);
  }

  // 6. Wall proximity bonus: frontier hugging a wall scores lower cost
  double wall_b = (wall_bonus_scale_ > 0.0) ? wallProximityBonus(frontier) * wall_bonus_scale_ : 0.0;

  // 7. Momentum bonus: when selecting between distant rooms, prefer continuing along heading
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
