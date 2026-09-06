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
                               rclcpp::Logger logger)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size)
  , wall_bonus_scale_(wall_bonus_scale)
  , momentum_scale_(momentum_scale)
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
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
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
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
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
  // that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  double base_cost =
      (potential_scale_ * frontier.min_distance * costmap_->getResolution()) -
      (gain_scale_ * frontier.size * costmap_->getResolution());

  // Subtract wall proximity bonus: frontier hugging a wall scores lower cost
  double wall_b = (wall_bonus_scale_ > 0.0) ? wallProximityBonus(frontier) * wall_bonus_scale_ : 0.0;

  // Subtract momentum bonus: frontier ahead of robot scores lower cost
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

  // STRICT ANTI-U-TURN POLICY:
  // If the frontier is behind the robot (>90 degrees), apply a massive penalty
  if (dot < 0.0) {
    return -1000.0; // Multiplied by momentum_scale (1.5), adds +1500 to cost!
  }

  // Otherwise, scale [0, 1] for positive dot product
  return dot;
}
}  // namespace frontier_exploration
