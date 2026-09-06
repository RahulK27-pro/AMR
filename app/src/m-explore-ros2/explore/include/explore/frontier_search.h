#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace frontier_exploration
{
/**
 * @brief Represents a frontier
 *
 */
struct Frontier {
  std::uint32_t size;
  double min_distance;
  double cost;
  geometry_msgs::msg::Point initial;
  geometry_msgs::msg::Point centroid;
  geometry_msgs::msg::Point middle;
  std::vector<geometry_msgs::msg::Point> points;
};

/**
 * @brief Thread-safe implementation of a frontier-search task for an input
 * costmap.
 */
class FrontierSearch
{
public:
  FrontierSearch() : logger_(rclcpp::get_logger("frontier_search")) {} // Default constructor for the logger

  /**
   * @brief Constructor for search task
   * @param costmap Reference to costmap data to search.
   */
  FrontierSearch(nav2_costmap_2d::Costmap2D* costmap, double potential_scale,
                 double gain_scale, double min_frontier_size,
                 double wall_bonus_scale, double momentum_scale,
                 rclcpp::Logger logger);

  /**
   * @brief Runs search implementation, outward from the start position
   * @param position Initial position to search from
   * @return List of frontiers, if any
   */
  std::vector<Frontier> searchFrom(geometry_msgs::msg::Point position);

  /**
   * @brief Update the robot's heading for momentum scoring.
   * @param hx  cos(yaw) component
   * @param hy  sin(yaw) component
   */
  void setRobotHeading(double hx, double hy)
  {
    robot_heading_x_ = hx;
    robot_heading_y_ = hy;
  }

protected:
  /**
   * @brief Starting from an initial cell, build a frontier from valid adjacent
   * cells
   * @param initial_cell Index of cell to start frontier building
   * @param reference Reference index to calculate position from
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return new frontier
   */
  Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                            std::vector<bool>& frontier_flag);

  /**
   * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate
   * for a new frontier.
   * @param idx Index of candidate cell
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return true if the cell is frontier cell
   */
  bool isNewFrontierCell(unsigned int idx,
                         const std::vector<bool>& frontier_flag);

  /**
   * @brief computes frontier cost
   * @details cost function is defined by potential_scale and gain_scale
   *
   * @param frontier frontier for which compute the cost
   * @return cost of the frontier
   */
  double frontierCost(const Frontier& frontier);

  /**
   * @brief Wall proximity bonus [0,1]: fraction of cells within
   *        wall_check_radius_ of the frontier centroid that are lethal.
   *        Higher value → frontier is hugging a wall → preferred.
   */
  double wallProximityBonus(const Frontier& frontier);

  /**
   * @brief Directional momentum bonus [0,1]: cosine similarity between
   *        the robot's current heading and the direction to the frontier.
   *        Higher value → frontier is ahead of the robot → preferred.
   */
  double momentumBonus(const Frontier& frontier);

private:
  nav2_costmap_2d::Costmap2D* costmap_;
  unsigned char* map_;
  unsigned int size_x_, size_y_;
  double potential_scale_, gain_scale_;
  double min_frontier_size_;
  double wall_bonus_scale_;
  double momentum_scale_;
  double robot_heading_x_;
  double robot_heading_y_;
  double robot_pos_x_;
  double robot_pos_y_;
  static constexpr int wall_check_radius_ = 15;  // cells (15 × 0.05m = 0.75m)
  rclcpp::Logger logger_;
};
}  // namespace frontier_exploration
#endif
