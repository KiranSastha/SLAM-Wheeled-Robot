#ifndef RRT_PLANNER__RRT_PLANNER_HPP_
#define RRT_PLANNER__RRT_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <random>
#include <cmath>
#include <limits>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace rrt_planner
{

struct Node {
  double x, y;
  int    parent;
};

class RRTPlanner : public nav2_core::GlobalPlanner
{
public:
  RRTPlanner() = default;
  ~RRTPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup()    override;
  void activate()   override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  Node randomSample(double min_x, double max_x, double min_y, double max_y);
  int  nearestNode(const std::vector<Node> & tree, const Node & q);
  Node steer(const Node & from, const Node & to, double step_size);
  bool isCollisionFree(const Node & from, const Node & to);
  bool   worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
  bool   isInCollision(unsigned int mx, unsigned int my);
  double distance(const Node & a, const Node & b);
  nav_msgs::msg::Path buildPath(
    const std::vector<Node> & tree, int goal_idx,
    const geometry_msgs::msg::PoseStamped & goal_pose);

  rclcpp_lifecycle::LifecycleNode::WeakPtr       node_;
  rclcpp::Logger                                 logger_{rclcpp::get_logger("RRTPlanner")};
  rclcpp::Clock::SharedPtr                       clock_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D *                   costmap_{nullptr};
  std::string name_, global_frame_;

  int    max_iterations_;
  double step_size_;
  double goal_tolerance_;
  double goal_bias_;
  int    lethal_threshold_;
};

}  // namespace rrt_planner
#endif
