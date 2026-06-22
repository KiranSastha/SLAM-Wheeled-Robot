#include "rrt_planner/rrt_planner.hpp"
#include <chrono>
#include "nav2_util/node_utils.hpp"

namespace rrt_planner
{

void RRTPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_         = parent;
  name_         = name;
  costmap_ros_  = costmap_ros;
  costmap_      = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  auto node     = node_.lock();
  clock_        = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(node, name_+".max_iterations",  rclcpp::ParameterValue(3000));
  nav2_util::declare_parameter_if_not_declared(node, name_+".step_size",       rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(node, name_+".goal_tolerance",  rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, name_+".goal_bias",       rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(node, name_+".lethal_threshold",rclcpp::ParameterValue(253));

  node->get_parameter(name_+".max_iterations",  max_iterations_);
  node->get_parameter(name_+".step_size",        step_size_);
  node->get_parameter(name_+".goal_tolerance",   goal_tolerance_);
  node->get_parameter(name_+".goal_bias",        goal_bias_);
  node->get_parameter(name_+".lethal_threshold", lethal_threshold_);

  RCLCPP_INFO(logger_,
    "RRTPlanner configured | max_iter=%d step=%.2fm tol=%.2fm bias=%.2f",
    max_iterations_, step_size_, goal_tolerance_, goal_bias_);
}

void RRTPlanner::cleanup()    { costmap_ = nullptr; }
void RRTPlanner::activate()   { RCLCPP_INFO(logger_, "RRTPlanner activated"); }
void RRTPlanner::deactivate() { RCLCPP_INFO(logger_, "RRTPlanner deactivated"); }

nav_msgs::msg::Path RRTPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  auto t0 = std::chrono::steady_clock::now();
  costmap_ = costmap_ros_->getCostmap();

  double ox = costmap_->getOriginX(), oy = costmap_->getOriginY();
  double min_x = ox, max_x = ox + costmap_->getSizeInMetersX();
  double min_y = oy, max_y = oy + costmap_->getSizeInMetersY();

  unsigned int smx, smy, gmx, gmy;
  if (!worldToMap(start.pose.position.x, start.pose.position.y, smx, smy)) {
    RCLCPP_ERROR(logger_, "Start outside costmap");
    return nav_msgs::msg::Path();
  }
  if (!worldToMap(goal.pose.position.x, goal.pose.position.y, gmx, gmy)) {
    RCLCPP_ERROR(logger_, "Goal outside costmap");
    return nav_msgs::msg::Path();
  }
  if (isInCollision(gmx, gmy)) {
    RCLCPP_ERROR(logger_, "Goal in collision");
    return nav_msgs::msg::Path();
  }

  std::vector<Node> tree;
  tree.reserve(static_cast<size_t>(max_iterations_) + 1);
  tree.push_back({start.pose.position.x, start.pose.position.y, -1});

  std::mt19937 rng(
    static_cast<unsigned>(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::uniform_real_distribution<double> bias_dist(0.0, 1.0);

  Node goal_node{goal.pose.position.x, goal.pose.position.y, -1};

  for (int iter = 0; iter < max_iterations_; ++iter) {

    if (cancel_checker()) {
      RCLCPP_INFO(logger_, "RRT cancelled at iter %d", iter);
      return nav_msgs::msg::Path();
    }

    Node q_rand = (bias_dist(rng) < goal_bias_)
      ? goal_node : randomSample(min_x, max_x, min_y, max_y);

    int  nn_idx = nearestNode(tree, q_rand);
    Node q_new  = steer(tree[nn_idx], q_rand, step_size_);

    if (!isCollisionFree(tree[nn_idx], q_new)) continue;

    q_new.parent = nn_idx;
    tree.push_back(q_new);
    int new_idx = static_cast<int>(tree.size()) - 1;

    if (distance(q_new, goal_node) <= goal_tolerance_) {
      double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
      RCLCPP_INFO(logger_, "RRT path found | nodes=%zu  time=%.0fms", tree.size(), ms);
      return buildPath(tree, new_idx, goal);
    }
  }

  double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();
  RCLCPP_WARN(logger_,
    "RRT failed in %d iters (%.0fms). Try increasing max_iterations.",
    max_iterations_, ms);
  return nav_msgs::msg::Path();
}

Node RRTPlanner::randomSample(double x0, double x1, double y0, double y1)
{
  static std::mt19937 rng{std::random_device{}()};
  std::uniform_real_distribution<double> dx(x0,x1), dy(y0,y1);
  return {dx(rng), dy(rng), -1};
}

int RRTPlanner::nearestNode(const std::vector<Node> & tree, const Node & q)
{
  int best = 0; double bd = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
    double d = distance(tree[i], q);
    if (d < bd) { bd = d; best = i; }
  }
  return best;
}

Node RRTPlanner::steer(const Node & from, const Node & to, double s)
{
  double dx = to.x-from.x, dy = to.y-from.y, d = std::hypot(dx,dy);
  if (d <= s) return {to.x, to.y, -1};
  return {from.x+dx/d*s, from.y+dy/d*s, -1};
}

bool RRTPlanner::isCollisionFree(const Node & from, const Node & to)
{
  double dx = to.x-from.x, dy = to.y-from.y;
  int steps = std::max(1,
    static_cast<int>(std::hypot(dx,dy)/(costmap_->getResolution()*0.5)));
  for (int i = 0; i <= steps; ++i) {
    double t = static_cast<double>(i)/steps;
    unsigned int mx, my;
    if (!worldToMap(from.x+t*dx, from.y+t*dy, mx, my)) return false;
    if (isInCollision(mx, my)) return false;
  }
  return true;
}

bool RRTPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{ return costmap_->worldToMap(wx, wy, mx, my); }

bool RRTPlanner::isInCollision(unsigned int mx, unsigned int my)
{ return static_cast<int>(costmap_->getCost(mx,my)) >= lethal_threshold_; }

double RRTPlanner::distance(const Node & a, const Node & b)
{ return std::hypot(a.x-b.x, a.y-b.y); }

nav_msgs::msg::Path RRTPlanner::buildPath(
  const std::vector<Node> & tree, int goal_idx,
  const geometry_msgs::msg::PoseStamped & goal_pose)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp    = clock_->now();

  std::vector<int> idx;
  for (int i = goal_idx; i >= 0; i = tree[i].parent) idx.push_back(i);
  std::reverse(idx.begin(), idx.end());

  for (size_t i = 0; i < idx.size(); ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = tree[idx[i]].x;
    ps.pose.position.y = tree[idx[i]].y;
    ps.pose.position.z = 0.0;
    if (i + 1 < idx.size()) {
      double yaw = std::atan2(
        tree[idx[i+1]].y - tree[idx[i]].y,
        tree[idx[i+1]].x - tree[idx[i]].x);
      ps.pose.orientation.z = std::sin(yaw/2.0);
      ps.pose.orientation.w = std::cos(yaw/2.0);
    } else {
      ps.pose.orientation = goal_pose.pose.orientation;
    }
    path.poses.push_back(ps);
  }
  path.poses.push_back(goal_pose);
  return path;
}

}  // namespace rrt_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav2_core::GlobalPlanner)
