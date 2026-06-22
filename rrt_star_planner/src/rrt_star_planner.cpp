#include "rrt_star_planner/rrt_star_planner.hpp"
#include <chrono>
#include "nav2_util/node_utils.hpp"

namespace rrt_star_planner
{

// ════════════════════════════════════════════════════════════
//  Lifecycle
// ════════════════════════════════════════════════════════════

void RRTStarPlanner::configure(
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
  nav2_util::declare_parameter_if_not_declared(node, name_+".rewire_radius",   rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(node, name_+".lethal_threshold",rclcpp::ParameterValue(253));

  node->get_parameter(name_+".max_iterations",  max_iterations_);
  node->get_parameter(name_+".step_size",        step_size_);
  node->get_parameter(name_+".goal_tolerance",   goal_tolerance_);
  node->get_parameter(name_+".goal_bias",        goal_bias_);
  node->get_parameter(name_+".rewire_radius",    rewire_radius_);
  node->get_parameter(name_+".lethal_threshold", lethal_threshold_);

  RCLCPP_INFO(logger_,
    "RRTStarPlanner configured | max_iter=%d step=%.2fm tol=%.2fm bias=%.2f rewire=%.2fm",
    max_iterations_, step_size_, goal_tolerance_, goal_bias_, rewire_radius_);
}

void RRTStarPlanner::cleanup()    { costmap_ = nullptr; }
void RRTStarPlanner::activate()   { RCLCPP_INFO(logger_, "RRTStarPlanner activated"); }
void RRTStarPlanner::deactivate() { RCLCPP_INFO(logger_, "RRTStarPlanner deactivated"); }

// ════════════════════════════════════════════════════════════
//  createPlan  (Jazzy: has cancel_checker)
// ════════════════════════════════════════════════════════════

nav_msgs::msg::Path RRTStarPlanner::createPlan(
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
    RCLCPP_ERROR(logger_, "Start pose is outside costmap");
    return nav_msgs::msg::Path();
  }
  if (!worldToMap(goal.pose.position.x, goal.pose.position.y, gmx, gmy)) {
    RCLCPP_ERROR(logger_, "Goal pose is outside costmap");
    return nav_msgs::msg::Path();
  }
  if (isInCollision(gmx, gmy)) {
    RCLCPP_ERROR(logger_, "Goal pose is in collision");
    return nav_msgs::msg::Path();
  }

  std::vector<Node> tree;
  tree.reserve(static_cast<size_t>(max_iterations_) + 1);
  tree.push_back({start.pose.position.x, start.pose.position.y, -1, 0.0});

  std::mt19937 rng(
    static_cast<unsigned>(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::uniform_real_distribution<double> bias_dist(0.0, 1.0);

  Node   goal_node{goal.pose.position.x, goal.pose.position.y, -1, 0.0};
  int    best_goal_idx  = -1;
  double best_goal_cost = std::numeric_limits<double>::infinity();

  // ── RRT* main loop ─────────────────────────────────────────
  for (int iter = 0; iter < max_iterations_; ++iter) {

    if (cancel_checker()) {
      RCLCPP_INFO(logger_, "RRT* planning cancelled at iter %d", iter);
      return nav_msgs::msg::Path();
    }

    // 1. Sample (with goal bias)
    Node q_rand = (bias_dist(rng) < goal_bias_)
      ? goal_node : randomSample(min_x, max_x, min_y, max_y);

    // 2. Nearest node in tree
    int  nn_idx = nearestNode(tree, q_rand);
    Node q_new  = steer(tree[nn_idx], q_rand, step_size_);

    // 3. Collision check
    if (!isCollisionFree(tree[nn_idx], q_new)) continue;

    // 4. Near neighbours — adaptive radius r(n)
    double n    = static_cast<double>(tree.size());
    double r    = std::min(rewire_radius_,
                    rewire_radius_ * 2.0 * std::sqrt(std::log(n) / n));
    auto   near = nearNodes(tree, q_new, r);

    // 5. Best parent selection
    int    best_par  = nn_idx;
    double best_cost = tree[nn_idx].cost + distance(tree[nn_idx], q_new);
    for (int idx : near) {
      double c = tree[idx].cost + distance(tree[idx], q_new);
      if (c < best_cost && isCollisionFree(tree[idx], q_new)) {
        best_cost = c; best_par = idx;
      }
    }
    q_new.parent = best_par;
    q_new.cost   = best_cost;
    tree.push_back(q_new);
    int new_idx = static_cast<int>(tree.size()) - 1;

    // 6. Rewire near neighbours
    rewire(tree, new_idx, near);

    // 7. Goal check — keep best (lowest cost) path found
    if (distance(q_new, goal_node) <= goal_tolerance_
        && q_new.cost < best_goal_cost) {
      best_goal_cost = q_new.cost;
      best_goal_idx  = new_idx;
    }
  }
  // ── End loop ────────────────────────────────────────────────

  double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();

  if (best_goal_idx < 0) {
    RCLCPP_WARN(logger_,
      "RRT* failed to reach goal in %d iters (%.0fms). "
      "Try increasing max_iterations or step_size.",
      max_iterations_, ms);
    return nav_msgs::msg::Path();
  }
  RCLCPP_INFO(logger_,
    "RRT* path found | cost=%.3fm  nodes=%zu  time=%.0fms",
    best_goal_cost, tree.size(), ms);
  return buildPath(tree, best_goal_idx, goal);
}

// ════════════════════════════════════════════════════════════
//  Core RRT* algorithm methods
// ════════════════════════════════════════════════════════════

Node RRTStarPlanner::randomSample(double x0, double x1, double y0, double y1)
{
  static std::mt19937 rng{std::random_device{}()};
  std::uniform_real_distribution<double> dx(x0, x1), dy(y0, y1);
  return {dx(rng), dy(rng), -1, 0.0};
}

int RRTStarPlanner::nearestNode(const std::vector<Node> & tree, const Node & q)
{
  int best = 0;
  double bd = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
    double d = distance(tree[i], q);
    if (d < bd) { bd = d; best = i; }
  }
  return best;
}

Node RRTStarPlanner::steer(const Node & from, const Node & to, double s)
{
  double dx = to.x - from.x, dy = to.y - from.y;
  double d  = std::hypot(dx, dy);
  if (d <= s) return {to.x, to.y, -1, 0.0};
  return {from.x + dx/d*s, from.y + dy/d*s, -1, 0.0};
}

bool RRTStarPlanner::isCollisionFree(const Node & from, const Node & to)
{
  double dx = to.x - from.x, dy = to.y - from.y;
  int steps = std::max(1,
    static_cast<int>(std::hypot(dx, dy) / (costmap_->getResolution() * 0.5)));
  for (int i = 0; i <= steps; ++i) {
    double t = static_cast<double>(i) / steps;
    unsigned int mx, my;
    if (!worldToMap(from.x + t*dx, from.y + t*dy, mx, my)) return false;
    if (isInCollision(mx, my)) return false;
  }
  return true;
}

std::vector<int> RRTStarPlanner::nearNodes(
  const std::vector<Node> & tree, const Node & q, double r)
{
  std::vector<int> res;
  for (int i = 0; i < static_cast<int>(tree.size()); ++i)
    if (distance(tree[i], q) <= r) res.push_back(i);
  return res;
}

void RRTStarPlanner::rewire(
  std::vector<Node> & tree, int ni, const std::vector<int> & near)
{
  for (int idx : near) {
    if (idx == tree[ni].parent) continue;
    double c = tree[ni].cost + distance(tree[ni], tree[idx]);
    if (c < tree[idx].cost && isCollisionFree(tree[ni], tree[idx])) {
      tree[idx].parent = ni;
      tree[idx].cost   = c;
    }
  }
}

// ════════════════════════════════════════════════════════════
//  Helpers
// ════════════════════════════════════════════════════════════

bool RRTStarPlanner::worldToMap(
  double wx, double wy, unsigned int & mx, unsigned int & my)
{ return costmap_->worldToMap(wx, wy, mx, my); }

bool RRTStarPlanner::isInCollision(unsigned int mx, unsigned int my)
{ return static_cast<int>(costmap_->getCost(mx, my)) >= lethal_threshold_; }

double RRTStarPlanner::distance(const Node & a, const Node & b)
{ return std::hypot(a.x - b.x, a.y - b.y); }

nav_msgs::msg::Path RRTStarPlanner::buildPath(
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
    ps.header          = path.header;
    ps.pose.position.x = tree[idx[i]].x;
    ps.pose.position.y = tree[idx[i]].y;
    ps.pose.position.z = 0.0;
    if (i + 1 < idx.size()) {
      double yaw = std::atan2(
        tree[idx[i+1]].y - tree[idx[i]].y,
        tree[idx[i+1]].x - tree[idx[i]].x);
      ps.pose.orientation.z = std::sin(yaw / 2.0);
      ps.pose.orientation.w = std::cos(yaw / 2.0);
    } else {
      ps.pose.orientation = goal_pose.pose.orientation;
    }
    path.poses.push_back(ps);
  }
  path.poses.push_back(goal_pose);
  return path;
}

}  // namespace rrt_star_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rrt_star_planner::RRTStarPlanner, nav2_core::GlobalPlanner)
