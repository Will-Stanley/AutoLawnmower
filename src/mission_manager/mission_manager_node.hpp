#pragma once

#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "open_mower_next/srv/area_coverage.hpp"
#include "open_mower_next/action/dock_robot_nearest.hpp"

namespace open_mower_next::mission_manager
{

enum class MissionState
{
  IDLE,
  REQUESTING_COVERAGE,
  NAVIGATING_TO_START,
  MOWING,
  RETURNING_TO_DOCK,
  CHARGING,
  ERROR
};

class MissionManagerNode : public rclcpp::Node
{
public:
  explicit MissionManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using FollowPath = nav2_msgs::action::FollowPath;
  using FollowPathGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  using DockRobotNearest = open_mower_next::action::DockRobotNearest;
  using DockRobotNearestGoalHandle = rclcpp_action::ClientGoalHandle<DockRobotNearest>;

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  std::string target_area_id_;
  double battery_low_percent_;
  double battery_charged_percent_;
  bool autostart_;

  std::mutex state_mutex_;
  MissionState state_ = MissionState::IDLE;
  double battery_percent_ = 100.0;

  bool coverage_in_flight_ = false;
  bool follow_path_goal_active_ = false;
  bool dock_goal_active_ = false;

  nav_msgs::msg::Path pending_path_;
  std::vector<nav_msgs::msg::Path> path_segments_;
  size_t current_segment_index_ = 0;

  static constexpr int kMaxActionRetries = 5;
  int navigate_retry_count_ = 0;
  int follow_path_retry_count_ = 0;
  int dock_retry_count_ = 0;

  rclcpp::TimerBase::SharedPtr navigate_retry_timer_;
  rclcpp::TimerBase::SharedPtr follow_path_retry_timer_;
  rclcpp::TimerBase::SharedPtr dock_retry_timer_;

  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;

  rclcpp::Client<open_mower_next::srv::AreaCoverage>::SharedPtr area_coverage_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
  rclcpp_action::Client<DockRobotNearest>::SharedPtr dock_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client_;

  void setState(MissionState new_state, const std::string & reason);
  static std::string stateName(MissionState s);

  void tick();
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void handleStartService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void requestCoveragePath();
  void sendFollowPathGoal(const nav_msgs::msg::Path & path);
  void sendNavigateToStartGoal(const nav_msgs::msg::Path & path);
  void sendDockGoal();

  std::vector<nav_msgs::msg::Path> splitPathIntoSegments(const nav_msgs::msg::Path & path);
  void removeReversalSpikes(std::vector<nav_msgs::msg::Path> & segments);
  void fixSegmentEndpointOrientations(std::vector<nav_msgs::msg::Path> & segments);
  void advanceToNextSegment();

  void scheduleNavigateRetry();
  void scheduleFollowPathRetry();
  void scheduleDockRetry();
};

}  // namespace open_mower_next::mission_manager