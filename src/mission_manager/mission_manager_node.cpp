#include "mission_manager_node.hpp"

#include <cmath>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace open_mower_next::mission_manager
{

MissionManagerNode::MissionManagerNode(const rclcpp::NodeOptions & options)
: Node("mission_manager", options)
{
  target_area_id_ = declare_parameter<std::string>("target_area_id", "");
  battery_low_percent_ = declare_parameter<double>("battery_low_percent", 30.0);
  battery_charged_percent_ = declare_parameter<double>("battery_charged_percent", 95.0);
  autostart_ = declare_parameter<bool>("autostart", true);

  if (target_area_id_.empty()) {
    RCLCPP_WARN(get_logger(), "No target_area_id set - mission will not start until configured");
  }

  state_pub_ = create_publisher<std_msgs::msg::String>(
    "mission/state", rclcpp::QoS(10).transient_local());

  battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
    "/power", 10, std::bind(&MissionManagerNode::batteryCallback, this, _1));

  start_service_ = create_service<std_srvs::srv::Trigger>(
    "mission/start", std::bind(&MissionManagerNode::handleStartService, this, _1, _2));

  area_coverage_client_ = create_client<open_mower_next::srv::AreaCoverage>("/area_coverage");
  follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "/follow_path");
  dock_client_ = rclcpp_action::create_client<DockRobotNearest>(this, "/dock_robot_nearest");
  navigate_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

  tick_timer_ = create_wall_timer(1s, std::bind(&MissionManagerNode::tick, this));

  RCLCPP_INFO(
    get_logger(), "Mission manager started (target_area_id=%s, autostart=%d)",
    target_area_id_.c_str(), autostart_);
}

std::string MissionManagerNode::stateName(MissionState s)
{
  switch (s) {
    case MissionState::IDLE: return "IDLE";
    case MissionState::REQUESTING_COVERAGE: return "REQUESTING_COVERAGE";
    case MissionState::NAVIGATING_TO_START: return "NAVIGATING_TO_START";
    case MissionState::MOWING: return "MOWING";
    case MissionState::RETURNING_TO_DOCK: return "RETURNING_TO_DOCK";
    case MissionState::CHARGING: return "CHARGING";
    case MissionState::ERROR: return "ERROR";
  }
  return "UNKNOWN";
}

void MissionManagerNode::setState(MissionState new_state, const std::string & reason)
{
  state_ = new_state;
  RCLCPP_INFO(get_logger(), "Mission state -> %s (%s)", stateName(new_state).c_str(), reason.c_str());

  std_msgs::msg::String msg;
  msg.data = stateName(new_state);
  state_pub_->publish(msg);
}

void MissionManagerNode::handleStartService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (state_ != MissionState::IDLE && state_ != MissionState::ERROR) {
    response->success = false;
    response->message = "Mission already running (state=" + stateName(state_) + ")";
    return;
  }

  response->success = true;
  response->message = "Starting mission";
  setState(MissionState::REQUESTING_COVERAGE, "manual start");
  requestCoveragePath();
}

void MissionManagerNode::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  if (msg->percentage <= 1.0) {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "battery percentage looks 0-1 scale; mission_manager expects 0-100");
  }

  battery_percent_ = msg->percentage;
}

void MissionManagerNode::tick()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  switch (state_) {
    case MissionState::IDLE:
      if (autostart_ && !target_area_id_.empty()) {
        setState(MissionState::REQUESTING_COVERAGE, "autostart");
      }
      break;

    case MissionState::REQUESTING_COVERAGE:
      requestCoveragePath();  // no-op if already in flight; retries automatically otherwise
      break;

    case MissionState::NAVIGATING_TO_START:
      if (battery_percent_ <= battery_low_percent_ && !dock_goal_active_) {
        RCLCPP_WARN(get_logger(), "Battery at %.1f%% - heading to dock", battery_percent_);
        navigate_client_->async_cancel_all_goals();
        setState(MissionState::RETURNING_TO_DOCK, "battery low");
        sendDockGoal();
      }
      break;  // otherwise handled by navigate_to_pose result callback

    case MissionState::MOWING:
      if (battery_percent_ <= battery_low_percent_ && !dock_goal_active_) {
        RCLCPP_WARN(get_logger(), "Battery at %.1f%% - heading to dock", battery_percent_);
        if (follow_path_goal_active_) {
          follow_path_client_->async_cancel_all_goals();
          follow_path_goal_active_ = false;
        }
        setState(MissionState::RETURNING_TO_DOCK, "battery low");
        sendDockGoal();
      }
      break;

    case MissionState::RETURNING_TO_DOCK:
      break;  // handled by dock action result callback

    case MissionState::CHARGING:
      if (battery_percent_ >= battery_charged_percent_) {
        setState(MissionState::REQUESTING_COVERAGE, "charge complete");
      }
      break;

    case MissionState::ERROR:
      break;
  }
}

void MissionManagerNode::requestCoveragePath()
{
  if (coverage_in_flight_) {
    return;
  }

  if (!area_coverage_client_->wait_for_service(0s)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for /area_coverage service...");
    return;
  }

  coverage_in_flight_ = true;

  auto request = std::make_shared<open_mower_next::srv::AreaCoverage::Request>();
  request->area_id = target_area_id_;
  request->with_exclusions = true;
  request->headland_loops = 3;
  request->swath_angle = 0;

  area_coverage_client_->async_send_request(
    request,
    [this](rclcpp::Client<open_mower_next::srv::AreaCoverage>::SharedFuture future) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      coverage_in_flight_ = false;

      auto response = future.get();
      if (response->code != open_mower_next::srv::AreaCoverage::Response::CODE_SUCCESS) {
        RCLCPP_WARN(
          get_logger(), "Coverage request failed (code=%d, %s) - will retry",
          response->code, response->message.c_str());
        return;  // stays in REQUESTING_COVERAGE; tick() retries next second
      }

      RCLCPP_INFO(get_logger(), "Coverage path received with %zu poses", response->path.poses.size());
      path_segments_ = splitPathIntoSegments(response->path);
      removeReversalSpikes(path_segments_);
      fixSegmentEndpointOrientations(path_segments_);
      current_segment_index_ = 0;
      pending_path_ = path_segments_[current_segment_index_];
      navigate_retry_count_ = 0;
      setState(MissionState::NAVIGATING_TO_START, "coverage path ready");
      sendNavigateToStartGoal(pending_path_);
    });
}

std::vector<nav_msgs::msg::Path> MissionManagerNode::splitPathIntoSegments(
  const nav_msgs::msg::Path & path)
{
  constexpr double kMaxSegmentJump = 1.0;  // meters - must stay safely under local_costmap's
                                            // pruning tolerance (half its largest dimension)

  std::vector<nav_msgs::msg::Path> segments;
  if (path.poses.empty()) {
    return segments;
  }

  nav_msgs::msg::Path current;
  current.header = path.header;
  current.poses.push_back(path.poses.front());

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & prev = path.poses[i - 1].pose.position;
    const auto & curr = path.poses[i].pose.position;
    double dx = curr.x - prev.x;
    double dy = curr.y - prev.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist > kMaxSegmentJump) {
      segments.push_back(current);
      current = nav_msgs::msg::Path();
      current.header = path.header;
    }
    current.poses.push_back(path.poses[i]);
  }
  segments.push_back(current);

  constexpr size_t kMinSegmentPoses = 3;
  constexpr double kMinSegmentLength = 0.3;  // meters

  auto segmentLength = [](const nav_msgs::msg::Path & seg) {
    double total = 0.0;
    for (size_t i = 1; i < seg.poses.size(); ++i) {
      const auto & p0 = seg.poses[i - 1].pose.position;
      const auto & p1 = seg.poses[i].pose.position;
      double dx = p1.x - p0.x;
      double dy = p1.y - p0.y;
      total += std::sqrt(dx * dx + dy * dy);
    }
    return total;
  };

  auto isDegenerate = [&](const nav_msgs::msg::Path & seg) {
    return seg.poses.size() < kMinSegmentPoses || segmentLength(seg) < kMinSegmentLength;
  };

  auto posDist = [](const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  };

  // Merge degenerate segments (too few poses or too short) into the following segment so
  // follow_path's pure-pursuit controller always has a stable forward direction to steer
  // toward - a standalone 1-2 pose segment gives it nothing to orbit toward, not a heading.
  // If merging would itself re-introduce an intra-segment jump larger than kMaxSegmentJump,
  // drop the fragment instead of violating that invariant.
  std::vector<nav_msgs::msg::Path> merged;
  for (size_t i = 0; i < segments.size(); ++i) {
    if (isDegenerate(segments[i]) && i + 1 < segments.size()) {
      auto & next = segments[i + 1];
      double gap = posDist(segments[i].poses.back().pose.position, next.poses.front().pose.position);
      if (gap > kMaxSegmentJump) {
        RCLCPP_WARN(
          rclcpp::get_logger("mission_manager"),
          "Dropping degenerate segment fragment (%zu poses) - merge gap %.2fm exceeds max %.2fm",
          segments[i].poses.size(), gap, kMaxSegmentJump);
        continue;
      }
      next.poses.insert(next.poses.begin(), segments[i].poses.begin(), segments[i].poses.end());
      continue;
    }
    if (isDegenerate(segments[i]) && i + 1 == segments.size() && !merged.empty()) {
      auto & prev = merged.back();
      double gap = posDist(prev.poses.back().pose.position, segments[i].poses.front().pose.position);
      if (gap > kMaxSegmentJump) {
        RCLCPP_WARN(
          rclcpp::get_logger("mission_manager"),
          "Dropping degenerate segment fragment (%zu poses) - merge gap %.2fm exceeds max %.2fm",
          segments[i].poses.size(), gap, kMaxSegmentJump);
        continue;
      }
      prev.poses.insert(prev.poses.end(), segments[i].poses.begin(), segments[i].poses.end());
      continue;
    }
    merged.push_back(segments[i]);
  }
  segments = std::move(merged);

  RCLCPP_INFO(
    rclcpp::get_logger("mission_manager"), "Split coverage path into %zu segment(s)",
    segments.size());

  return segments;
}

void MissionManagerNode::removeReversalSpikes(std::vector<nav_msgs::msg::Path> & segments)
{
  constexpr double kSpikeLegMaxLength = 0.3;   // meters
  constexpr double kSpikeTurnMinDegrees = 150.0;

  for (auto & segment : segments) {
    bool removed_any = true;
    while (removed_any && segment.poses.size() > 2) {
      removed_any = false;

      for (size_t i = 1; i + 1 < segment.poses.size(); ++i) {
        const auto & prev = segment.poses[i - 1].pose.position;
        const auto & curr = segment.poses[i].pose.position;
        const auto & next = segment.poses[i + 1].pose.position;

        double in_dx = curr.x - prev.x, in_dy = curr.y - prev.y;
        double out_dx = next.x - curr.x, out_dy = next.y - curr.y;
        double in_len = std::sqrt(in_dx * in_dx + in_dy * in_dy);
        double out_len = std::sqrt(out_dx * out_dx + out_dy * out_dy);

        if (in_len < 1e-6 || out_len < 1e-6) {
          continue;
        }

        double in_yaw = std::atan2(in_dy, in_dx);
        double out_yaw = std::atan2(out_dy, out_dx);
        double turn = std::atan2(std::sin(out_yaw - in_yaw), std::cos(out_yaw - in_yaw));
        double turn_deg = std::abs(turn * 180.0 / M_PI);

        bool short_leg = (in_len < kSpikeLegMaxLength) || (out_len < kSpikeLegMaxLength);

        if (short_leg && turn_deg > kSpikeTurnMinDegrees) {
          RCLCPP_WARN(
            get_logger(),
            "Dropping spurious path vertex at index %zu (turn=%.1f deg, in=%.3fm, out=%.3fm)",
            i, turn_deg, in_len, out_len);
          segment.poses.erase(segment.poses.begin() + i);
          removed_any = true;
          break;  // restart scan since indices shifted
        }
      }
    }
  }
}

void MissionManagerNode::fixSegmentEndpointOrientations(std::vector<nav_msgs::msg::Path> & segments)
{
  auto yawToQuaternion = [](double yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(q);
  };

  for (auto & segment : segments) {
    if (segment.poses.size() < 2) {
      continue;  // no direction information available - leave as-is
    }

    auto & first = segment.poses.front();
    const auto & second = segment.poses[1];
    double entry_yaw = std::atan2(
      second.pose.position.y - first.pose.position.y,
      second.pose.position.x - first.pose.position.x);
    first.pose.orientation = yawToQuaternion(entry_yaw);

    auto & last = segment.poses.back();
    const auto & second_last = segment.poses[segment.poses.size() - 2];
    double exit_yaw = std::atan2(
      last.pose.position.y - second_last.pose.position.y,
      last.pose.position.x - second_last.pose.position.x);
    last.pose.orientation = yawToQuaternion(exit_yaw);
  }
}

void MissionManagerNode::advanceToNextSegment()
{
  ++current_segment_index_;

  if (current_segment_index_ >= path_segments_.size()) {
    RCLCPP_INFO(get_logger(), "All coverage segments complete - returning to dock");
    setState(MissionState::RETURNING_TO_DOCK, "coverage complete");
    sendDockGoal();
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Segment %zu/%zu complete - navigating to next segment start",
    current_segment_index_, path_segments_.size());
  pending_path_ = path_segments_[current_segment_index_];
  navigate_retry_count_ = 0;
  setState(MissionState::NAVIGATING_TO_START, "next segment");
  sendNavigateToStartGoal(pending_path_);
}

void MissionManagerNode::sendNavigateToStartGoal(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    RCLCPP_ERROR(get_logger(), "Coverage path has no poses - cannot navigate to start");
    setState(MissionState::ERROR, "empty coverage path");
    return;
  }

  if (!navigate_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(get_logger(), "/navigate_to_pose action server not available");
    setState(MissionState::ERROR, "navigate_to_pose server unavailable");
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = path.header.frame_id;
  goal.pose.header.stamp = now();
  goal.pose.pose = path.poses.front().pose;

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  options.goal_response_callback = [this](const NavigateToPoseGoalHandle::SharedPtr & goal_handle) {
    if (!goal_handle) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      RCLCPP_WARN(get_logger(), "navigate_to_pose goal was rejected");
      scheduleNavigateRetry();
    }
  };
  options.result_callback = [this](const NavigateToPoseGoalHandle::WrappedResult & result) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (state_ != MissionState::NAVIGATING_TO_START) {
      return;  // already moved on - ignore stale result
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Reached start of coverage path - beginning swath following");
      navigate_retry_count_ = 0;
      setState(MissionState::MOWING, "at coverage start");
      sendFollowPathGoal(pending_path_);
    } else {
      RCLCPP_WARN(
        get_logger(), "navigate_to_pose to coverage start failed (code=%d)",
        static_cast<int>(result.code));
      scheduleNavigateRetry();
    }
  };

  navigate_client_->async_send_goal(goal, options);
}

void MissionManagerNode::scheduleNavigateRetry()
{
  if (state_ != MissionState::NAVIGATING_TO_START) {
    return;
  }
  if (++navigate_retry_count_ > kMaxActionRetries) {
    RCLCPP_ERROR(get_logger(), "navigate_to_pose failed %d times - giving up", kMaxActionRetries);
    setState(MissionState::ERROR, "navigate_to_start repeatedly failed");
    return;
  }
  RCLCPP_WARN(
    get_logger(), "Retrying navigate_to_pose in 3s (attempt %d/%d)",
    navigate_retry_count_, kMaxActionRetries);
  navigate_retry_timer_ = create_wall_timer(
    3s, [this]() {
      navigate_retry_timer_->cancel();
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_ == MissionState::NAVIGATING_TO_START) {
        sendNavigateToStartGoal(pending_path_);
      }
    });
}

void MissionManagerNode::sendFollowPathGoal(const nav_msgs::msg::Path & path)
{
  if (!follow_path_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(get_logger(), "/follow_path action server not available");
    setState(MissionState::ERROR, "follow_path server unavailable");
    return;
  }

  FollowPath::Goal goal;
  goal.path = path;
  goal.controller_id = "FollowPath";

  auto options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  options.goal_response_callback = [this](const FollowPathGoalHandle::SharedPtr & goal_handle) {
    if (!goal_handle) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      follow_path_goal_active_ = false;
      RCLCPP_WARN(get_logger(), "follow_path goal was rejected");
      scheduleFollowPathRetry();
    }
  };
  options.result_callback = [this](const FollowPathGoalHandle::WrappedResult & result) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    follow_path_goal_active_ = false;

    if (state_ != MissionState::MOWING) {
      return;  // already moved on (e.g. battery-low cancel) - ignore stale result
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      follow_path_retry_count_ = 0;
      advanceToNextSegment();
    } else {
      RCLCPP_WARN(get_logger(), "follow_path did not succeed (code=%d)", static_cast<int>(result.code));
      scheduleFollowPathRetry();
    }
  };

  follow_path_goal_active_ = true;
  follow_path_client_->async_send_goal(goal, options);
}

void MissionManagerNode::scheduleFollowPathRetry()
{
  if (state_ != MissionState::MOWING) {
    return;
  }
  if (++follow_path_retry_count_ > kMaxActionRetries) {
    RCLCPP_ERROR(get_logger(), "follow_path failed %d times - giving up", kMaxActionRetries);
    setState(MissionState::ERROR, "follow_path repeatedly failed");
    return;
  }
  RCLCPP_WARN(
    get_logger(), "Retrying follow_path in 3s (attempt %d/%d)",
    follow_path_retry_count_, kMaxActionRetries);
  follow_path_retry_timer_ = create_wall_timer(
    3s, [this]() {
      follow_path_retry_timer_->cancel();
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_ == MissionState::MOWING) {
        sendFollowPathGoal(pending_path_);
      }
    });
}

void MissionManagerNode::sendDockGoal()
{
  if (!dock_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(get_logger(), "/dock_robot_nearest action server not available");
    setState(MissionState::ERROR, "dock server unavailable");
    return;
  }

  DockRobotNearest::Goal goal;

  auto options = rclcpp_action::Client<DockRobotNearest>::SendGoalOptions();
  options.goal_response_callback = [this](const DockRobotNearestGoalHandle::SharedPtr & goal_handle) {
    if (!goal_handle) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      dock_goal_active_ = false;
      RCLCPP_WARN(get_logger(), "dock_robot_nearest goal was rejected");
      scheduleDockRetry();
    }
  };
  options.result_callback = [this](const DockRobotNearestGoalHandle::WrappedResult & result) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    dock_goal_active_ = false;

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        result.result->code == DockRobotNearest::Result::CODE_SUCCESS)
    {
      RCLCPP_INFO(get_logger(), "Docked successfully - charging");
      dock_retry_count_ = 0;
      setState(MissionState::CHARGING, "docked");
    } else {
      RCLCPP_WARN(
        get_logger(), "Docking failed (code=%d): %s",
        result.result ? result.result->code : 999,
        result.result ? result.result->message.c_str() : "no result");
      scheduleDockRetry();
    }
  };

  dock_goal_active_ = true;
  dock_client_->async_send_goal(goal, options);
}

void MissionManagerNode::scheduleDockRetry()
{
  if (state_ != MissionState::RETURNING_TO_DOCK) {
    return;
  }
  if (++dock_retry_count_ > kMaxActionRetries) {
    RCLCPP_ERROR(get_logger(), "dock_robot_nearest failed %d times - giving up", kMaxActionRetries);
    setState(MissionState::ERROR, "docking repeatedly failed");
    return;
  }
  RCLCPP_WARN(
    get_logger(), "Retrying dock_robot_nearest in 3s (attempt %d/%d)",
    dock_retry_count_, kMaxActionRetries);
  dock_retry_timer_ = create_wall_timer(
    3s, [this]() {
      dock_retry_timer_->cancel();
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_ == MissionState::RETURNING_TO_DOCK) {
        sendDockGoal();
      }
    });
}

}  // namespace open_mower_next::mission_manager