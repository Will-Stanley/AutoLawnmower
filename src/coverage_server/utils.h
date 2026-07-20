#pragma once

#include "fields2cover/types/Path.h"
#include "fields2cover/types/Swaths.h"
#include "open_mower_next/msg/coverage_geometry.hpp"
#include "open_mower_next/msg/polygon_with_holes.hpp"

#include <tf2/LinearMath/Quaternion.hpp>

#include <cmath>

#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace open_mower_next::coverage_server::utils
{
inline geometry_msgs::msg::Point toMsg(const f2c::types::Point & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.getX();
  msg.y = point.getY();
  return msg;
}

inline geometry_msgs::msg::Point32 toPoint32Msg(const f2c::types::Point & point)
{
  geometry_msgs::msg::Point32 msg;
  msg.x = static_cast<float>(point.getX());
  msg.y = static_cast<float>(point.getY());
  return msg;
}

inline bool isValid(const geometry_msgs::msg::Polygon & polygon)
{
  if (polygon.points.size() < 3) {
    return false;
  }

  return true;
}

inline f2c::types::LinearRing toLinearRing(const geometry_msgs::msg::Polygon & polygon)
{
  f2c::types::LinearRing ring;
  for (const auto & point : polygon.points) {
    ring.addPoint(f2c::types::Point(point.x, point.y));
  }

  if (ring.isEmpty()) {
    return ring;
  }

  auto first_point = ring.at(0);
  auto last_point = ring.at(ring.size() - 1);

  // Add the first point again to close the loop if not already closed
  if (first_point.getX() != last_point.getX() || first_point.getY() != last_point.getY()) {
    ring.addPoint(first_point);
  }

  return ring;
}

inline f2c::types::LinearRing toLinearRing(const geometry_msgs::msg::PolygonStamped & polygon)
{
  return toLinearRing(polygon.polygon);
}

inline f2c::types::Cell toCell(const geometry_msgs::msg::PolygonStamped & polygon)
{
  f2c::types::Cell cell;

  const auto boundary_ring = toLinearRing(polygon);
  if (!boundary_ring.isEmpty()) {
    cell.addRing(boundary_ring);
  }

  return cell;
}

inline f2c::types::Cells toCells(
  const geometry_msgs::msg::PolygonStamped & boundary_polygon,
  const std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons)
{
  f2c::types::Cells cells{toCell(boundary_polygon)};

  for (const auto & exclusion : exclusion_polygons) {
    const auto exclusion_cell = toCell(exclusion);
    if (exclusion_cell.isEmpty() || cells.disjoint(exclusion_cell)) {
      continue;
    }

    cells = cells.difference(exclusion_cell);
  }

  return cells;
}

inline geometry_msgs::msg::Polygon toMsg(const f2c::types::LinearRing & ring)
{
  geometry_msgs::msg::Polygon msg;
  if (ring.isEmpty()) {
    return msg;
  }

  size_t point_count = ring.size();
  if (point_count > 1) {
    const auto first_point = ring.getGeometry(0);
    const auto last_point = ring.getGeometry(point_count - 1);
    if (first_point.getX() == last_point.getX() && first_point.getY() == last_point.getY()) {
      --point_count;
    }
  }

  for (size_t i = 0; i < point_count; ++i) {
    msg.points.push_back(toPoint32Msg(ring.getGeometry(i)));
  }

  return msg;
}

inline open_mower_next::msg::PolygonWithHoles toMsg(
  const f2c::types::Cell & cell, const std::string & frame_id)
{
  open_mower_next::msg::PolygonWithHoles msg;
  msg.header.frame_id = frame_id;

  if (cell.isEmpty() || cell.size() == 0) {
    return msg;
  }

  msg.exterior = toMsg(cell.getGeometry(0));
  for (size_t i = 1; i < cell.size(); ++i) {
    msg.holes.push_back(toMsg(cell.getGeometry(i)));
  }

  return msg;
}

inline open_mower_next::msg::CoverageGeometry toMsg(
  const f2c::types::Cells & cells, const std::string & frame_id)
{
  open_mower_next::msg::CoverageGeometry msg;
  msg.header.frame_id = frame_id;

  if (cells.isEmpty()) {
    return msg;
  }

  for (size_t i = 0; i < cells.size(); ++i) {
    const auto cell = cells.getGeometry(i);
    if (cell.isEmpty() || cell.size() == 0) {
      continue;
    }

    msg.cells.push_back(toMsg(cell, frame_id));
  }

  return msg;
}

inline geometry_msgs::msg::PoseStamped toMsg(
  const double x, const double y, const double yaw, const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = x;
  pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  pose.pose.orientation = tf2::toMsg(q);

  return pose;
}

// densified so downstream segment-splitting (kMaxSegmentJump) only triggers on true
// discontinuities, not sparse sampling
inline nav_msgs::msg::Path toMsg(const f2c::types::Swaths & swaths, const std::string & frame_id)
{
  nav_msgs::msg::Path msg;
  msg.header.frame_id = frame_id;
  for (const auto & swath : swaths) {
    const size_t n = swath.numPoints();
    for (size_t j = 0; j + 1 < n; ++j) {
      const auto & p = swath.getPoint(j);
      const auto & q = swath.getPoint(j + 1);
      double dx = q.getX() - p.getX();
      double dy = q.getY() - p.getY();
      double d = std::hypot(dx, dy);
      double yaw = std::atan2(dy, dx);
      int steps = std::max(1, static_cast<int>(std::ceil(d / 0.5)));
      for (int k = 0; k < steps; ++k) {
        double x = p.getX() + dx * (static_cast<double>(k) / steps);
        double y = p.getY() + dy * (static_cast<double>(k) / steps);
        msg.poses.push_back(toMsg(x, y, yaw, frame_id));
      }
    }
    if (n >= 2) {
      const auto & p = swath.getPoint(n - 2);
      const auto & q = swath.getPoint(n - 1);
      double yaw = std::atan2(q.getY() - p.getY(), q.getX() - p.getX());
      msg.poses.push_back(toMsg(q.getX(), q.getY(), yaw, frame_id));
    } else if (n == 1) {
      const auto & p = swath.getPoint(0);
      msg.poses.push_back(toMsg(p.getX(), p.getY(), 0.0, frame_id));
    }
  }
  return msg;
}

inline nav_msgs::msg::Path toMsg(const f2c::types::Path & path, const std::string & frame_id)
{
  nav_msgs::msg::Path msg;
  msg.header.frame_id = frame_id;

  for (const auto & state : path) {
    const geometry_msgs::msg::PoseStamped pose =
      toMsg(state.point.getX(), state.point.getY(), state.angle, frame_id);
    msg.poses.push_back(pose);
  }

  return msg;
}
}  // namespace open_mower_next::coverage_server::utils
