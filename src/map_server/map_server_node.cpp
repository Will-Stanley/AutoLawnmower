#include "map_server/map_server_node.hpp"
#include "map_server/polygon_iterator.hpp"
#include "map_server/polygon_utils.hpp"
#include "map_server/geo_json_map.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <cmath>

namespace open_mower_next::map_server
{
MapServerNode::MapServerNode(const rclcpp::NodeOptions& options) : rclcpp::Node("map_server_node", options)
{
  configureServices();
  configureMap();
  configureGaussianBlur();

  declare_parameter("world_frame", "map");

  auto occupancy_grid_topic_name_ = declare_parameter("grid.topic_name", "map_grid");
  occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      occupancy_grid_topic_name_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  auto map_topic_name_ = declare_parameter("topic_name", "map");
  map_publisher_ =
      this->create_publisher<msg::Map>(map_topic_name_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  auto visualization_topic_name_ = declare_parameter("visualization.topic_name", "map_visualization");
  map_visualization_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      visualization_topic_name_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  auto docking_stations_topic_name = declare_parameter("docking_stations.topic_name", "docking_stations_poses");
  docking_station_poses_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      docking_stations_topic_name, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Must match the robot_radius used by the nav2 costmaps - the keep-in wall is
  // buffered outward by this much so the robot's center (not its footprint) can
  // reach the true operation/navigation boundary.
  robot_radius_ = declare_parameter("grid.robot_radius", 0.48);

  grid_resolution_ = declare_parameter("grid.resolution", 0.1);
  grid_max_size_ = declare_parameter("grid.max_size", 2000);

  publishMap();
}

void MapServerNode::publishMap()
{
  current_map_ = map_io_->load();

  // log areas
  RCLCPP_INFO(get_logger(), "Loaded %lu areas", current_map_.areas.size());
  RCLCPP_INFO(get_logger(), "%-10s %-20s %-10s %10s", "ID", "Name", "Type", "Area");

  for (const auto& area : current_map_.areas)
  {
    std::string type = area.type == msg::Area::TYPE_NAVIGATION ?
                           "navigation" :
                           (area.type == msg::Area::TYPE_OPERATION ? "operation" : "exclusion");

    auto calculated = polygonArea(area.area.polygon);

    RCLCPP_INFO(get_logger(), "%-10s %-20s %-10s %10.2fsqmt", area.id.c_str(), area.name.c_str(), type.c_str(),
                calculated);
  }

  // log docks
  RCLCPP_INFO(get_logger(), "Loaded %lu docks", current_map_.docking_stations.size());
  RCLCPP_INFO(get_logger(), "%-10s %-20s", "ID", "Name");

  for (const auto& dock : current_map_.docking_stations)
  {
    RCLCPP_INFO(get_logger(), "%-10s %-20s", dock.id.c_str(), dock.name.c_str());
  }

  RCLCPP_INFO(get_logger(), "Publishing map");
  map_publisher_->publish(current_map_);

  // Always publish occupancy grid, even if there are no areas
  RCLCPP_INFO(get_logger(), "Publishing occupancy grid");
  occupancy_grid_publisher_->publish(mapToOccupancyGrid(current_map_));

  RCLCPP_INFO(get_logger(), "Publishing visualization markers");
  map_visualization_publisher_->publish(mapToVisualizationMarkers(current_map_));

  RCLCPP_INFO(get_logger(), "Publishing docking stations as PoseArray");
  docking_station_poses_publisher_->publish(dockingStationsToPoseArray(current_map_));
}

void MapServerNode::configureMap()
{
  map_type_ = declare_parameter("type", "geojson");

  if (map_type_ == "geojson")
  {
    map_file_ = declare_parameter("path", "~/.openmower/map.geojson");
    RCLCPP_INFO(get_logger(), "Using GeoJSON map: %s", map_file_.c_str());
    map_io_ = new GeoJSONMap(map_file_, std::shared_ptr<MapServerNode>(this, [](MapServerNode*) {}));

    return;
  }

  throw std::runtime_error("Unsupported map type: " + map_type_);
}

void MapServerNode::saveAndPublishMap()
{
  RCLCPP_INFO(get_logger(), "Saving map");
  map_io_->save(current_map_);
  publishMap();
  RCLCPP_INFO(get_logger(), "Map saved");
}

void MapServerNode::configureServices()
{
  save_area_service_ = this->create_service<srv::SaveArea>(
      "save_area", std::bind(&MapServerNode::saveAreaHandler, this, std::placeholders::_1, std::placeholders::_2));
  remove_area_service_ = this->create_service<srv::RemoveArea>(
      "remove_area", std::bind(&MapServerNode::removeAreaHandler, this, std::placeholders::_1, std::placeholders::_2));
  save_docking_station_service_ = this->create_service<srv::SaveDockingStation>(
      "save_docking_station",
      std::bind(&MapServerNode::saveDockingStationHandler, this, std::placeholders::_1, std::placeholders::_2));
  remove_docking_station_service_ = this->create_service<srv::RemoveDockingStation>(
      "remove_docking_station",
      std::bind(&MapServerNode::removeDockingStationHandler, this, std::placeholders::_1, std::placeholders::_2));
}

void MapServerNode::saveAreaHandler(srv::SaveArea::Request::SharedPtr request,
                                    srv::SaveArea::Response::SharedPtr response)
{
  RCLCPP_INFO(get_logger(), "Saving area %s", request->area.id.c_str());

  try
  {
    for (auto& area : current_map_.areas)
    {
      if (area.id == request->area.id)
      {
        RCLCPP_INFO(get_logger(), "Updating area %s", request->area.id.c_str());
        area = request->area;

        saveAndPublishMap();

        response->code = srv::SaveArea::Response::CODE_SUCCESS;
        response->message = "Area updated successfully";
        return;
      }
    }

    RCLCPP_INFO(get_logger(), "Adding area %s", request->area.id.c_str());
    current_map_.areas.push_back(request->area);

    saveAndPublishMap();

    response->code = srv::SaveArea::Response::CODE_SUCCESS;
    response->message = "Area added successfully";
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to save area: %s", e.what());
    response->code = srv::SaveArea::Response::CODE_SAVE_FAILED;
    response->message = std::string("Failed to save area: ") + e.what();
  }
}

void MapServerNode::removeAreaHandler(srv::RemoveArea::Request::SharedPtr request,
                                      srv::RemoveArea::Response::SharedPtr response)
{
  RCLCPP_INFO(get_logger(), "Removing area %s", request->id.c_str());

  try
  {
    for (auto it = current_map_.areas.begin(); it != current_map_.areas.end(); ++it)
    {
      if (it->id == request->id)
      {
        current_map_.areas.erase(it);
        saveAndPublishMap();

        response->code = srv::RemoveArea::Response::CODE_SUCCESS;
        response->message = "Area removed successfully";
        return;
      }
    }

    RCLCPP_WARN(get_logger(), "Area %s not found", request->id.c_str());
    response->code = srv::RemoveArea::Response::CODE_NOT_FOUND;
    response->message = "Area not found: " + request->id;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to remove area: %s", e.what());
    response->code = srv::RemoveArea::Response::CODE_UNKNOWN_ERROR;
    response->message = std::string("Failed to remove area: ") + e.what();
  }
}

void MapServerNode::saveDockingStationHandler(srv::SaveDockingStation::Request::SharedPtr request,
                                              srv::SaveDockingStation::Response::SharedPtr response)
{
  RCLCPP_INFO(get_logger(), "Saving docking station %s", request->docking_station.id.c_str());

  try
  {
    for (auto& docking_station : current_map_.docking_stations)
    {
      if (docking_station.id == request->docking_station.id)
      {
        RCLCPP_INFO(get_logger(), "Updating docking station %s", request->docking_station.id.c_str());
        docking_station = request->docking_station;

        saveAndPublishMap();

        response->code = srv::SaveDockingStation::Response::CODE_SUCCESS;
        response->message = "Docking station updated successfully";
        return;
      }
    }

    RCLCPP_INFO(get_logger(), "Adding docking station %s", request->docking_station.id.c_str());
    current_map_.docking_stations.push_back(request->docking_station);

    saveAndPublishMap();

    response->code = srv::SaveDockingStation::Response::CODE_SUCCESS;
    response->message = "Docking station added successfully";
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to save docking station: %s", e.what());
    response->code = srv::SaveDockingStation::Response::CODE_SAVE_FAILED;
    response->message = std::string("Failed to save docking station: ") + e.what();
  }
}

void MapServerNode::removeDockingStationHandler(srv::RemoveDockingStation::Request::SharedPtr request,
                                                srv::RemoveDockingStation::Response::SharedPtr response)
{
  RCLCPP_INFO(get_logger(), "Removing docking station %s", request->id.c_str());

  try
  {
    for (auto it = current_map_.docking_stations.begin(); it != current_map_.docking_stations.end(); ++it)
    {
      if (it->id == request->id)
      {
        current_map_.docking_stations.erase(it);
        saveAndPublishMap();

        response->code = srv::RemoveDockingStation::Response::CODE_SUCCESS;
        response->message = "Docking station removed successfully";
        return;
      }
    }

    RCLCPP_WARN(get_logger(), "Docking station %s not found", request->id.c_str());
    response->code = srv::RemoveDockingStation::Response::CODE_NOT_FOUND;
    response->message = "Docking station not found: " + request->id;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to remove docking station: %s", e.what());
    response->code = srv::RemoveDockingStation::Response::CODE_UNKNOWN_ERROR;
    response->message = std::string("Failed to remove docking station: ") + e.what();
  }
}

void MapServerNode::configureGaussianBlur()
{
  use_gaussian_blur_ = declare_parameter("grid.use_gaussian_blur", false);
  if (use_gaussian_blur_)
  {
    RCLCPP_INFO(get_logger(), "Using Gaussian blur");

    std::vector<double> kernel =
        declare_parameter("map.gaussian_blur.kernel", std::vector<double>{ 1, 2, 1, 2, 4, 2, 1, 2, 1 });
    std::vector<float> kernel_float(kernel.begin(), kernel.end());

    gaussian_filter_ = new SomeGaussianFilter(kernel_float);
  }
}

nav_msgs::msg::OccupancyGrid MapServerNode::mapToOccupancyGrid(msg::Map map)
{
  float minX = std::numeric_limits<float>::max();
  float minY = std::numeric_limits<float>::max();
  float maxX = std::numeric_limits<float>::lowest();
  float maxY = std::numeric_limits<float>::lowest();

  // Add a fallback if no areas
  if (map.areas.empty())
  {
    RCLCPP_WARN(get_logger(), "No areas found in the map. Creating default empty grid.");
    minX = -10.0;
    minY = -10.0;
    maxX = 10.0;
    maxY = 10.0;
  }
  else
  {
    for (const auto& area : map.areas)
    {
      for (const auto& point : area.area.polygon.points)
      {
        minX = std::min(minX, static_cast<float>(point.x));
        minY = std::min(minY, static_cast<float>(point.y));
        maxX = std::max(maxX, static_cast<float>(point.x));
        maxY = std::max(maxY, static_cast<float>(point.y));
      }
    }
    // Add padding
    maxX += 1.0f;
    maxY += 1.0f;
    minX -= 1.0f;
    minY -= 1.0f;
  }

  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.header = map.header;

  // The origin of the map [m, m, rad]. This is the real-world pose of the bottom left corner of cell (0,0) in the map.
  occupancy_grid.info.origin.position.x = minX;
  occupancy_grid.info.origin.position.y = minY;

  // cell size in meters - get from parameter
  occupancy_grid.info.resolution = grid_resolution_;

  // Limit the max size of the grid to avoid memory issues with large areas
  const int MAX_GRID_SIZE = grid_max_size_;
  int raw_width = static_cast<int>((maxX - minX) / occupancy_grid.info.resolution);
  int raw_height = static_cast<int>((maxY - minY) / occupancy_grid.info.resolution);
  int width = std::min(MAX_GRID_SIZE, raw_width);
  int height = std::min(MAX_GRID_SIZE, raw_height);

  if (raw_width > MAX_GRID_SIZE || raw_height > MAX_GRID_SIZE)
  {
    RCLCPP_WARN(get_logger(), "Occupancy grid truncated to max size %d cells (requested %dx%d, truncated by %dx%d)",
                MAX_GRID_SIZE, raw_width, raw_height, std::max(0, raw_width - MAX_GRID_SIZE),
                std::max(0, raw_height - MAX_GRID_SIZE));
  }

  // Ensure minimum grid size
  width = std::max(width, 10);
  height = std::max(height, 10);

  occupancy_grid.info.width = width;
  occupancy_grid.info.height = height;
  occupancy_grid.info.map_load_time = this->now();

  // Default to lethal: anything not explicitly covered by an operation/navigation
  // polygon is out of bounds and must not be drivable.
  occupancy_grid.data = std::vector<int8_t>(occupancy_grid.info.width * occupancy_grid.info.height, 100);

  auto paintArea = [&](const msg::Area& area, int8_t value) {
    try
    {
      fillGridWithPolygon(occupancy_grid, area.area.polygon, value);
    }
    catch (const std::out_of_range& e)
    {
      RCLCPP_ERROR(get_logger(), "Error filling grid with polygon: %s", e.what());
      RCLCPP_ERROR(get_logger(), "Area ID: %s", area.id.c_str());
      RCLCPP_ERROR(get_logger(), "Area type: %d", area.type);
      RCLCPP_ERROR(get_logger(), "Area polygon points: %zu", area.area.polygon.points.size());
      // Continue with other areas rather than returning incomplete grid
    }
  };

  // Pass 1: operation + navigation interiors are free (0). Everything else stays
  // lethal (100), i.e. drivable region defaults to nothing outside these polygons.
  std::vector<msg::Area> exclusion_areas;
  for (const auto& area : map.areas)
  {
    if (area.type == msg::Area::TYPE_EXCLUSION)
    {
      exclusion_areas.push_back(area);
      continue;
    }
    paintArea(area, 0);
  }

  // Pass 2: buffer the keep-in wall outward by one robot radius so the robot's
  // center can reach the true operation/navigation edge.
  dilateFreeRegion(occupancy_grid, robot_radius_);

  // Pass 3: exclusions are stamped last, always lethal (100), overriding any free
  // cell (original or dilated) they overlap.
  for (const auto& area : exclusion_areas)
  {
    paintArea(area, 100);
  }

  RCLCPP_INFO(get_logger(), "Occupancy grid size: %.2fm x %.2fm (%.2fm resolution, %dx%d cells)",
              occupancy_grid.info.width * occupancy_grid.info.resolution,
              occupancy_grid.info.height * occupancy_grid.info.resolution, occupancy_grid.info.resolution,
              occupancy_grid.info.width, occupancy_grid.info.height);

  if (gaussian_filter_)
  {
    RCLCPP_INFO(get_logger(), "Applying Gaussian filter");
    gaussian_filter_->apply(occupancy_grid.data, occupancy_grid.info.width, occupancy_grid.info.height);
  }

  return occupancy_grid;
}

visualization_msgs::msg::MarkerArray MapServerNode::mapToVisualizationMarkers(msg::Map map)
{
  visualization_msgs::msg::MarkerArray markers;

  for (const auto& area : map.areas)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = map.header;
    marker.ns = "areas";
    marker.id = std::hash<std::string>{}(area.id);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;

    if (area.type == msg::Area::TYPE_EXCLUSION)
    {
      marker.ns = "exclusion";

      // red color
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }
    else if (area.type == msg::Area::TYPE_OPERATION)
    {
      marker.ns = "operation";

      // green color
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
    else
    {
      marker.ns = "navigation";

      // white color
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
    }

    for (const auto& point : area.area.polygon.points)
    {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0;
      marker.points.push_back(p);
    }

    if (!area.area.polygon.points.empty())
    {
      auto point = area.area.polygon.points.front();
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0;
      marker.points.push_back(p);
    }

    markers.markers.push_back(marker);
  }

  for (const auto& docking_station : map.docking_stations)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = map.header;
    marker.ns = "docking_station";
    marker.id = std::hash<std::string>{}(docking_station.id);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = docking_station.pose.pose.position.x;
    marker.pose.position.y = docking_station.pose.pose.position.y;
    marker.pose.position.z = 0.1;
    marker.pose.orientation.w = docking_station.pose.pose.orientation.w;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.05;

    // black color
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    markers.markers.push_back(marker);
  }

  return markers;
}

geometry_msgs::msg::PoseArray MapServerNode::dockingStationsToPoseArray(msg::Map map)
{
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header = map.header;

  for (const auto& docking_station : map.docking_stations)
  {
    pose_array.poses.push_back(docking_station.pose.pose);
  }

  return pose_array;
}

void MapServerNode::fillGridWithPolygon(nav_msgs::msg::OccupancyGrid& occupancy_grid,
                                        const geometry_msgs::msg::Polygon& polygon, uint8_t value)
{
  const int width = occupancy_grid.info.width;
  const int height = occupancy_grid.info.height;
  const double resolution = occupancy_grid.info.resolution;
  const double origin_x = occupancy_grid.info.origin.position.x;
  const double origin_y = occupancy_grid.info.origin.position.y;

  auto iterator = PolygonGridIterator(polygon, resolution);
  while (iterator.next())
  {
    auto point = *iterator;
    int grid_x = static_cast<int>((point.x - origin_x) / resolution);
    int grid_y = static_cast<int>((point.y - origin_y) / resolution);

    if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height)
    {
      continue;
    }

    auto index = grid_x + grid_y * width;

    if (index < 0 || index >= static_cast<int>(occupancy_grid.data.size()))
    {
      continue;
    }

    // Last write wins - callers rely on paint order (operation/navigation first,
    // exclusions last) to make exclusions override interiors they overlap.
    occupancy_grid.data[index] = value;
  }
}

void MapServerNode::dilateFreeRegion(nav_msgs::msg::OccupancyGrid& occupancy_grid, double radius)
{
  const int width = occupancy_grid.info.width;
  const int height = occupancy_grid.info.height;
  const double resolution = occupancy_grid.info.resolution;
  const int cell_radius = static_cast<int>(std::round(radius / resolution));

  if (cell_radius <= 0)
  {
    return;
  }

  // Snapshot which cells are free before dilating, so newly-freed cells don't
  // themselves seed further dilation in this same pass.
  std::vector<int> free_cells;
  for (int i = 0; i < static_cast<int>(occupancy_grid.data.size()); ++i)
  {
    if (occupancy_grid.data[i] == 0)
    {
      free_cells.push_back(i);
    }
  }

  std::vector<int8_t> dilated = occupancy_grid.data;
  const double radius_sq = radius * radius;

  for (const int idx : free_cells)
  {
    const int cx = idx % width;
    const int cy = idx / width;

    for (int dy = -cell_radius; dy <= cell_radius; ++dy)
    {
      const int ny = cy + dy;
      if (ny < 0 || ny >= height)
      {
        continue;
      }

      for (int dx = -cell_radius; dx <= cell_radius; ++dx)
      {
        const int nx = cx + dx;
        if (nx < 0 || nx >= width)
        {
          continue;
        }

        // Circular structuring element - matches the circular robot_radius footprint
        // used by the nav2 costmaps, rather than over-inflating corners (square) or
        // under-inflating diagonals (diamond).
        if ((dx * dx + dy * dy) * resolution * resolution > radius_sq)
        {
          continue;
        }

        const int nidx = nx + ny * width;
        if (occupancy_grid.data[nidx] == 100)
        {
          dilated[nidx] = 0;
        }
      }
    }
  }

  occupancy_grid.data = std::move(dilated);
}
}  // namespace open_mower_next::map_server
