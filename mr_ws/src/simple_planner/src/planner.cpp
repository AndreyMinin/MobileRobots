
#include "planner.h"

#include <cstddef>
#include <queue>
#include <set>
#include <utility>

namespace simple_planner
{

const MapIndex neighbors[8] = { {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
const int8_t kObstacleValue = 100;


Planner::Planner(ros::NodeHandle& nh) :
 nh_(nh)
{
  while(!map_server_client_.waitForExistence(ros::Duration(1))) {
    ROS_INFO_STREAM("Wait map server");
  }
  ROS_INFO_STREAM("Service connected");
}

void Planner::on_pose(const nav_msgs::Odometry& odom)
{
  start_pose_ = odom.pose.pose;
}

void Planner::on_target(const geometry_msgs::PoseStamped& pose)
{
  ROS_INFO_STREAM("Get goal " << pose.pose.position.x << " " << pose.pose.position.y);
  ROS_INFO_STREAM("Start is " << start_pose_.position.x << " " << start_pose_.position.y);
  target_pose_ = pose.pose;

  if (!update_static_map() )
  {
    ROS_ERROR_STREAM("Can not receive map");
    return ;
  }

  increase_obstacles(ceil(robot_radius_/map_.info.resolution));
  obstacle_map_publisher_.publish(obstacle_map_);

  calculate_path();

  if (!path_msg_.points.empty()) {
    path_msg_.header.stamp = ros::Time::now();
    path_msg_.header.frame_id = pose.header.frame_id;
    path_publisher_.publish(path_msg_);
  } else {
  	ROS_WARN_STREAM("Path not found!");
  }
}

bool Planner::update_static_map()
{
  nav_msgs::GetMap service;
  if (!map_server_client_.call(service))
  {
    ROS_ERROR_STREAM("Failed to receive a map");
    return false;
  }
  map_ = service.response.map;
  ROS_INFO_STREAM("Map received : " << map_.info.width << " " << map_.info.height);
  return true;
}

bool Planner::indices_in_map(int i, int j)
{
  return i >= 0 && j >= 0 && i < map_.info.width && j < map_.info.height;
}

void Planner::increase_obstacles(std::size_t cells)
{
  obstacle_map_.info = map_.info;
  obstacle_map_.header = map_.header;
  obstacle_map_.data.resize(map_.data.size());
  obstacle_map_.data = map_.data;

  std::queue<MapIndex> wave;
  for (int i = 0; i < map_.info.width; ++i)
  {
    for (int j = 0; j < map_.info.height; ++j)
    {
      if (map_value(map_.data, i, j) != kObstacleValue)
      {
        continue;
      }
      // else - obstacle
      // check neighbors
      for(const auto& shift : neighbors)
      {
        int neighbor_i = i + shift.i;
        int neighbor_j = j + shift.j;
        if (!indices_in_map(neighbor_i, neighbor_j))
        {
          continue;
        }
        // if neighbor is not obstacle - add i, j to wave
        if (map_value(map_.data, neighbor_i, neighbor_j) != kObstacleValue)
        {
          wave.push({i, j});
          break;
        }
      }
    }
  }
  ROS_INFO_STREAM("Start wave size = " << wave.size());
  for(std::size_t step = 0; step < cells; ++step)
  {
    std::queue<MapIndex> next_wave;
    while(!wave.empty()) {
      auto indices = wave.front();
      wave.pop();
      for(const auto& shift : neighbors)
      {
        auto neightbor_index = indices;
        neightbor_index.i += shift.i;
        neightbor_index.j += shift.j;
        if (!indices_in_map(neightbor_index.i, neightbor_index.j))
        {
          continue;
        }
        if (map_value(obstacle_map_.data, neightbor_index.i, neightbor_index.j) != kObstacleValue)
        {
          map_value(obstacle_map_.data, neightbor_index.i, neightbor_index.j) = kObstacleValue;
          next_wave.push(neightbor_index);
        }
      }
    } // wave empty
    std::swap(wave, next_wave);
    ROS_INFO_STREAM("Wave size = " << wave.size());
  }
}

double Planner::heruistic(int i, int j) {
  return 0;
}

class CompareSearchNodes {
public:
  explicit CompareSearchNodes(Planner& planner): planner_(planner) {}
  bool operator () (const MapIndex& left_index, const MapIndex& right_index) const {
  	SearchNode& left = planner_.map_value(planner_.search_map_, left_index.i, left_index.j);
  	SearchNode& right = planner_.map_value(planner_.search_map_, right_index.i, right_index.j);
    if (left.g + left.h == right.g + right.h) {
    	if (left_index.i == right_index.i) {
    		return left_index.j < right_index.j;
    	}
    	return left_index.i < right_index.i;
    }
    return left.g + left.h < right.g + right.h;
  }
private:
  Planner& planner_;
};

void Planner::calculate_path()
{
  // очищаем карту поиска
  search_map_.resize(map_.data.size());
  std::fill(search_map_.begin(), search_map_.end(), SearchNode());
  path_msg_.points.clear();

  // Здесь необходимо продолжить код для поиска пути
  std::set<MapIndex, CompareSearchNodes> queue(CompareSearchNodes(*this));
  MapIndex start_index = point_index(start_pose_.position.x, start_pose_.position.y);
  SearchNode& start = map_value(search_map_, start_index.i, start_index.j);
  start.g = 0;
  start.h = heruistic(start_index.i, start_index.j);
  start.state = SearchNode::OPEN;

  auto& start_obstacle_value = map_value(obstacle_map_.data, start_index.i, start_index.j);
  if (start_obstacle_value == kObstacleValue) {
  	ROS_WARN_STREAM("Start is in obstacle!");
  	return;
  }
  queue.insert(start_index);

  MapIndex target_index = point_index(target_pose_.position.x, target_pose_.position.y);
  bool found = false;
  while (!queue.empty()) {
  	auto node_index_iter = queue.begin();
  	auto node_index = *node_index_iter;
  	auto& node = map_value(search_map_, node_index.i, node_index.j);
  	node.state = SearchNode::CLOSE;
  	queue.erase(node_index_iter);

  	// insert code here

  }

  // fill path message with points from path 
  if (found) {
  	int i = target_index.i;
  	int j = target_index.j;
  	geometry_msgs::Point32 p;
  	while (i != start_index.i || j != start_index.j) {
  		p.x = i * map_.info.resolution + map_.info.origin.position.x;
  		p.y = j * map_.info.resolution + map_.info.origin.position.y;
  		auto& node = map_value(search_map_, i, j);
  		path_msg_.points.push_back(p);
  		ROS_INFO_STREAM("i = "<< i <<" j = " << j << " g = " << node.g);
  		double min_g = node.g;
  		
  	}
  }


}

} /* namespace simple_planner */
