
#include "planner.h"

#include <cstddef>
#include <queue>
#include <utility>

namespace simple_planner
{

const std::pair<int, int> neighbors[8] = { {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
const int8_t kObstacleValue = 100;

Planner::Planner(ros::NodeHandle& nh) : nh_(nh)
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

  if (!path_msg_.poses.empty()) {
    path_msg_.header.stamp = ros::Time::now();
    path_msg_.header.frame_id = pose.header.frame_id;
    path_publisher_.publish(path_msg_);
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

  std::queue<std::pair<int, int>> wave;
  for (int i = 0; i < map_.info.height; ++i)
  {
    for (int j = 0; j < map_.info.width; ++j)
    {
      if (map_value(map_.data, i, j) != kObstacleValue)
      {
        continue;
      }
      // else - obstacle
      // check neighbors
      for(const auto& shift : neighbors)
      {
        int shifted_i = i + shift.first;
        int shifted_j = j + shift.second;
        if (!indices_in_map(shifted_i, shifted_j))
        {
          continue;
        }
        // if neighbor is not obstacle - add i, j to wave
        if (map_value(map_.data, shifted_i, shifted_j) != kObstacleValue)
        {
          wave.push(std::make_pair(i, j));
          break;
        }
      }
    }
  }
  ROS_INFO_STREAM("Start wave size = " << wave.size());
  for(std::size_t step = 0; step < cells; ++step)
  {
    std::queue<std::pair<int, int>> next_wave;
    while(!wave.empty()) {
      auto indices = wave.front();
      wave.pop();
      for(const auto& shift : neighbors)
      {
        auto neightbor_index = indices;
        neightbor_index.first += shift.first;
        neightbor_index.second += shift.second;
        if (!indices_in_map(neightbor_index.first, neightbor_index.second))
        {
          continue;
        }
        if (map_value(obstacle_map_.data, neightbor_index.first, neightbor_index.second) != kObstacleValue)
        {
          map_value(obstacle_map_.data, neightbor_index.first, neightbor_index.second) = kObstacleValue;
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
  CompareSearchNodes(){}
  bool operator () (const SearchNode* left, const SearchNode* right) const {
    return left->g + left->h > right->g + right->h;
  }
};

void Planner::calculate_path()
{
  // очищаем карту поиска
  search_map_.resize(map_.data.size());
  std::fill(search_map_.begin(), search_map_.end(), SearchNode());
  path_msg_.poses.clear();

  // Здесь необходимо поместить код для поиска пути
  std::priority_queue<SearchNode*, std::vector<SearchNode*>,  CompareSearchNodes> queue;
  int start_i = floor((start_pose_.position.x - map_.info.origin.position.x)/ map_.info.resolution);
  int start_j = floor((start_pose_.position.y - map_.info.origin.position.y)/ map_.info.resolution);
  SearchNode& start = map_value(search_map_, start_i, start_j);
  start.g = 0;
  start.h = heruistic(start_i, start_j);
  start.state = SearchNode::OPEN;
  queue.push(&start);


}

} /* namespace simple_planner */
