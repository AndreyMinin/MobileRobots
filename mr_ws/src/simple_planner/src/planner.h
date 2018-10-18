#ifndef SRC_SIMPLE_PLANNER_SRC_PLANNER_H_
#define SRC_SIMPLE_PLANNER_SRC_PLANNER_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <limits>

namespace simple_planner
{


 // структура, описывающая узел поиска
 struct SearchNode{
   enum State {
      CLOSE, OPEN, UNDEFINED, FINISH
    };
   // состояние узла
   State state = UNDEFINED;
   // значение функции оптимальной стоимости достижения узла
   double g = std::numeric_limits<double>::max();;
   // значение функции эвристики
   double h = 0;
 };



class Planner
{
public:
  Planner(ros::NodeHandle& nh);

private:
  // обновление положения робота
  void on_pose(const nav_msgs::Odometry& odom);
  // колбек целевой точки
  void on_target(const geometry_msgs::PoseStamped& pose);
  // функция обновления карты (map_)
  bool update_static_map();
  // функция расширения карты препятствий (obstacle_map_)
  void increase_obstacles(std::size_t cells);
  // функция вычисления пути в заданную точку
  void calculate_path();

  double heruistic(int i, int j);

  // функции для работы с картами и индексами
  // Проверка индексов в карте
  bool indices_in_map(int i, int j);
  // Возвращает ссылку на значение в карте
  template <class T>
  T& map_value(std::vector<T>& data, int i, int j)
  {
    int index = i * map_.info.width + j;
    ROS_ASSERT(index < data.size() && index >= 0);
    return data[index];
  }

private:
  ros::NodeHandle nh_;
  nav_msgs::OccupancyGrid map_;
  nav_msgs::OccupancyGrid obstacle_map_;
  nav_msgs::OccupancyGrid cost_map_;

  ros::Publisher obstacle_map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("obstacle_map", 1);
  ros::Publisher cost_map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("cost_map", 1);
  ros::Publisher path_publisher_ = nh_.advertise<nav_msgs::Path>("path", 1);

  ros::ServiceClient map_server_client_ =  nh_.serviceClient<nav_msgs::GetMap>("/static_map");

  ros::Subscriber pose_sub_ = nh_.subscribe("ground_truth", 1, &Planner::on_pose, this);
  ros::Subscriber target_sub_ = nh_.subscribe("target_pose", 1, &Planner::on_target, this);

  geometry_msgs::Pose start_pose_;
  geometry_msgs::Pose target_pose_;

  nav_msgs::Path path_msg_;

  double robot_radius_ = nh_.param("robot_radius", 0.5);

  // карта поиска
  std::vector<SearchNode> search_map_;
};

} /* namespace simple_planner */

#endif /* SRC_SIMPLE_PLANNER_SRC_PLANNER_H_ */
