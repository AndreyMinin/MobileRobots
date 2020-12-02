#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <vector>
//#include <tf/transform_publisher.h>

class Matcher
{
public:
  explicit Matcher(ros::NodeHandle& nh) : nh(nh) {}

private:
  void on_laser_scan(const sensor_msgs::LaserScan& scan);
  // ищем особенные точки в скане
  void detect_features(const sensor_msgs::LaserScan& scan);
  // предсказываем положение новых особенных точек относительно опорных 
  // по предыдущему инкрементальному трансформуб считая скорость меняется незначительно
  void predict_features_poses();
  // ищем пары ближайших особенных точек
  void find_feature_pairs();
  // функция определения трансформов по парам особенных точек
  void find_transform();
  // публикуем результаты
  void publish_transform(const std_msgs::Header& header);
  // публикуем особенные точки из текущего скана
  void publish_features(const std_msgs::Header& header);
  // обновляем вектор опорных особенных точек
  void update_base_features();
  // добавляет новую feature на которую папали лучи скана от start до finish включительно
  void add_feature(const sensor_msgs::LaserScan& scan, std::size_t start, std::size_t finish);

private:
  ros::NodeHandle nh;
  ros::Subscriber laser_sub = nh.subscribe("/base_scan", 1, &Matcher::on_laser_scan, this);
  ros::Publisher feature_pub = nh.advertise<visualization_msgs::Marker>("features", 1, true);
  ros::Publisher odo_pub = nh.advertise<nav_msgs::Odometry>("odo", 1 , false);
  double feature_rad = nh.param<double>("feature_radius", 0.55);
  const std::string map_frame = nh.param<std::string>("map_frame", "map");
  // features from base scan набор опорных особенных точек в СК карты
  std::vector<Eigen::Vector2d> base_features;
  // features from new scan набор особенных точек из нового скана в СК лазерного дальномера
  std::vector<Eigen::Vector2d> new_features;
  // features from new scan transfromed to base scan according to interpolation
  // набор особенных точек нового скана переведенных в СК карты
  // с учетом предсказания - считаем что робот продолжит движение как на предыдущем шаге
  std::vector<Eigen::Vector2d> predicted_features;
  // indexes of corresponding new features for every base feature
  // вектор индексов особенных точек нового скана для каждой опорной особенной точки
  // либо -1 - если нет таковой
  std::vector<int> feature_pair_indices;
  // transform from current scan to map
  // преобразование переводящее текущий скан в карту, обновляется в результате каждого шага
  Eigen::Isometry2d transform = Eigen::Isometry2d::Identity();
  // transform from current scan to previous
  // инкрементальное преобразование за последний шаг в СК карты
  Eigen::Isometry2d incremental_transform = Eigen::Isometry2d::Identity();

  tf::TransformBroadcaster br;
  // отметка времени предыдущего скана
  ros::Time last_stamp = ros::Time::now();
};

