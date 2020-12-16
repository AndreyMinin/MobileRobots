#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>

const std::size_t ROBOT_STATE_SIZE = 3;
const std::size_t NUMBER_LANDMARKS = 12;
const double HUGE_COVARIANCE = 1e10;

class Slam {
private:
  ros::NodeHandle nh;
  ros::Subscriber odo_sub;
  ros::Subscriber scan_sub;
  // публикатор положения робота
  ros::Publisher pose_pub;
  // публикатор положений маяков
  ros::Publisher landmark_pub[NUMBER_LANDMARKS];
  void on_odo(const nav_msgs::Odometry& odom);
  void on_scan(const sensor_msgs::LaserScan& scan);
  void publish_results(const std::string& frame, const ros::Time& time);
  void predict(double dt);
  void advertize_landmark_publishers();
  // поиск координад маяков по скану
  void detect_landmarks(const sensor_msgs::LaserScan& scan);
  void add_landmark(const sensor_msgs::LaserScan& scan, std::size_t start, std::size_t finish);
  // поиск индекса маяка в векторе состояния подходящего для измерения, -1 - новый маяк
  int associate_measuriment(const Eigen::Vector2d& landmark_measuriment);
  int add_landmark_to_state(const Eigen::Vector2d& landmark_measuriment);
  void correct(int index, const Eigen::Vector2d& landmark_measuriment);
  // публикуем результаты
  void publish_transform(const std_msgs::Header& scan_header);
  double v = 0;
  double w = 0;

  // state vector
  // вектор состояния
  Eigen::VectorXd X;
  // system Jacobi
  // линеаризованная матрица системы
  Eigen::Matrix3d A;
  // covariation of system
  // матрица ковариации ошибок оценок
  Eigen::MatrixXd P;
  // covariation of measurement errors for each landmark
  // матрица ковариации ошибок измерения
  Eigen::Matrix2d Q;
  // covariation of system vulnerability for x y fi
  // матрица ковариации возмущений системы
  Eigen::Matrix3d R;

  ros::Time last_time = ros::Time::now();

  std::size_t landmarks_found_quantity = 0;

  std::vector<Eigen::Vector2d> new_landmarks;

  tf::TransformBroadcaster br;

  const std::string map_frame = nh.param<std::string>("map_frame", "map");

public:
  Slam();
};
