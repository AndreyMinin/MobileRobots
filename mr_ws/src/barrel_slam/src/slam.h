#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <Eigen/Core>

const std::size_t ROBOT_VARS = 3;
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
  ros::Timer timer;
  void on_odo(const nav_msgs::Odometry& odom);
  void on_scan(const sensor_msgs::LaserScan& scan);
  void on_timer(const ros::TimerEvent& event);
  void publish_results(const std::string& frame, ros::Time& time);
  void predict(double dt);
  void advertize_landmark_publishers();
  // поиск координад маяков по скану
  std::vector<Eigen::Vector2d> detect_landmarks(const sensor_msgs::LaserScan& scan);
  // поиск индекса маяка в векторе состояния подходящего для измерения, -1 - новый маяк
  int associate_measuriment(const Eigen::Vector2d& landmark_measuriment);
  int add_landmark(const Eigen::Vector2d& landmark_measuriment);
  void correct(int index, const Eigen::Vector2d& landmark_measuriment);
  double v = 0;
  double w = 0;

  // state vector
  Eigen::VectorXd X;
  // system Jacobi
  Eigen::Matrix3d A;
  // covariation of system
  Eigen::MatrixXd P;
  // covariation of measurement errors for each landmark
  Eigen::Matrix2d R;
  // covariation of system vulnerability for x y fi
  Eigen::Matrix3d D;

  ros::Time last_time = ros::Time::now();

  std::size_t last_found_landmark_index = 0;

public:
  Slam();
};
