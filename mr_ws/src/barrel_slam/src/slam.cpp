#include "slam.h"
#include <angles/angles.h>
#include <sstream>
#include <math.h>

void Slam::on_odo(const nav_msgs::Odometry& odom)
{
  v = odom.twist.twist.linear.x;
  w = odom.twist.twist.angular.z;
}

void Slam::add_landmark(const sensor_msgs::LaserScan& scan, std::size_t start, std::size_t finish) {
  // добавляем только особенные  точки на которые попало более 2 лучей
  if (finish - start < 2) {
    return;
  }
  ROS_INFO_STREAM("Add landmark between " << start << " " << finish);
  // TODO Здесь должен быть код определения координаты центра круглого препятствия 
  
  // добавляем в вектор особенных точек
  // new_features.push_back(Eigen::Vector2d(x, y)));
}

void Slam::detect_landmarks(const sensor_msgs::LaserScan& scan)
{
  new_landmarks.clear();
  // TODO Здесь должен быть код для пределения особенные точек скана
  // ищем начальный и конечный индексы лучей, падающих на одно препятствие
  // и вызываем add_landmark

}

int Slam::associate_measuriment(const Eigen::Vector2d& landmark_measuriment)
{
  double nearest_distance = 1e10;
  int nearest_index = -1;
  for (std::size_t i = 0; i < NUMBER_LANDMARKS; ++i) {
    double distance = (landmark_measuriment - X.segment(i*2, 2)).norm();
    if (distance < nearest_distance) {
      nearest_index = i;
    }
  }
  // naive association
  const double kAssocThreshold = 5.0;
  if (nearest_index < kAssocThreshold)
  {
    return nearest_index;
  }
  return -1;
}

int Slam::add_landmark_to_state(const Eigen::Vector2d& landmark_measuriment)
{
  ++last_found_landmark_index;
  // TODO init landmark in state
  // Здесь должен быть код по инициализации части вектора состояния, соответствующей 
  // маяку с индексом last_found_landmark_index
  return last_found_landmark_index;
}

void Slam::correct(int index, const Eigen::Vector2d& landmark_measuriment)
{
  // TODO 
  // Здесь должен быть код для обновления состояния по измерению iого маяка
}

void Slam::on_scan(const sensor_msgs::LaserScan& scan)
{
  detect_landmarks(scan);
  predict((scan.header.stamp - last_time).toSec());
  last_time = scan.header.stamp;
  for (std::size_t i = 0; i < new_landmarks.size(); ++i) {
    const auto landmark_index = associate_measuriment(new_landmarks[i]);
    if (landmark_index >= 0) {
      correct(landmark_index, new_landmarks[i]);
    } else {
      add_landmark_to_state(new_landmarks[i]);
    }
  }

  publish_results("map", scan.header.stamp);
  publish_transform(scan.header);
}

void fill_pose_msg(geometry_msgs::PoseWithCovariance& pose,
                   double x, double y, double fi,
                   const Eigen::Matrix2d& cov_matr)
{
  pose.covariance.assign(0);
  pose.covariance[0] = cov_matr(0,0); pose.covariance[1] = cov_matr(0,1);
  pose.covariance[6] = cov_matr(1,0); pose.covariance[7] = cov_matr(1,1);
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.w = cos(fi/2);
  pose.pose.orientation.z = sin(fi/2);
}

void Slam::publish_results(const std::string& frame, const ros::Time& time)
{
  geometry_msgs::PoseWithCovarianceStamped pose;

  pose.header.frame_id = frame;
  pose.header.stamp = time;
  // публикуем сообщение с позицией робота
  fill_pose_msg(pose.pose, X(0), X(1), X(2), P.topLeftCorner(2, 2));
  pose_pub.publish(pose);

  // публикуем сообщения с положениями маяков
  for (int i = 0; i < last_found_landmark_index; ++i) 
  {
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = frame;
    pose.header.stamp = time;
    fill_pose_msg(pose.pose, X(ROBOT_STATE_SIZE + i * 2), X(ROBOT_STATE_SIZE + i * 2 + 1), 0,
                  P.block<2, 2>(ROBOT_STATE_SIZE + i * 2, ROBOT_STATE_SIZE + i * 2));
    landmark_pub[i].publish(pose);
  }
}

void Slam::publish_transform(const std_msgs::Header& scan_header)
{  
  // публикуем трансформ от скана до карты, 
  // не наоборот, так как дерево tf - однонаправленное
  tf::Transform tf_transform;
  double angle = X(2);
  Eigen::Matrix2d R;
  R (0, 0) = R (1, 1) = cos (angle);
  R (0, 1) = -sin (angle);
  R (1, 0) = sin (angle);
  Eigen::Vector2d t = X.head(2);
  Eigen::Isometry2d transform = Eigen::Translation2d(t) * Eigen::Isometry2d(R);

  Eigen::Isometry2d inverted_transform = transform.inverse();
  tf_transform.setOrigin( tf::Vector3(inverted_transform.translation().x(), 
                      inverted_transform.translation().y(), 
                      0.0) );
  tf::Quaternion inv_q;
  const auto& inv_matrix = inverted_transform.matrix();
  double inv_yaw = atan2(inv_matrix(1, 0), inv_matrix(0, 0));
  inv_q.setRPY(0, 0, inv_yaw);
  tf_transform.setRotation(inv_q);
  br.sendTransform(tf::StampedTransform(tf_transform, 
                                        scan_header.stamp, 
                                        scan_header.frame_id, 
                                        map_frame));
}

void Slam::predict(double dt)
{
  // X(t+1) = g(t)
  X(0) += v * cos(X(2)) * dt;
  X(1) += v * sin(X(2)) * dt;
  X(2) += w * dt;
  X(2) = angles::normalize_angle(X(2));

  // вычисляем якобиан
  A = Eigen::Matrix3d::Identity();
  A(0,0) = 1.0; A(0,1) = 0; A(0,2) = -v * sin(X(2)) * dt;
  A(1,0) = 0.0; A(1,1) = 1.0; A(1,2) = v * cos(X(2)) * dt;
  A(2,0) = 0.0; A(2,1) = 0.0; A(2,2) = 1.0;

  // P = A*P*AT + R для блока соответствующего роботу
  P.topLeftCorner(ROBOT_STATE_SIZE, ROBOT_STATE_SIZE) =
      A * P.topLeftCorner(ROBOT_STATE_SIZE, ROBOT_STATE_SIZE) * A.transpose() + R;
  // для остальных блоков
  P.topRightCorner(ROBOT_STATE_SIZE, NUMBER_LANDMARKS * 2) = 
    A * P.topRightCorner(ROBOT_STATE_SIZE, NUMBER_LANDMARKS * 2);
  P.bottomLeftCorner(NUMBER_LANDMARKS * 2, ROBOT_STATE_SIZE) = 
    P.topRightCorner(ROBOT_STATE_SIZE, NUMBER_LANDMARKS * 2).transpose();
}

void Slam::advertize_landmark_publishers()
{
  std::string landmark("landmark");
  for (int i = 0; i < NUMBER_LANDMARKS; ++i)
  {
    std::stringstream stream;
    stream << landmark << i;
    landmark_pub[i] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(stream.str(), 1);
  }
}


Slam::Slam():
    nh("~"),
    odo_sub(nh.subscribe("/odom", 1, &Slam::on_odo, this)),
    scan_sub(nh.subscribe("/scan", 1, &Slam::on_scan, this)),
    pose_pub(nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("slam_pose", 1)),
    X(3 + 2*NUMBER_LANDMARKS),
    A(Eigen::Matrix3d::Identity()),
    P(Eigen::MatrixXd::Zero(X.size(), X.size()))
{
  // начальный вектор состояния заполняем нулями
  X = Eigen::VectorXd::Zero(X.size());
  // начальное значение ковариации для маяков
  P.bottomRightCorner(NUMBER_LANDMARKS * 2, NUMBER_LANDMARKS * 2) =
      HUGE_COVARIANCE * Eigen::MatrixXd::Identity(NUMBER_LANDMARKS * 2, NUMBER_LANDMARKS * 2);

  advertize_landmark_publishers();

  Q = Eigen::Matrix2d::Zero();
  Q(0, 0) = nh.param<double>("range_sigma_sqr", 0.01);
  Q(1, 1) = nh.param<double>("angle_sigma_sqr", 0.001);

  R = Eigen::Matrix3d::Zero();
  R(0, 0) = nh.param<double>("x_sigma_sqr", 0.01);
  R(1, 1) = nh.param<double>("y_sigma_sqr", 0.01);
  R(2, 2) = nh.param<double>("angle_sigma_sqr", 0.001);
}
