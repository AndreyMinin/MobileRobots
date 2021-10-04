# Макет системы управления мобильным роботом

Является ROS пакетом, предназначенным для разработки системы управления мобильным роботом.

Использует модель робота в симуляторе GAZEBO (www.gazebosim.org)
либо модель робота в симуляторе Stage

## Включает в себя следующие пакеты

**cart_launch** - проект с описанием сцены для Gazebo с картом и объектами сцены
* cart_launch/stage_worlds - описание сцены для stage
* cart_launch/launch/cart_launch - файл запуска модели в среде gazebo
* cart_launch/cart_stage.launch - файл для запуска модели в среде stage

**polaris_ranger_ev** - проект, содержащий модель робота с датчиками для Gazebo
* model/simple_cart.sdf - робот без датчиков,
* model/scan_cart.sdf - робот с плоским дальномером и камерой

**vehicle_ros_plugin** - плагин для Gazebo, позволяющий управлять моделью из ROS.
* robot/velocity - топик для задания скорости движения
* robot/steering - топик для задания кривизны траектории (поворот руля)
* robot/odom   -   топик с текущими данными одометрии от робота

**stage_controller** - модуль управления моделью робота в stage, преобразует команды управления поворотом руля (топик /steering), команды управления акселератором в команды управления моделью в stage (cmd_vel) 

**simple_controller** - простейший ROS контроллер, управляющий движением карта по овальной траектории, с помощью пид регулятора, в качестве ошибки используется расстояние от траектории, вычисляемое на основе абсолютного положения робота. Модуль управляет поворотом руля карта

**odo2tf** - модуль преобразования данных о реальном положении модели робота (Odometry) в сообщения tf для формирования преобразования систем координат world->base_link
Пописывается на топики:
* /odo - топик с данными одометрии ([nav_msgs/Odometry])
Публикует преобразование tf

**laser2d_map** - модуль построения карты препятствий по данным сканирующего двумерного дальномера
Подписывается на топик
* /laser_scan - данные сканирующего дальномера (sensor_msgs/LaserScan)
* /tf - преобразование СК от лазера до world
Публикует
* laser_map - карта препятствий в системе координат world

Установка

Необходимо, чтобы были установлены ROS (Тестировалось на [melodic](http://wiki.ros.org/melodic/Installation)) и Gazebo (для управления моделью в Gazebo) или Stage (для управления моделью в среде stage), а также некоторые пакеты ROS (про которые будет указано внутри заданий)

Возможна разработка из [докер образа](https://github.com/AndreyMinin/MobileRobots#%D0%B8%D1%81%D0%BF%D0%BE%D0%BB%D1%8C%D0%B7%D0%BE%D0%B2%D0%B0%D0%BD%D0%B8%D0%B5-docker), где всё необходимое уже установлено.

1. Нужно склонировать репозиторий:
```bash
git clone https://github.com/AndreyMinin/MobileRobots.git <папка для размещения проекта>
```

2. Перейти в рабочую папку
```bash
cd ros_ws
```

3. собрать проект
```bash
catkin_make
```
или воспользоваться скриптом **build.sh** (из папки src)
```bash
./build.sh
```
Если сборка прошла без ошибок можно переходить к запуску

Для первого запуска может потребоваться выход в интернет- чтобы закачать некторые модели с репозитария Gazebo

4. запуск
  команды выполняются из директории рабочей области (ros_ws)
инициализация рабочей области ROS
```bash
source devel/setup.bash
```

5. запуск модели в среде Gazebo и контроллера 
```bash
roslaunch simple_controller controller.launch
```
модель стартует, затем запускаетя интерфейс gzclient, rviz и rqt
 Либо запуск модели в среде stage(предпочтительный вариант) и контроллера 
```bash
roslaunch simple_controller controller_stage.launch
```
модель стартует, затем запускаетя интерфейс  rqt

В rqt в плагине publish можно отправить сообщение с заданной скоростью в топик /robot/velocity

  Должно появиться окно симулятора (GAZEBO либо  stage), в котором робот ездит в некоторой сцене, отрабатывая заданную траекторию
  г) управление скоростью можно осуществлять с помощью rqt (publish_message) публикуя сообщение со значением скорости в топик /robot/velocity

  Также можно осуществить запуск с помощью скрипта start.sh, находящегося в папке src
```bash
./start.sh
```
	Остановить проект можно с помощью ctrl-C в консоли

## Практические задания курса
1. [Контроллер управления движением по траектории](https://github.com/AndreyMinin/MobileRobots/tree/master/mr_ws/src/simple_controller)
2. [Контроллер управления скоростью](https://github.com/AndreyMinin/MobileRobots/tree/master/mr_ws/src/velocity_controller)
3. [MPC контроллер](https://github.com/AndreyMinin/MobileRobots/tree/master/mr_ws/src/mpc_controller)
4. [Планирование траектории](https://github.com/AndreyMinin/MobileRobots/tree/master/mr_ws/src/simple_planner)
5. [Построение карты](https://github.com/AndreyMinin/MobileRobots/tree/master/mr_ws/src/simple_map)
6. [Локализация по дальномеру](https://github.com/AndreyMinin/MobileRobots/tree/master/mr_ws/src/feature_matcher)
7. [EKF SLAM](https://github.com/AndreyMinin/MobileRobots/tree/master/mr_ws/src/barrel_slam)



