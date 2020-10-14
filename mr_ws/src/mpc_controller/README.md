# Управление движением ровера по траектории с помощью MPC(model predictive control)
В этой задаче мы будем управлять движением ровера по траектории с помощью метоов MPC, реализованных в специализированной библиотеке [ACADO](https://acado.github.io/)

1. Для того, чтобы использовать эту библиотеку ее необходимо установить: в случае ROS под Ubuntu это очень просто:
Установка ACADO:
```bash
sudo apt install ros-<версия ROS>-acado
```
Должны будут скачаться и установиться необходимые для работы пакеты

2. Необходимо обновить (или создать заново)репозиторий: 

Если ранее была удалена папка проекта mpc_controller, для ее восстановления нужно выполнить
```bash
git checkout master mr_ws/src/mpc_controller
```
Обновить репозиторий
```bash
git pull
```

3. Запуск модели и контроллера 

Переходим в папку ROS workspace и инициализируем проект
```bash
cd mr_ws
source devel/setup.bash
```
Запускаем модель в среде Stage (предпочтительный вариант)
```bash
roslaunch mpc_controller controller_stage.launch
```
Или в среде Gazebo
```bash
roslaunch mpc_controller controller.launch
```
Запустится модель робота, а также контроллер, rqt с графиками ошибок и текущих скоростей и rviz, в котором отображается заданная (зеленым цветом), рассчетная по mpc (красными точками) траектория и текущее положение робота.
Робот движется по заданной траектории.

В этой задаче контроллер управляет как скоростью ровера, так и поворотом руля.
Контроллер реализован в виде класса [MPCController](https://github.com/AndreyMinin/MobileRobots/blob/141e6174d1fb35a738321fb8b91e3798a38cc135/mr_ws/src/mpc_controller/src/mpccontroller.h#L37).
Траектория задается в виде [списка сегментов](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/mpc_controller/src/mpccontroller.h#L73) как и в задаче simple_controller.
Некоторые функции класса MPCController являются коллбэками на сообщения ROS:
- [on_pose](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/mpc_controller/src/mpccontroller.cpp#L168) - обновляет положение робота (подписана на топик с ground truth симулятора)
- [on_odo](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/mpc_controller/src/mpccontroller.cpp#L184) - получение одометрии робота, обновление текущей скорости.

Основная логика контроллера выполняется в функции- колбеке [таймера](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/mpc_controller/src/mpccontroller.cpp#L140). Состоит из следующих шагов:
- apply_control - отправляет желаемую скорость и желаемый угол поворота руля, рассчитанные на предыдущем шаге;
- update_robot_pose - на основе последних данных о положении робота и его скорости расчитывает положение робота к следующему срабатыванию таймера;
- update_trajectory_segment - обновляет указатель на ближайший сегмент траектории вдоль, которого движется робот;
- update_control_points - обновляет массив контрольных точек(длина control_points_num), лежащих на траектории, начиная с ближайшей к роботу. Расстояние между точками control_points_dl;
- convert_control_points - переводит координаты точек в систему координат, связанную с центром робота;
- calculate_control_coefs - рассчитывает коэфициенты полинома третьей степени, который лучшим образом описывает контрольные точки в координатах робота;
 Далее запускается MPC на вход которому подается начальноя скорость и текущий угол поворота руля, коэффициенты полинома. На выходе имеем ускорение и скорость поворота руля котрые необходимо применить, чтобы остаться на траектории с соблюдением огранияений заданных в MPC, а также оптимальная траектория, рассчитанная в MPC
 После расчета для отладки публикуются заданная траектрия - publish_trajectory, полином - publish_poly и траектория MPC - publish_mpc_trajectory. Траектории публикуются в виде облаков точек, которые отображаются в rviz. Также публикуется ошибка - расстояние до ближайшей точки траектории - publish_error

 ## Устройство MPC
 Сам механизм расчета mpc траектории реализован в класс [MPC](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/mpc_controller/include/mpc.h)
 Он состоит из [конструктора](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/mpc_controller/src/mpc.cpp#L16), где задаются параметры MPC и модель системы. И функцци [solve](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/mpc_controller/src/mpc.cpp#L41), которая ищет MPC решение.
 Для решения  MPC спользуется библиотека ACADO. 
 В классе определяются переменные описывающие процесс управления в типах, заданных в библиотеке. 
 Это переменные состояния 

 [ACADO::DifferentialState x, y, fi, delta, vel;](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/mpc_controller/include/mpc.h#L17) - координаты, угол ориентации, угол поворота руля и скорость робота
Управление 

[ACADO::Control delta_rate, acc;](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/mpc_controller/include/mpc.h#L18) - скорость вращения руля и ускорение
Дифференциальное уравнение 

[  ACADO::DifferentialEquation f;](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/mpc_controller/include/mpc.h#L19) описывающее процесс. Оно формируется в конструкторе в удобном для пользователя [виде](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/mpc_controller/src/mpc.cpp#L32). По существу это уравнения кинематики нашего робота. операторы здесь не операторы языка, а операции билиотеки ACADO, которые формируют специальный объект библиотеки ACADO.

В функциии solve задаются ограничения под которыми будет осуществлен поиск решения в виде специального объекта ACADO [ACADO::OCP](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/mpc_controller/src/mpc.cpp#L46)
Задается начальное состояние - нулевое, так как каждый раз мы решаем задачу MPC в коодинатах, привязанных к начальному положению робота. Задаются из параметров класса огранияения на максимальной ускорение максимальный угол поворота руля и скорость поворота руля.

Далее задаются цели оптимизации:
траектории в виде специальных объектов ACADO [Expression](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/mpc_controller/src/mpc.cpp#L65)
cte - crosstrackerror - ошибка по положению (это разница между координатой y и значением полинома в x)
epsi - ошибка по углу(это разница между арктангенсом производной полинома и текущим углом)
steer_cost - квадрат скорости вращения руля, который тоже хотим минимизировать, чтобы получить плавное уравнение движения.

В [строчке](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/mpc_controller/src/mpc.cpp#L72) задаем линейную комбинацию целей оптимизации с коэффициентами

Далее запускается расчет, на выходе которрого получаем оптимальную траекторию в виде вектора состояний и контролов, из кторой извлекаем координаты траектории для отображения и управление на следующий шаг из первого контрола.

### Задача
Подобрать коэффициенты (параметры) MPC для устойчивого движения робота со скоростью 10 м/сек. Параметры задются в [yaml файле](https://github.com/AndreyMinin/MobileRobots/blob/master/mr_ws/src/mpc_controller/launch/controller.yaml). 

