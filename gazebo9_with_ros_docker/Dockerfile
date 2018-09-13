FROM osrf/ros:kinetic-desktop
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
#ADD non_ros_dependencies non_ros_dependencies
RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list

# Install OpenCV
RUN apt-get update && apt-get -y install libopencv-dev build-essential cmake git libgtk2.0-dev pkg-config python-dev python-numpy libdc1394-22 libdc1394-22-dev libjpeg-dev libpng12-0 libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev libtbb-dev libqt4-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev x264 v4l-utils unzip glew-utils libglew-dev

RUN apt-get update && apt-get install -y \
        libignition-math3 
RUN apt-get update && apt-get install -y libsdformat6 \
        libgazebo9 \
        gazebo9 \
        libgazebo9-dev \
	ros-kinetic-pluginlib \
        ros-kinetic-ros-control \
        ros-kinetic-ros-controllers \
	ros-kinetic-rosgraph-msgs \
        ros-kinetic-std-srvs \
        ros-kinetic-realtime-tools \
        ros-kinetic-image-proc \
        ros-kinetic-nmea-msgs \
        ros-kinetic-jsk-visualization \
        ros-kinetic-roslint \
        ros-kinetic-nmea-navsat-driver \
        ros-kinetic-opencv3 \
	ros-kinetic-cv-bridge \
        ros-kinetic-grid-map \
        ros-kinetic-camera-info-manager-py \
	ros-kinetic-image-transport \
	ros-kinetic-camera-calibration-parsers \
	ros-kinetic-polled-camera \
	ros-kinetic-camera-info-manager \
	ros-kinetic-pcl-ros \
	libpcap-dev \
        libcamera-info-manager-dev \
        libnlopt-dev freeglut3-dev \
        libomp-dev \
        libssh2-1-dev libarmadillo-dev libpcap-dev gksu

RUN apt-get update && apt-get install -y qtcreator qt5-default

WORKDIR /catkin_ws/src
RUN git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b kinetic-devel
WORKDIR /catkin_ws

RUN source /opt/ros/kinetic/setup.bash \
	&& catkin_init_workspace src \
	&& catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic -DCMAKE_BUILD_TYPE=Release install

WORKDIR /ros_opt/src
RUN git clone https://github.com/ros-drivers/velodyne.git
WORKDIR /ros_opt
RUN source /opt/ros/kinetic/setup.bash \
	&& catkin_init_workspace src \
	&& catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic -DCMAKE_BUILD_TYPE=Release install

WORKDIR /opt/
RUN git clone https://github.com/acado/acado.git
WORKDIR /opt/acado/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. && make  -j4 && make install 
WORKDIR /
RUN apt-get install -y ros-kinetic-stage-ros
RUN apt-get install -y python-pip
RUN pip install gdbgui --upgrade
RUN apt-get update && apt-get install -y ros-kinetic-gmapping
RUN apt-get update && apt-get install -y ros-kinetic-navigation


