FROM osrf/ros:kinetic-desktop
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
#ADD non_ros_dependencies non_ros_dependencies
RUN apt-get update && apt-get upgrade -y
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list

# Install OpenCV
RUN apt-get update && apt-get -y install libopencv-dev build-essential cmake git libgtk2.0-dev pkg-config python-dev python-numpy libdc1394-22 libdc1394-22-dev libjpeg-dev libpng12-0 libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev libtbb-dev libqt4-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev x264 v4l-utils unzip glew-utils libglew-dev

RUN apt-get update && apt-get install -y --allow-unauthenticated \
        libignition-math3 libsdformat6 \
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
	ros-kinetic-gazebo9-ros-pkgs \
	libpcap-dev \
        libcamera-info-manager-dev \
        libnlopt-dev freeglut3-dev \
        libomp-dev \
        libssh2-1-dev libarmadillo-dev libpcap-dev gksu \
	ros-kinetic-stage-ros ros-kinetic-gmapping ros-kinetic-navigation\
	python-pip
RUN pip install gdbgui --upgrade
RUN apt-get update && apt-get install -y qtcreator qt5-default


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


