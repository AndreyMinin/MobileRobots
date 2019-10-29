FROM osrf/ros:melodic-desktop
RUN apt-get update && apt-get upgrade -y
RUN apt-get install sudo -y
RUN apt-get install -y \
        libgazebo9 \
        gazebo9 \
        libgazebo9-dev \
        libignition-transport-dev \
        ros-melodic-gazebo-dev \
        ros-melodic-gazebo-ros \
	ros-melodic-gazebo-plugins \
	ros-melodic-acado \
	ros-melodic-stage-ros \
	ros-melodic-map-server

RUN apt-get update && apt-get install -y qtcreator qt5-default
RUN rm /bin/sh && ln -s /bin/bash /bin/sh



