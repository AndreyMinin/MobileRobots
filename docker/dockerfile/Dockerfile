FROM osrf/ros:melodic-desktop-full-bionic
RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install sudo -y
RUN apt-get install -y \
	ros-melodic-acado 

RUN apt-get install -y mc
RUN apt-get install -y qtcreator qt5-default
RUN rm /bin/sh && ln -s /bin/bash /bin/sh



