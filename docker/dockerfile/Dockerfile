FROM osrf/ros:noetic-desktop-full-focal
RUN for retry in $(seq 10); do apt-get update -o Acquire::https::Verify-Peer="false" -o Acquire::https::Verify-Host="false" -o Acquire::AllowInsecureRepositories="true" && break; done

RUN apt-get upgrade -y
RUN apt-get install sudo -y


RUN apt-get install -y \
        ros-noetic-navigation \
        ros-noetic-gmapping
        # ros-noetic-acado 
        
RUN apt-get install -y mc
RUN apt-get install -y qtcreator qt5-default
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt-get install -y git
RUN git clone https://github.com/acado/acado.git -b stable ACADOtoolkit
RUN cd ACADOtoolkit && mkdir build && cd build && cmake .. -DACADO_WITH_EXAMPLES=OFF && make && make install

