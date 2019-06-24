FROM ros:kinetic-perception-xenial

#####################################################
#####################################################

ENV DEBIAN_FRONTEND noninteractive

# See:
# http://answers.ros.org/question/325039/apt-update-fails-cannot-install-pkgs-key-not-working/
RUN apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && apt-get -y upgrade
RUN apt-get update && \
    apt-get install -y jq \
                       libgoogle-glog-dev \
                       locales \
                       curl \
                       sudo \
                       apt-transport-https \
                       ros-kinetic-rviz

RUN echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && \
    locale-gen en_US.UTF-8 && \
    /usr/sbin/update-locale LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN echo 'deb [arch=amd64] https://nexus.ifm.com/repository/ifm-robotics_ubuntu_xenial_amd64_ros xenial main' > /etc/apt/sources.list.d/ifm-robotics.list
RUN apt-key adv \
         --keyserver hkp://ha.pool.sks-keyservers.net:80 \
         --recv-key 8AB59D3A2BD7B692

RUN apt-get update && \
    apt-get install -y ifm3d-camera \
                       ifm3d-framegrabber \
                       ifm3d-image \
                       ifm3d-opencv \
                       ifm3d-pcicclient \
                       ifm3d-tools \
                       ifm3d-python \
                       ifm3d-python3

RUN id ifm 2>/dev/null || useradd --uid 30000 --create-home -s /bin/bash -U ifm
RUN echo "ifm ALL=(ALL) NOPASSWD: ALL" | tee -a /etc/sudoers

USER ifm
WORKDIR /home/ifm
RUN /bin/bash -c 'mkdir src && \
    cd src && \
    source /opt/ros/kinetic/setup.bash && \
    git clone https://github.com/ifm/ifm3d-ros.git && \
    cd ~ && mkdir -p catkin/ifm3d/src && cd catkin/ifm3d/src && \
    catkin_init_workspace && \
    ln -s ~/src/ifm3d-ros ifm3d && \
    cd ~/catkin/ifm3d && \
    catkin_make && \
    catkin_make -DCMAKE_INSTALL_PREFIX=/home/ifm/ros/ifm3d install && \
    cd ~ && rm -rf ~/catkin ~/src'

RUN /bin/bash -c 'echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc && \
    echo "source /home/ifm/ros/ifm3d/setup.bash --extend" >> ~/.bashrc'
