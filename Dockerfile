# build on top of the erc provided base image
FROM  ros:melodic
# install some dependencies. Vim for quick viewing of scripts inside the cli.
RUN apt update && apt -y upgrade && apt install -y \
    #python3-vcstool \
    ros-melodic-rospy-tutorials \ 
    ros-melodic-tf 
    #vim
     
# install python requirements 
RUN sudo apt-get -y install python3-pip 
RUN python3 -m pip install --upgrade pip 
RUN sudo apt-get -y install ros-melodic-catkin python3-catkin-tools
RUN sudo apt-get install -y dos2unix

RUN pip3 install \
    numpy==1.19.4 \
    opencv-python \
    rospkg \
    pygame 

# copy .repos file in for fast cloning of remote repository if necessary
COPY motion.repos /

# build ROS workspace
WORKDIR /motion_control
COPY src ./src
# Convert CRLF line endings in python scripts to LF endings
RUN find /motion_control -type f -print0 | xargs -0 dos2unix
RUN apt-get --purge remove -y dos2unix && rm -rf /var/lib/apt/lists/*

#RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /motion_control; catkin build'
#cleaner way of building the workspace
RUN catkin config --extend /opt/ros/melodic && catkin build --no-status

# Automatically source the workspace when starting a bash session
RUN echo "source /motion_control/devel/setup.bash" >> /etc/bash.bashrc
