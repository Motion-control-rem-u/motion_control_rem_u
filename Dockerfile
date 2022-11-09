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
RUN sudo apt-get install ros-melodic-catkin python3-catkin-tools


RUN pip3 install \
    numpy==1.19.4 \
    opencv-python \
    rospkg 

# copy .repos file in for fast cloning of remote repository if necessary
COPY motion.repos /

# build ROS workspace
WORKDIR /motion_control
COPY src ./src

#RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /motion_control; catkin build'
#cleaner way of building the workspace
RUN catkin config --extend /opt/ros/melodic && catkin build --no-status

# Automatically source the workspace when starting a bash session
RUN echo "source /motion_control/devel/setup.bash" >> /etc/bash.bashrc
