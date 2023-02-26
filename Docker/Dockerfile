FROM osrf/ros:noetic-desktop

RUN apt-get update
RUN apt-get install -y git && apt-get install -y python3-pip

RUN mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src

RUN git clone https://github.com/noshluk2/ros1_wiki

RUN echo "ALL DONE!"