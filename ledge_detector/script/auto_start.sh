#!/bin/sh
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/$USER/ws_perching/devel/setup.bash

accountname=$USER
PCpassword=$USER


xterm -e "echo $PCpassword | sudo -S chmod a+rw /dev/ttyACM0" & sleep 1
xterm -e "echo $PCpassword | sudo -S chmod a+rw /dev/ttyACM1" & sleep 1
xterm -e "echo $PCpassword | sudo -S chmod a+rw /dev/ttyUSB0" & sleep 1
xterm -e "echo $PCpassword | sudo -S chmod a+rw /dev/ttyUSB1" & sleep 1


xterm -geometry 80x36+0+0 -e "source /opt/ros/$ROS_DISTRO/setup.bash && roscore" & sleep 3
xterm -geometry 80x36+0+0 -e "source /opt/ros/$ROS_DISTRO/setup.bash && rosparam set /use_sim_time true" & sleep 3

xterm -geometry 80x36+300+0 -hold -e "source /opt/ros/$ROS_DISTRO/setup.bash && source /home/$USER/ws_perching/devel/setup.bash && roslaunch ledge_detector ledge_detect_with_rviz.launch" & sleep 3
#detecting the ledge and publish goal trajectory

xterm -geometry 80x36+300+0 -hold -e "source /opt/ros/$ROS_DISTRO/setup.bash && rosbag play -s 80 -u 30 -l --clock /home/nuc/Desktop/rosbag_2022-03-29-09-42-07.bag" & sleep 3
