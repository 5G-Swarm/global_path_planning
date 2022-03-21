# global_path_planning



catkin_make

source devel/setup.bash

roslaunch map_pub map_pub.launch  //发送地图
roslaunch global_path_planning global_path_planning.launch  //根据确定的起点，和goal进行路径规划
rosrun global_path_planning receiver.py  //进行C++和Python的格式转换