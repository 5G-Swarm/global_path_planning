# global_path_planning



catkin_make

source devel/setup.bash

roslaunch global_path_planning global_path_planning.launch  //根据起点和goal进行路径规划
rosrun global_path_planning receiver.py  //进行C++和Python的格式转换

