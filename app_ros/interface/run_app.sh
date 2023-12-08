launch_app() {
    echo "launch app"
    # export PYTHONPATH='/opt/ros/melodic/lib/python2.7/dist-packages', '/home/bee/catkin_ws/devel/lib/python2.7/dist-packages'
	#!/bin/bash
	# source /home/bee/catkin_ws/devel/setup.bash
    #rosrun app_ros app_ros.py 

    cd /home/stivietnam/catkin_ws/src/app_ros/script
    python3 app_ros_vs2.py
}

launch_app

echo "
     *********** Please Wait! ************
     "

