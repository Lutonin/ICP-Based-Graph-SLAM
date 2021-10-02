Steps to run the simulation:

1) copy m2wr_description and my_work files into your workspace in ros
2) in the command terminal type the following:
	- cd your_workspace
	- catkin build
	- roslaunch m2wr_description spawn.launch to  launch the gazebo
	- rosrun teleop_twist_keyboard teleop_twist_keyboard.py to move the robot
	- go to the src file in my_work
	- python RecordData.py to record a dataset
3) for running the slam algorithm on clf files (intel.clf, ace.clf)
	- python SLAM.py --input file.clf (pick one of the above or download other clf files)
4) for running recorded datasets in step 2
	- python SLAM_Custom.py

Libraries needed:
- numpy
- matplotlib
- scipy
- g2o
- sklearn