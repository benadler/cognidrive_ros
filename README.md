cognidrive_ros
==============

cognidrive_ros tries to connect MetraLab's CogniDrive to ROS. This is a little tricky because CogniDrive uses its own middleware called MIRA (http://www.mira-project.org/). In effect, we translate between ROS-topics and MIRA-channels, ROS-actionlib and MIRA-Navigation-Tasks etc.