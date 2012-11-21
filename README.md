cognidrive_ros
==============

cognidrive_ros tries to connect [MetraLab](http://http://www.metralabs.com/)'s CogniDrive to [ROS](http://www.ros.org).

CogniDrive is proprietary software used to control differential-drive robots. For communication with other software running on the robot, it uses [MIRA](http://www.mira-project.org/). MIRA is also proprietary (but sources are partly available) and very similar to ROS. Currently, MIRA only runs on Windows, Ubuntu 12.04 32bit and CentOS6/RHEL6 32bit.

Connecting MIRA/CogniDrive with ROS mainly comes down to:
 * subscribing MIRA channels and publishing that data using ROS topics.
 * subscribing ROS topics and publishing that data using MIRA channels.
 * forwarding transforms from one framework to the other.
 * offering an actionlib-interface like [move_base](http://www.ros.org/wiki/move_base) to MIRA's [Task-based navigation](http://www.mira-project.org/MIRA-doc/domains/navigation/Pilot/).
 * allowing direct driving "without CogniDrive" by subscribing to ROS:/cmd_vel and forwarding the geometry_msgs/Twist to the robot's wheels.

In our setup, laserscanners, drives, encoders and battery are connected to MIRA, so their data needs to be forwarded into the ROS world when the robot is used in real application. During simulation, however, the virtual devices are created in gazebo (ROS), so their data needs to be forwarded to MIRA to embed cognidrive into the simulation. So, the direction in which cognidrive_ros converts between the frameworks is determined by setting a --simulation flag on startup.

This code was developed and tested on a MetrLabs Scitos G5 robot running Ubuntu 12.04 LTS 32bit and ROS fuerte.

Usage
=====

When cognidrive_ros starts (as a ROS node), it also starts a complete MIRA framework, forwarding all command-line arguments. If you pass

 * "-c | --config   miraconfig.xml", the contained MIRA framework will start in-process, loading all other MIRA units.
 * "-k | --known-fw host:port", the contained MIRA framework will connect to an already-running MIRA-framework on host:port.

Right now, the -c argument is disabled, because running MIRA and ROS in the same process leads to crashes. This is because MIRA uses the system's version of opencv (2.3) and ROS uses its own (2.4), but these versions are not binary compatible.

A typical way to use ROS with CogniDrive on the robot is:

 * go to the directory containing the MIRA configuration file (e.g. PilotDemo.xml), then start "mira -c PilotDemo.xml". In this file, MIRA is instructed to listen on xml:root -> communication -> port (e.g. port 1234).
 * start "cognidrive_ros -k 127.0.0.1:1234", so that the MIRA framework in cognidrive_ros will connect to the instance of MRIA you started above.
 * start "rosrun rviz rviz" to visualize laserscans and transforms, set pose estimates or 2d nav goals.
 * start "miracenter" and connect to the running MIRA framework at 127.0.0.1:1234 to see the same things in MIRA.

You can distribute both ROS nodes and MIRA units over multiple hosts using those framework's native mechanisms. For example, you can run rviz on another machine by setting the environment variable ROS_MASTER_URI to http://robothost:11311.
 
Support
=======

In case of questions, you can reach me at: adler at informatik dot uni-hamburg dot de.