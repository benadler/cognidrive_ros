#include <cognidriveros.h>
#include <tclap/CmdLine.h>

CogniDriveRos::CogniDriveRos(int argc, char **argv)
{
    TCLAP::CmdLine cmd("cognidrive_ros tries to connect MetraLab's CogniDrive to ROS.");
    TCLAP::SwitchArg simulationSwitch("s","simulation","Enable simulation mode.", false);
    cmd.add(simulationSwitch);
    cmd.parse(argc, argv);
    const bool simulation = simulationSwitch.getValue();

    // ros init
    ros::init(argc, argv, "cognidrive_ros");

    mRosNodeHandle = new ros::NodeHandle;

    mRosTransformBroadcaster = new tf::TransformBroadcaster;

    mNumberOfPacketsMira2RosLaserScan = 0;
    mNumberOfPacketsMira2RosOdometry = 0;
    mNumberOfPacketsMira2RosBattery = 0;

    // set up all the topics we want to send into ROS - laserscans first, topic name matches PR2
    mRosPubLaserScan = mRosNodeHandle->advertise<sensor_msgs::LaserScan>("/base_scan", 50);

    // this is the odometry as we know it. We could also publish /base_odometry/odometry
    // (total distance/angle travelled) and /base_odometry/odomstate (errors and expected slip)
    mRosPubOdometry = mRosNodeHandle->advertise<nav_msgs::Odometry>("/base_odometry/odom", 50);

    // the ROS BatteryServer2 messages are a bit overkill for MIRA's BatteryState, as the PR2
    // manages around 48 batteries, while MIRA abstracts Scitos' 2 batteries into a single one.
    mRosPubBatteryState = mRosNodeHandle->advertise<pr2_msgs::BatteryServer2>("/battery/server2", 50);

    // whenever we receive laserscans from ROS (gazebo), we want to send it to MIRA/CogniDrive
    mRosSubLaserScan = mRosNodeHandle->subscribe("/base_scan", 1000, &CogniDriveRos::onRosLaserScan, this);
    // whenever we receive odometry from ROS (gazebo), we want to send it to MIRA/CogniDrive
    mRosSubOdometry = mRosNodeHandle->subscribe("/base_odometry/odom", 1000, &CogniDriveRos::onRosOdometry, this);

    // Create and start the mira framework
    // add -c scitos....xml to start cognidrive here - we should specify it in the ROS launchfile
    mMiraFramework = new mira::Framework(argc, argv, true);

    // Create mira authority. This is like ros::NodeHandle
    mMiraAuthority = new mira::Authority("/", "cognidrive_ros");

    // Create dummy drive, so cognidrive will set the velocity on this object, which will forward it to ROS
    mDummyDrive = new DummyDrive(mRosNodeHandle->advertise<geometry_msgs::Twist>("cmd_vel", 1));
    // and publish its methods as service
    mMiraAuthority->publishService(*mDummyDrive);

    // The names of the channels need to match those defined in the robot configuration file (e.g. PilotDemo.xml).

    // create a MIRA channel to publish rangescans coming from ROS/gazebo (in simulation)
    mMiraChannelRangeScan = mMiraAuthority->publish<mira::robot::RangeScan>("/robot/FrontLaser/Laser");
    // create a MIRA channel to publish odometry coming from ROS/gazebo (in simulation)
    mMiraChannelOdometry = mMiraAuthority->publish<mira::robot::Odometry2>("/robot/Odometry");


    if(simulation)
    {
      ROS_INFO("cognidrive_ros started in simulation mode - ignoring MIRA rangescans, odometry and battery.");
    }
    else
    {
      ROS_INFO("cognidrive_ros started in application mode - listening to MIRA rangescans, odometry and battery.");

      // subscribe to MIRA laserscans and forward them to ROS (in real application)
      mMiraAuthority->subscribe<mira::robot::RangeScan>("/robot/Laser", &CogniDriveRos::onMiraLaserScan, this);

      // subscribe to MIRA odometry and forward it to ROS (in real application)
      mMiraAuthority->subscribe<mira::robot::Odometry2>("/robot/Odometry", &CogniDriveRos::onMiraOdometry, this);

      // subscribe to MIRA batterystate and forward it to ROS (in real application)
      mMiraAuthority->subscribe<mira::robot::BatteryState>("/robot/charger/Battery", &CogniDriveRos::onMiraBatteryState, this);
    }

    // transform things
    //mMiraAuthority->addTransformLink("child", "parent");
    //mMiraAuthority->publishTransform("child", mira::Pose2(0.0, 0.0, 0.0), mira::Time::now());
    //mira::Pose2 t = mMiraAuthority->getTransform<mira::Pose2>("child", "parent", mira::Time::now());

    // provide ros-actionlib interface to CogniDrive
    mMoveBaseAction = new MoveBaseAction(ros::this_node::getName(), mMiraAuthority);
}

CogniDriveRos::~CogniDriveRos()
{
}

void CogniDriveRos::onMiraLaserScan(mira::ChannelRead<mira::robot::RangeScan> data)
{
    // do something with value->range, ...
    sensor_msgs::LaserScan scanRos = Converter::mira2ros(*data);

    scanRos.header.stamp = ros::Time::fromBoost(data.getTimestamp());
    scanRos.header.frame_id = data.getChannelID();

    // There is data->sequenceID, but this isn't currently used in MIRA
    scanRos.header.seq = mNumberOfPacketsMira2RosLaserScan++;

    // now publish the laserscan to ROS
    mRosPubLaserScan.publish(scanRos);
}

void CogniDriveRos::onMiraOdometry(mira::ChannelRead<mira::robot::Odometry2> data)
{
    nav_msgs::Odometry odomRos = Converter::mira2ros(*data);

    odomRos.header.stamp = ros::Time::fromBoost(data.getTimestamp());
    odomRos.header.frame_id = data.getChannelID();

    // There is data->sequenceID, but this isn't currently used in MIRA
    odomRos.header.seq = mNumberOfPacketsMira2RosOdometry++;

    // now publish the laserscan to ROS
    mRosPubOdometry.publish(odomRos);

    // This is a slight hack: when driving, cognidrive publishes the robot's pose into mira's
    // tf tree. But there is no way to be notified of this event. Since cognidrive always
    // publishes odometry and transform-tree at the same time (exact order is unknown), we
    // simply abuse this odometry-callback to read the transform and publish that into ROS::TF
    mira::Pose2 p = mMiraAuthority->getTransform<mira::Pose2>("/robot/RobotFrame", "/GlobalFrame", data.getTimestamp());

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(data->pose.x(), data->pose.y(), 0.0) );
    transform.setRotation( tf::Quaternion(data->pose.phi(), 0, 0) );
    mRosTransformBroadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::fromBoost((data.getTimestamp())), "world", "robot??! XXX TODO"));
}

void CogniDriveRos::onMiraBatteryState(mira::ChannelRead<mira::robot::BatteryState> data)
{
    // This message has no header, thus no seq, time and frame_id
    pr2_msgs::BatteryServer2 batteryRos = Converter::mira2ros(*data);

    batteryRos.header.stamp = ros::Time::fromBoost(data.getTimestamp());
    batteryRos.header.frame_id = data.getChannelID();

    // There is data->sequenceID, but this isn't currently used in MIRA
    batteryRos.header.seq = mNumberOfPacketsMira2RosBattery++;

    // now publish the laserscan to ROS
    mRosPubBatteryState.publish(batteryRos);
}

// Called for every laserscan from ros (probably coming from gazebo).
// Forward to mira/cognidrive so it can drive in the simulator.
void CogniDriveRos::onRosLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    mMiraChannelRangeScan.post(Converter::ros2mira(*msg), mira::Time(msg->header.stamp.toBoost()));
}

void CogniDriveRos::onRosOdometry(const nav_msgs::Odometry::ConstPtr& msg)
{
    mMiraChannelOdometry.post(Converter::ros2mira(*msg), mira::Time(msg->header.stamp.toBoost()));
}

int CogniDriveRos::exec()
{
    // do the locomotion
    ros::spin();
    return 0;
}