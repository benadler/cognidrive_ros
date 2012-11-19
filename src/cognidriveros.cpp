#include <cognidriveros.h>
#include <tclap/CmdLine.h>

CogniDriveRos::CogniDriveRos(int argc, char **argv)
{
    TCLAP::CmdLine cmd("cognidrive_ros connects MetraLab's CogniDrive to ROS.");
    TCLAP::SwitchArg switchSimulation("s","simulation","Enable simulation mode.", false);
    // we don't really process the -c parameter using TCLAP, but TCLAP would fail if we specify -c without it knowing the parameter
    TCLAP::ValueArg<std::string> switchMiraConfiguration("c","config","A MIRA configuration file (*.xml).",false,"","string");
    TCLAP::ValueArg<int> switchMiraPort("p","fw-port","A MIRA network port.",false,1234,"int");
    cmd.add(switchSimulation);
    cmd.add(switchMiraConfiguration);
    cmd.add(switchMiraPort);
    cmd.parse(argc, argv);
    mSimulation = switchSimulation.getValue();

    // ros init
    ros::init(argc, argv, "cognidrive_ros");

    mRosNodeHandle = new ros::NodeHandle;

    mRosTransformBroadcaster = new tf::TransformBroadcaster;

    mNumberOfPacketsMira2RosLaserScan = 0;
    mNumberOfPacketsMira2RosOdometry = 0;
    mNumberOfPacketsMira2RosBattery = 0;
    mNumberOfPacketsMira2RosMapMetaData = 0;
    mNumberOfPacketsMira2RosMapOccupancyGrid = 0;

    // Create and start the mira framework
    // add -c scitos....xml to start cognidrive here - we should specify it in the ROS launchfile
    mMiraFramework = new mira::Framework(argc, argv, true);

    // Create mira authority. This is like ros::NodeHandle
    mMiraAuthority = new mira::Authority("/", "cognidrive_ros");
    
    // Whenever we receive an initial pose estimate, we want to send it to MIRA/CogniDrive
    mRosSubInitialPose = mRosNodeHandle->subscribe("initialpose", 1, &CogniDriveRos::onRosInitialPose, this);
    
    if(mSimulation)
    {
      ROS_INFO("cognidrive_ros started in simulation mode:");
      ROS_INFO(" -> ignoring MIRA rangescans, odometry and battery to prevent loops");
      ROS_INFO(" -> forwarding ROS laserscans and odometry to cognidrive.");
      ROS_INFO(" -> forwarding cognidrive's drive-command to ROS/cmd_vel topic.");
      
      // In simulation, whenever we receive laserscans from ROS (gazebo), we want to send it to MIRA/CogniDrive
      mRosSubLaserScan = mRosNodeHandle->subscribe("/base_scan", 10, &CogniDriveRos::onRosLaserScan, this);

      // In simulation, whenever we receive odometry from ROS (gazebo), we want to send it to MIRA/CogniDrive
      mRosSubOdometry = mRosNodeHandle->subscribe("/base_odometry/odom", 100, &CogniDriveRos::onRosOdometry, this);

      // The names of the channels need to match those defined in the robot configuration file (e.g. PilotDemo.xml).
      // create a MIRA channel to publish rangescans coming from ROS/gazebo (in simulation)
      mMiraChannelRangeScan = mMiraAuthority->publish<mira::robot::RangeScan>("/robot/FrontLaser/Laser");
      // create a MIRA channel to publish odometry coming from ROS/gazebo (in simulation)
      mMiraChannelOdometry = mMiraAuthority->publish<mira::robot::Odometry2>("/robot/Odometry");

      // Create dummy drive, so cognidrive will set the velocity on this object, which will forward it to ROS.
      // That way, gazebo can move the robot according to CogniDrive's commands
      mDummyDrive = new DummyDrive(mRosNodeHandle->advertise<geometry_msgs::Twist>("cmd_vel", 1));
      // and publish its methods as service
      mMiraAuthority->publishService(*mDummyDrive);
    }
    else
    {
      ROS_INFO("cognidrive_ros started in application mode:");
      ROS_INFO(" -> forwarding MIRA rangescans, odometry and battery to ROS.");
      ROS_INFO(" -> forwarding ROS/cmd_vel to cognidrive for transparent driving.");
      
      // set up all the topics we want to send into ROS - laserscans first, topic name matches PR2
      mRosPubLaserScanFront = mRosNodeHandle->advertise<sensor_msgs::LaserScan>("/base_scan", 10);
      mRosPubLaserScanRear = mRosNodeHandle->advertise<sensor_msgs::LaserScan>("/base_scan_rear", 10);

      // this is the odometry as we know it. We could also publish /base_odometry/odometry
      // (total distance/angle travelled) and /base_odometry/odomstate (errors and expected slip)
      mRosPubOdometry = mRosNodeHandle->advertise<nav_msgs::Odometry>("/base_odometry/odom", 50);

      // the ROS BatteryServer2 messages are a bit overkill for MIRA's BatteryState, as the PR2
      // manages around 48 batteries, while MIRA abstracts Scitos' 2 batteries into a single one.
      mRosPubBatteryState = mRosNodeHandle->advertise<pr2_msgs::BatteryServer2>("/battery/server2", 1);

      // In application, whenever we receive CmdVel/Twists from ROS, we want to send it to the robots real motors.
      // In simulation, we don't listen, because the differential-drive node in ROS does the wheel-moving in gazebo
      mRosSubCmdVel = mRosNodeHandle->subscribe("/cmd_vel", 100, &CogniDriveRos::onRosCmdVel, this);

      // subscribe to MIRA laserscans and forward them to ROS (in real application)
      mMiraAuthority->subscribe<mira::robot::RangeScan>("/robot/FrontLaser/Laser", &CogniDriveRos::onMiraLaserScanFront, this);
      mMiraAuthority->subscribe<mira::robot::RangeScan>("/robot/RearLaser/Laser", &CogniDriveRos::onMiraLaserScanRear, this);

      // subscribe to MIRA odometry and forward it to ROS (in real application)
      mMiraAuthority->subscribe<mira::robot::Odometry2>("/robot/Odometry", &CogniDriveRos::onMiraOdometry, this);

      // subscribe to MIRA batterystate and forward it to ROS (in real application)
      mMiraAuthority->subscribe<mira::robot::BatteryState>("/robot/charger/Battery", &CogniDriveRos::onMiraBatteryState, this);
    }
    
    // publish MIRA/cognidrive maps to ROS in both simulation and application
    ROS_INFO(" -> forwarding cognidrive's map to ROS");
    mRosPubMapMetaData = mRosNodeHandle->advertise<nav_msgs::MapMetaData>("/map_metadata", 1, /*latch:*/ true);
    mRosPubMapOccupancyGrid = mRosNodeHandle->advertise<nav_msgs::OccupancyGrid>("/map", 1, /*latch:*/ true);
    mira::Channel<mira::maps::OccupancyGrid> miraMapChannel = mMiraAuthority->subscribe<mira::maps::OccupancyGrid>("/maps/static/Map", &CogniDriveRos::onMiraMap, this);

    // Just in case MIRA published the map before we had our callbck set up, manually try to read the map from the channel.
    try {
      onMiraMap(miraMapChannel.read());
    } catch(mira::Exception& e)
    {
       ROS_INFO("error reading map from channel: %s", e.what());
    }
    
    // transform things
    //mMiraAuthority->addTransformLink("child", "parent");
    //mMiraAuthority->publishTransform("child", mira::Pose2(0.0, 0.0, 0.0), mira::Time::now());
    //mira::Pose2 t = mMiraAuthority->getTransform<mira::Pose2>("child", "parent", mira::Time::now());

    // provide ros-actionlib interface to CogniDrive
    mMoveBaseAction = new MoveBaseAction(mRosNodeHandle, mMiraAuthority);
}

CogniDriveRos::~CogniDriveRos()
{
}

void CogniDriveRos::onMiraLaserScanFront(mira::ChannelRead<mira::robot::RangeScan> data)
{
    //ROS_INFO("CogniDriveRos::onMiraLaserScan(): forwarding a laserscan (front) from MIRA to ROS");
    // do something with value->range, ...
    sensor_msgs::LaserScan scanRos = Converter::mira2ros(*data);

    scanRos.header.stamp = ros::Time::fromBoost(data.getTimestamp());
    scanRos.header.frame_id = data->frameID;//data.getChannelID();
    
    mira::Pose2 p = mMiraAuthority->getTransform<mira::Pose2>(data->frameID, "/GlobalFrame", data.getTimestamp());
//     ROS_INFO("laserscanner to global transform: %2.2f", mira::rad2deg(p.phi())/*(mira::MakeString() << p).c_str()*/);
    mRosTransformBroadcaster->sendTransform(tf::StampedTransform(miraPoseToRosTransform(&p), ros::Time::fromBoost((data.getTimestamp())), "map", data->frameID));

    // There is data->sequenceID, but this isn't currently used in MIRA
    scanRos.header.seq = mNumberOfPacketsMira2RosLaserScan++;

    // now publish the laserscan to ROS
    mRosPubLaserScanFront.publish(scanRos);
}

void CogniDriveRos::onMiraLaserScanRear(mira::ChannelRead<mira::robot::RangeScan> data)
{
    //ROS_INFO("CogniDriveRos::onMiraLaserRear(): forwarding a laserscan (rear) from MIRA to ROS");
    // do something with value->range, ...
    sensor_msgs::LaserScan scanRos = Converter::mira2ros(*data);

    scanRos.header.stamp = ros::Time::fromBoost(data.getTimestamp());
    scanRos.header.frame_id = data.getChannelID();

    // There is data->sequenceID, but this isn't currently used in MIRA
    scanRos.header.seq = mNumberOfPacketsMira2RosLaserScan++;

    // now publish the laserscan to ROS
    mRosPubLaserScanRear.publish(scanRos);
}

void CogniDriveRos::onMiraOdometry(mira::ChannelRead<mira::robot::Odometry2> data)
{
    // The converted ROS-odometry might be useless or even dangerous, see the callee for info.
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

    //ROS_INFO("CogniDriveRos::onMiraOdometry(): forwarding odometry %d from MIRA to ROS and publishing cognidrive's position (%2.2f/%2.2f/%2.2f deg) into ROS::tf", mNumberOfPacketsMira2RosOdometry, p.x(), p.y(), mira::rad2deg(p.phi()));

    mRosTransformBroadcaster->sendTransform(tf::StampedTransform(miraPoseToRosTransform(&p), ros::Time::fromBoost((data.getTimestamp())), "map", "base_link"));
}

void CogniDriveRos::onMiraBatteryState(mira::ChannelRead<mira::robot::BatteryState> data)
{
    
    // This message has no header, thus no seq, time and frame_id
    pr2_msgs::BatteryServer2 batteryRos = Converter::mira2ros(*data);

    batteryRos.header.stamp = ros::Time::fromBoost(data.getTimestamp());
    batteryRos.header.frame_id = data.getChannelID();

    // There is data->sequenceID, but this isn't currently used in MIRA
    batteryRos.header.seq = mNumberOfPacketsMira2RosBattery++;

    //ROS_INFO("CogniDriveRos::onMiraBatteryState(): forwarding batterystate %d from MIRA to ROS: %d percent charge", mNumberOfPacketsMira2RosBattery, batteryRos.average_charge);
    
    // now publish the laserscan to ROS
    mRosPubBatteryState.publish(batteryRos);
}

void CogniDriveRos::onMiraMap(mira::ChannelRead<mira::maps::OccupancyGrid> data)
{
    ROS_INFO("CogniDriveRos::onMiraMap(): forwarding a map from MIRA to ROS");

    // Fetch map transform from MIRA
    const mira::Pose2 poseMap = mMiraAuthority->getTransform<mira::Pose2>("/maps/static/MapFrame", "/GlobalFrame", mira::Time::now());

    nav_msgs::OccupancyGrid mapOccupancyGridRos = Converter::mira2ros(*data, poseMap);
    mapOccupancyGridRos.header.stamp = ros::Time::fromBoost(data.getTimestamp());
    mapOccupancyGridRos.header.frame_id = std::string("/map");//data.getChannelID();
    mapOccupancyGridRos.header.seq = mNumberOfPacketsMira2RosMapMetaData++;
    mRosPubMapOccupancyGrid.publish(mapOccupancyGridRos);
    
    nav_msgs::MapMetaData mapMetaDataRos = mapOccupancyGridRos.info;
//     mapMetaDataRos.header.stamp = ros::Time::fromBoost(data.getTimestamp());
//     mapMetaDataRos.header.frame_id = data.getChannelID();
//     mapMetaDataRos.header.seq = mNumberOfPacketsMira2RosMapMetaData++;
    mRosPubMapMetaData.publish(mapMetaDataRos);
    
    ROS_INFO("size of map1 is %d * %d.", mapOccupancyGridRos.info.width, mapOccupancyGridRos.info.height);
    ROS_INFO("size of map2 is %d * %d.", mapMetaDataRos.width, mapMetaDataRos.height);
}

// Called for every laserscan from ros (probably coming from gazebo).
// Forward to mira/cognidrive so it can drive in the simulator. Make sure to mask invalid (self-scanning) rays, otherwise cognidrive is afraid of running itself over
void CogniDriveRos::onRosLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("CogniDriveRos::onRosLaserScan(): forwarding a laserscan from ROS to MIRA (probably from gazebo), masking first and last 55 rays to prevent self-collision");
    mMiraChannelRangeScan.post(Converter::ros2mira(*msg, 55), mira::Time(msg->header.stamp.toBoost()));
}

void CogniDriveRos::onRosOdometry(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_INFO("CogniDriveRos::onRosOdometry(): forwarding odometry data from ROS to MIRA");
    mMiraChannelOdometry.post(Converter::ros2mira(*msg), mira::Time(msg->header.stamp.toBoost()));
}

void CogniDriveRos::onRosCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
{
    // We received a ROS cmd_vel (e.g. from playstation controller). Lets make the wheels on the real robot move!
    ROS_INFO("CogniDriveRos::onRosCmdVel(): forwarding ROS cmd_vel message to MIRA::setVelocity()");
    mMiraAuthority->callService<void>("/robot/Robot", "setVelocity", mira::Velocity2(msg->linear.x, 0.0f, msg->angular.z));
}

void CogniDriveRos::onRosInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ROS_INFO("CogniDriveRos::onRosInitialPose(): forwarding ROS initialpose (%2.2f/%2.2f/%2.2f deg) to MIRA", msg->pose.pose.position.x, msg->pose.pose.position.y, mira::rad2deg(tf::getYaw(msg->pose.pose.orientation)));
    
    mira::PoseCov2 pose = Converter::ros2mira(msg);
    auto s = mMiraAuthority->queryServicesForInterface("ILocalization");
    if(!s.empty())
    {
      auto result = mMiraAuthority->callService<void>(s.front(), "setInitPose", pose);
      result.timedWait(mira::Duration::seconds(10));
      result.get(); // causes exception if something went wrong.
    }
}

int CogniDriveRos::exec()
{
    // do the locomotion
    ROS_INFO("CogniDriveRos::exec(): starting event loop...");
    ros::spin();
    ROS_INFO("CogniDriveRos::exec(): quit ROS event loop...");
    
    ROS_INFO("CogniDriveRos::exec(): requesting MIRA to terminate...");
    mMiraFramework->requestTermination();
    ROS_INFO("CogniDriveRos::exec(): MIRA terminated successfully.");
 
    delete mDummyDrive;
    delete mMoveBaseAction;
    
    delete mMiraAuthority;
    delete mMiraFramework;
    
    delete mRosNodeHandle;
    delete mRosTransformBroadcaster;
    
    return 0;
}