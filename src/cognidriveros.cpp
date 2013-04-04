#include <cognidriveros.h>
#include <tclap/CmdLine.h>

CogniDriveRos::CogniDriveRos(int argc, char **argv)
{
    TCLAP::CmdLine cmd("cognidrive_ros connects MetraLab's CogniDrive to ROS.");

    // Disable -c for now, so that people have to start MIRA in one process and cognidrive_ros in another.
    // Currently, the only reason for having both in separate processes is that both link to different
    // versions of opencv, leading to crashes when setting a driving task in MoveBaseAction.
    //TCLAP::ValueArg<std::string> switchMiraConfiguration("c","config","A MIRA configuration file (*.xml).",false,"","string");
    //cmd.add(switchMiraConfiguration);

    TCLAP::ValueArg<int> switchMiraPort("p","fw-port","A MIRA network port.",false,1234,"int");
    cmd.add(switchMiraPort);

    TCLAP::ValueArg<int> switchMiraDebug("d","debug-level","The log level from 0=CRITICAL to 5=TRACE",false,2,"int");
    cmd.add(switchMiraDebug);

    TCLAP::ValueArg<std::string> switchMiraRemoteFramework("k","known-fw","Connect to a remote MIRA framework.",false,"127.0.0.1:1234","string");
    cmd.add(switchMiraRemoteFramework);

    TCLAP::SwitchArg switchSimulation("s","simulation","Enable simulation mode.", false);
    cmd.add(switchSimulation);

    cmd.parse(argc, argv);
    mSimulation = switchSimulation.getValue();

    // Create and start the mira framework before starting ROS to allow for ordered shutdown.
    // (both re-set signal handlers.).
    mMiraFramework = new mira::Framework(argc, argv, true);

    // ros init
    ros::init(argc, argv, "cognidrive_ros");

    mRosNodeHandle = new ros::NodeHandle;

    mRosTransformBroadcaster = new tf::TransformBroadcaster;

    mNumberOfPacketsMira2RosLaserScan = 0;
    mNumberOfPacketsMira2RosOdometry = 0;
    mNumberOfPacketsMira2RosBattery = 0;
    mNumberOfPacketsMira2RosMapMetaData = 0;
    mNumberOfPacketsMira2RosMapOccupancyGrid = 0;


    // Create mira authority. This is like ros::NodeHandle
    mMiraAuthority = new mira::Authority("/", "cognidrive_ros");

    // Whenever we receive an initial pose estimate, we want to send it to MIRA/CogniDrive
    mRosSubInitialPose = mRosNodeHandle->subscribe("initialpose", 1, &CogniDriveRos::onRosInitialPose, this);
    
    mRosServiceLoadMap = mRosNodeHandle->advertiseService("LoadMap", &CogniDriveRos::onRosLoadMap, this);

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

    // provide ros-actionlib interface to CogniDrive
    mMoveBaseAction = new MoveBaseAction(mMiraAuthority);
}

CogniDriveRos::~CogniDriveRos()
{
    printf("CogniDriveRos::~CogniDriveRos(): requesting MIRA to terminate...\n");
    mMiraFramework->requestTermination();
    printf("CogniDriveRos::~CogniDriveRos(): MIRA terminated successfully.\n");

    printf("CogniDriveRos::~CogniDriveRos(): deleting members.\n");

    if(mSimulation) delete mDummyDrive;

    delete mMoveBaseAction;

    delete mMiraAuthority;
    delete mMiraFramework;

    delete mRosNodeHandle;
    delete mRosTransformBroadcaster;

    printf("CogniDriveRos::~CogniDriveRos(): finished.\n");
    fflush(stdout);
}

void CogniDriveRos::onMiraLaserScanFront(mira::ChannelRead<mira::robot::RangeScan> data)
{
    //ROS_INFO("CogniDriveRos::onMiraLaserScan(): forwarding a laserscan (front) from MIRA to ROS");
    // do something with value->range, ...
    sensor_msgs::LaserScan scanRos;
    Converter::mira2ros(*data, &scanRos);

    // There is data->sequenceID, but this isn't currently used in MIRA
    scanRos.header.seq = mNumberOfPacketsMira2RosLaserScan++;
    scanRos.header.stamp = ros::Time::fromBoost(data.getTimestamp());
    scanRos.header.frame_id = data->frameID;//data.getChannelID();

    // publish the laserscanners pose to ROS::tf
    const mira::Pose3 p = mMiraAuthority->getTransform<mira::Pose3>(data->frameID, "/GlobalFrame", data.getTimestamp());
    mRosTransformBroadcaster->sendTransform(tf::StampedTransform(miraPoseToRosTransform(&p), ros::Time::fromBoost((data.getTimestamp())), "map", data->frameID));

    // now publish the laserscan to ROS
    mRosPubLaserScanFront.publish(scanRos);
}

void CogniDriveRos::onMiraLaserScanRear(mira::ChannelRead<mira::robot::RangeScan> data)
{
    //ROS_INFO("CogniDriveRos::onMiraLaserRear(): forwarding a laserscan (rear) from MIRA to ROS");
    // do something with value->range, ...
    sensor_msgs::LaserScan scanRos;
    Converter::mira2ros(*data, &scanRos);

    // There is data->sequenceID, but this isn't currently used in MIRA
    scanRos.header.seq = mNumberOfPacketsMira2RosLaserScan++;
    scanRos.header.stamp = ros::Time::fromBoost(data.getTimestamp());
    scanRos.header.frame_id = data->frameID;//data.getChannelID();

    // publish the laserscanners pose to ROS::tf
    const mira::Pose3 p = mMiraAuthority->getTransform<mira::Pose3>(data->frameID, "/GlobalFrame", data.getTimestamp());
    mRosTransformBroadcaster->sendTransform(tf::StampedTransform(miraPoseToRosTransform(&p), ros::Time::fromBoost((data.getTimestamp())), "map", data->frameID));

    // now publish the laserscan to ROS
    mRosPubLaserScanRear.publish(scanRos);
}

void CogniDriveRos::onMiraOdometry(mira::ChannelRead<mira::robot::Odometry2> data)
{
    // The converted ROS-odometry might be useless or even dangerous, see the callee for info.
    nav_msgs::Odometry odomRos;
    Converter::mira2ros(*data, &odomRos);

    // There is data->sequenceID, but this isn't currently used in MIRA
    odomRos.header.seq = mNumberOfPacketsMira2RosOdometry++;
    odomRos.header.stamp = ros::Time::fromBoost(data.getTimestamp());
    odomRos.header.frame_id = data->frameID;//.getChannelID();

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
    pr2_msgs::BatteryServer2 batteryRos;
    Converter::mira2ros(*data, &batteryRos);

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
    ROS_INFO("CogniDriveRos::onMiraMap(): waiting for map transform...");
    const bool transformFound = mMiraAuthority->waitForTransform("/maps/static/MapFrame", "/GlobalFrame", mira::Duration::seconds(10));
    if(transformFound)
    {
      ROS_INFO("CogniDriveRos::onMiraMap(): map transform received, forwarding map.");
    }
    else
    {
      ROS_ERROR("CogniDriveRos::onMiraMap(): transform not found");
      return;
    }
    const mira::Pose2 poseMap = mMiraAuthority->getTransform<mira::Pose2>("/maps/static/MapFrame", "/GlobalFrame", mira::Time::now());

    nav_msgs::OccupancyGrid mapOccupancyGridRos;
    Converter::mira2ros(*data, &mapOccupancyGridRos, poseMap);
    mapOccupancyGridRos.header.stamp = ros::Time::fromBoost(data.getTimestamp());
    mapOccupancyGridRos.header.frame_id = std::string("/map");//data.getChannelID();
    mapOccupancyGridRos.header.seq = mNumberOfPacketsMira2RosMapMetaData++;
    mRosPubMapOccupancyGrid.publish(mapOccupancyGridRos);

    nav_msgs::MapMetaData mapMetaDataRos = mapOccupancyGridRos.info;
    mRosPubMapMetaData.publish(mapMetaDataRos);
}

// Called for every laserscan from ros (probably coming from gazebo).
// Forward to mira/cognidrive so it can drive in the simulator. Make sure to mask invalid (self-scanning) rays, otherwise cognidrive is afraid of running itself over
void CogniDriveRos::onRosLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("CogniDriveRos::onRosLaserScan(): forwarding a laserscan from ROS to MIRA (probably from gazebo), masking first and last 55 rays to prevent self-collision");
    mira::robot::RangeScan scanMira;
    Converter::ros2mira(*msg, &scanMira, 55);
    mMiraChannelRangeScan.post(scanMira, mira::Time(msg->header.stamp.toBoost()));
}

void CogniDriveRos::onRosOdometry(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_INFO("CogniDriveRos::onRosOdometry(): forwarding odometry data from ROS to MIRA");
    mira::robot::Odometry2 odomMira;
    Converter::ros2mira(*msg, &odomMira);
    mMiraChannelOdometry.post(odomMira, mira::Time(msg->header.stamp.toBoost()));
}

void CogniDriveRos::onRosCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
{
    // We received a ROS cmd_vel (e.g. from playstation controller). Lets make the wheels on the real robot move!
    ROS_INFO("CogniDriveRos::onRosCmdVel(): forwarding ROS cmd_vel message to MIRA::setVelocity()");

    // We can either use the "/robot/Robot"-service directly and make sure the cmd_vel goes directly to the scitos'
    // wheels, or we can query for service "IDrive". Then, we might find the service exposed by ourselves (DummyDrive)
    // and cause infinite loops: We receive a cmd_vel from ROS, forward it to MIRA-IDrive which calls our own
    // DummyDrive, which publishes cmd_vel again. To avoid this, we use "/robot/Robot".
    mMiraAuthority->callService<void>("/robot/Robot", "setVelocity", mira::Velocity2(msg->linear.x, 0.0f, msg->angular.z));
}

void CogniDriveRos::onRosInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ROS_INFO("CogniDriveRos::onRosInitialPose(): forwarding ROS initialpose (%2.2f/%2.2f/%2.2f deg) to MIRA", msg->pose.pose.position.x, msg->pose.pose.position.y, mira::rad2deg(tf::getYaw(msg->pose.pose.orientation)));

    mira::PoseCov2 pose;
    Converter::ros2mira(msg, &pose);
    auto s = mMiraAuthority->queryServicesForInterface("ILocalization");
    if(!s.empty())
    {
      auto result = mMiraAuthority->callService<void>(s.front(), "setInitPose", pose);
      result.timedWait(mira::Duration::seconds(1));
      result.get(); // causes exception if something went wrong.
    }
}

bool CogniDriveRos::onRosLoadMap(cognidrive_ros::LoadMap::Request &req, cognidrive_ros::LoadMap::Response &res)
{
    ROS_INFO("CogniDriveRos::onRosLoadMap(): request to load map: %s", req.map.c_str());
    
    auto services = mMiraAuthority->queryServicesForInterface("IMapLoader");
    if(!services.empty())
    {
        auto result = mMiraAuthority->callService<void>(services.front(), "loadMap", std::string("/maps/static/Map"), std::string(req.map + ".xml"), std::string("/GlobalFrame"));
        result.timedWait(mira::Duration::seconds(5));
	try
	{
	    result.get(); // causes exception if something went wrong.
	    res.result = std::string("OK: MIRA loaded map: ") + req.map + std::string(".xml");
	    ROS_INFO("CogniDriveRos::onRosLoadMap(): successful, sending back response: %s", res.result.c_str());
	}
	catch(mira::Exception e)
	{
	    res.result = std::string("ERROR, could not load map: \"") + req.map + std::string(".xml\": ") + e.what();
	    ROS_INFO("CogniDriveRos::onRosLoadMap(): failed, sending back response: %s", res.result.c_str());
	}
    }
    else
    {
        res.result = std::string("ERROR, could not find MIRA maploader to load map: ") + req.map + std::string(".xml");
	ROS_INFO("CogniDriveRos::onRosLoadMap(): failed, sending back response: %s", res.result.c_str());
    }

    // We always return true, even if the map loading failed, because when returning false,
    // the caller doesn't even our response.status indicating what the problem is. See
    // http://www.ros.org/wiki/roscpp/Overview/Services
    return true;
}

int CogniDriveRos::exec()
{
    // do the locomotion
    ROS_INFO("CogniDriveRos::exec(): starting event loop...");
    ros::spin();
    ROS_INFO("CogniDriveRos::exec(): quit ROS event loop...");

    return 0;
}