#ifndef COGNIDRIVEROS_H
#define COGNIDRIVEROS_H

#include <algorithm>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <fw/Framework.h>
#include <robot/RangeScan.h>
#include <maps/OccupancyGrid.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <pr2_msgs/BatteryServer2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <converter.h>
#include <dummydrive.h>
#include <movebaseaction.h>

class CogniDriveRos
{
private:
    DummyDrive* mDummyDrive;
    MoveBaseAction* mMoveBaseAction;
    
    bool mSimulation;

    ros::NodeHandle* mRosNodeHandle;

    tf::TransformBroadcaster* mRosTransformBroadcaster;

    // necessary publishers for ROS topics
    ros::Publisher mRosPubLaserScanFront;
    ros::Publisher mRosPubLaserScanRear;
    ros::Publisher mRosPubOdometry;
    ros::Publisher mRosPubBatteryState;
    ros::Publisher mRosPubMapMetaData;
    ros::Publisher mRosPubMapOccupancyGrid;

    // subscribe to the relevant ros topics
    ros::Subscriber mRosSubInitialPose; // to allow setting an initial pose estimate in e.g. rviz, just like for amcl
    ros::Subscriber mRosSubLaserScan;
    ros::Subscriber mRosSubOdometry;
    ros::Subscriber mRosSubCmdVel;

    mira::Framework* mMiraFramework;

    // create mira authority and publish the babble channel
    mira::Authority* mMiraAuthority;

    mira::Channel<mira::robot::RangeScan> mMiraChannelRangeScan;
    mira::Channel<mira::robot::Odometry2> mMiraChannelOdometry;

    unsigned int mNumberOfPacketsMira2RosLaserScan;
    unsigned int mNumberOfPacketsMira2RosOdometry;
    unsigned int mNumberOfPacketsMira2RosBattery;
    unsigned int mNumberOfPacketsMira2RosMapMetaData;
    unsigned int mNumberOfPacketsMira2RosMapOccupancyGrid;

    void onMiraLaserScanFront(mira::ChannelRead<mira::robot::RangeScan> data);
    void onMiraLaserScanRear(mira::ChannelRead<mira::robot::RangeScan> data);
    void onMiraOdometry(mira::ChannelRead<mira::robot::Odometry2> data);
    void onMiraBatteryState(mira::ChannelRead<mira::robot::BatteryState> data);
    
    void onMiraMap(mira::ChannelRead<mira::maps::OccupancyGrid> data);
   
    void onRosLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg);
    void onRosOdometry(const nav_msgs::Odometry::ConstPtr& msg);
    void onRosCmdVel(const geometry_msgs::Twist::ConstPtr& msg);
    void onRosInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg); // to allow setting an initial pose estimate in e.g. rviz, just like for amcl

public:
    CogniDriveRos(int argc, char **argv);
    ~CogniDriveRos();

    int exec();
};

#endif