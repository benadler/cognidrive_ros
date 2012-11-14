#ifndef COGNIDRIVEROS_H
#define COGNIDRIVEROS_H

#include <algorithm>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <fw/Framework.h>
#include <robot/RangeScan.h>


#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <pr2_msgs/BatteryServer2.h>

#include <converter.h>
#include <dummydrive.h>
#include <movebaseaction.h>

class CogniDriveRos
{
private:
    DummyDrive* mDummyDrive;
    MoveBaseAction* mMoveBaseAction;

    ros::NodeHandle* mRosNodeHandle;

    tf::TransformBroadcaster* mRosTransformBroadcaster;

    // necessary publishers for ROS topics
    ros::Publisher mRosPubLaserScan;
    ros::Publisher mRosPubOdometry;
    ros::Publisher mRosPubBatteryState;

    // subscribe to the relevant ros topics
    ros::Subscriber mRosSubLaserScan;
    ros::Subscriber mRosSubOdometry;

    // create and start the mira framework
    // add -c scitos....xml to start cognidrive here
    mira::Framework* mMiraFramework;

    // create mira authority and publish the babble channel
    mira::Authority* mMiraAuthority;

    mira::Channel<mira::robot::RangeScan> mMiraChannelRangeScan;
    mira::Channel<mira::robot::Odometry2> mMiraChannelOdometry;

    unsigned int mNumberOfPacketsMira2RosLaserScan;
    unsigned int mNumberOfPacketsMira2RosOdometry;
    unsigned int mNumberOfPacketsMira2RosBattery;

    void onMiraLaserScan(mira::ChannelRead<mira::robot::RangeScan> data);
    void onMiraOdometry(mira::ChannelRead<mira::robot::Odometry2> data);
    void onMiraBatteryState(mira::ChannelRead<mira::robot::BatteryState> data);
    void onRosLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg);
    void onRosOdometry(const nav_msgs::Odometry::ConstPtr& msg);

public:
    CogniDriveRos(int argc, char **argv);
    ~CogniDriveRos();

    int exec();
};

#endif