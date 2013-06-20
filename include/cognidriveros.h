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


#include "cognidrive_ros/LoadMap.h" // our service, generated from LoadMap.srv
#include "cognidrive_ros/GetInRoller.h"
#include "cognidrive_ros/GetOutRoller.h"
#include "cognidrive_ros/StopMotorRoller.h"
#include "cognidrive_ros/DockOff.h"
#include "cognidrive_ros/DockOn.h"
#include "cognidrive_ros/SetInitPose.h"

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
    ros::Publisher mRosEmergency;
    ros::Publisher mRosBumper;
    ros::Publisher mRosPubBatteryState;
    ros::Publisher mRosPubMapMetaData;
    ros::Publisher mRosPubMapOccupancyGrid;
    ros::Publisher mRosPubRollerStatus;

    // subscribe to the relevant ros topics
    ros::Subscriber mRosSubInitialPose; // to allow setting an initial pose estimate in e.g. rviz, just like for amcl
    ros::Subscriber mRosSubLaserScan;
    ros::Subscriber mRosSubOdometry;
    ros::Subscriber mRosSubCmdVel;
	ros::Subscriber mRosSubResEmergencyStop;
	ros::Subscriber mRosSubEmergencyStop;
    
    // A service that e.g. peis_ros can call to change the map (forwarded to MIRA)
    ros::ServiceServer mRosServiceLoadMap;
    ros::ServiceServer mRosServiceGetInRoller;
    ros::ServiceServer mRosServiceGetOutRoller;
    ros::ServiceServer mRosServiceStopMotorRoller;
    ros::ServiceServer mRosServiceDockOn;
    ros::ServiceServer mRosServiceDockOff;

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
    void onMiraMotorStatus(mira::ChannelRead<uint8> data);
    void onMiraBatteryState(mira::ChannelRead<mira::robot::BatteryState> data);
    void onMiraMap(mira::ChannelRead<mira::maps::OccupancyGrid> data);
    void onMiraRollerStatus(mira::ChannelRead<std::string> data);

    bool onRosLoadMap(cognidrive_ros::LoadMap::Request &req, cognidrive_ros::LoadMap::Response &res);
    bool onGetInRoller(cognidrive_ros::GetInRoller::Request &req, cognidrive_ros::GetInRoller::Response &res);
    bool onGetOutRoller(cognidrive_ros::GetOutRoller::Request &req, cognidrive_ros::GetOutRoller::Response &res);
    bool onStopMotorRoller(cognidrive_ros::StopMotorRoller::Request &req, cognidrive_ros::StopMotorRoller::Response &res);
    bool onDockOn(cognidrive_ros::DockOn::Request &req, cognidrive_ros::DockOn::Response &res);
    bool onDockOff(cognidrive_ros::DockOff::Request &req, cognidrive_ros::DockOff::Response &res);

    void onRosLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg);
    void onRosOdometry(const nav_msgs::Odometry::ConstPtr& msg);
    void onRosCmdVel(const geometry_msgs::Twist::ConstPtr& msg);
    void onResetEmergencyStop(const std_msgs::Bool::ConstPtr& msg);
    void onRequestEmergencyStop(const std_msgs::Bool::ConstPtr& msg);
    void onRosInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg); // to allow setting an initial pose estimate in e.g. rviz, just like for amcl

    tf::Transform miraPoseToRosTransform(const mira::Pose2* const pose)
    {
      tf::Transform transform;
      tf::Quaternion q;
      q.setEuler(0.0f, 0.0f, pose->phi());
      transform.setRotation(q);
      transform.setOrigin(tf::Vector3(pose->x(), pose->y(), 0.0));
      return transform;
    }

    tf::Transform miraPoseToRosTransform(const mira::Pose3* const pose)
    {
      tf::Transform transform;
      tf::Quaternion q;
      q.setEuler(0.0f, 0.0f, pose->yaw());
      transform.setRotation(q);
      transform.setOrigin(tf::Vector3(pose->x(), pose->y(), pose->z()));
      return transform;
    }

public:
    CogniDriveRos(int argc, char **argv);
    ~CogniDriveRos();

    int exec();
};

#endif
