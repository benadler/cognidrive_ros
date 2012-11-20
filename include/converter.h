#ifndef CONVERTER_H
#define CONVERTER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <pr2_msgs/BatteryState2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <fw/Framework.h>
#include <robot/BatteryState.h>
#include <robot/RangeScan.h>
#include <robot/Odometry.h>

struct Converter
{
  /*
    static void ros2mira(const pr2_msgs::BatteryState2& batteryRos, mira::robot::BatteryState* const batteryMira)
    {
      // TODO: implement!
    }
    */

    static void mira2ros(const mira::robot::BatteryState& batteryMira, pr2_msgs::BatteryServer2* const batteryRos)
    {
	batteryRos->id = 1;
	batteryRos->last_system_update = ros::Time::now();
	batteryRos->time_left = ros::Duration(batteryMira.lifeTime * 60, 0);
	batteryRos->average_charge = batteryMira.lifePercent;

	if(batteryMira.voltage > 24.0f)
	  batteryRos->message = "Battery good.";
	else if(batteryMira.voltage > 22.1f)
	  batteryRos->message = "Starting shutdown when voltage drops by another " + boost::lexical_cast<std::string>(batteryMira.voltage - 22.0f) + " volts";
	else
	  batteryRos->message = "Shutting down due to low battery voltage.";

	batteryRos->last_controller_update = ros::Time::now();
	batteryRos->battery.resize(1);
	batteryRos->battery[0].present = true;
	batteryRos->battery[0].charging = batteryMira.charging;
	batteryRos->battery[0].discharging = batteryMira.current > 0.5f; // battery charging: current < 0, discharging: current > 0
	batteryRos->battery[0].power_present = batteryMira.voltage > 22.0f;
	batteryRos->battery[0].power_no_good = batteryMira.voltage < 22.0f;
	batteryRos->battery[0].inhibited = false;
	batteryRos->battery[0].last_battery_update = ros::Time::now();

	std::fill(batteryRos->battery[0].battery_register.begin(), batteryRos->battery[0].battery_register.end(), 0);

	// The registers have never been updated
	std::fill(batteryRos->battery[0].battery_update_flag.begin(), batteryRos->battery[0].battery_update_flag.end(), false);

	// I hope ROS doesn't choke when the registers have never been updated, but the last update was just now
	std::fill(batteryRos->battery[0].battery_register_update.begin(), batteryRos->battery[0].battery_register_update.end(), ros::Time::now());
    }

    static void mira2ros(const mira::robot::Odometry2& odomMira, nav_msgs::Odometry* const odomRos)
    {
	// I have no idea how useful a converted odometry message from MIRA is for ROS,
	// as the MIRA message only contains pose and velocity, while ROS might also
	// need linear and angular twist as well as covariances for both pose and twist.
	// These will be set to zero during the conversion.

	// this conversion is almost certainly wrong.
	odomRos->pose.pose.position.x = odomMira.pose.x();
	odomRos->pose.pose.position.y = odomMira.pose.y();

	odomRos->pose.pose.orientation = tf::createQuaternionMsgFromYaw(odomMira.pose.phi());

	odomRos->twist.twist.linear.x = odomMira.velocity.x();
	odomRos->twist.twist.linear.y = odomMira.velocity.y();

	odomRos->twist.twist.angular.z = odomMira.velocity.phi();
    }

    static void ros2mira(const nav_msgs::Odometry& odomRos, mira::robot::Odometry2* const odomMira)
    {
	// mira::robot::Odometry2 only has pose.{x,y} and velocity.{x,y}
	odomMira->pose.x() = odomRos.pose.pose.position.x;
	odomMira->pose.y() = odomRos.pose.pose.position.y;

	odomMira->velocity.x() = odomRos.twist.twist.linear.x;
	odomMira->velocity.y() = odomRos.twist.twist.linear.y;
    }

    static void mira2ros(const mira::robot::RangeScan& scanMira, sensor_msgs::LaserScan* const scanRos)
    {
	// the caller can replace this with the timestamp from the mira channel
	scanRos->header.stamp = ros::Time::now();

	// the caller should replace this with the frame from the mira channel
	scanRos->header.frame_id = "laser_frame";

	scanRos->angle_min = scanMira.startAngle.value();
	scanRos->angle_max = scanMira.startAngle.value() + (scanMira.range.size() * scanMira.deltaAngle.rad());
	scanRos->angle_increment = scanMira.deltaAngle.rad();

	scanRos->time_increment = 0.0f; // mira doesn't know :|
	scanRos->scan_time = 0.0f; // mira doesn't know :|

	scanRos->range_min = scanMira.minimumRange;
	scanRos->range_max = scanMira.maximumRange;

	scanRos->ranges.resize(scanMira.range.size());
	for(unsigned int i = 0; i < scanMira.range.size(); ++i)
	    scanRos->ranges[i] = scanMira.range[i];

	scanRos->intensities.resize(scanMira.reflectance.size());
	for(unsigned int i = 0; i < scanMira.reflectance.size(); ++i)
	    scanRos->intensities[i] = scanMira.reflectance[i];
    }

    static void ros2mira(const sensor_msgs::LaserScan& scanRos, mira::robot::RangeScan* const scanMira, const unsigned int numberOfOuterRaysToMask = 20)
    {
	scanMira->startAngle.setValue(scanRos.angle_min);
	scanMira->deltaAngle.setValue(scanRos.angle_increment);
	scanMira->coneAngle.setValue(scanRos.angle_increment); // not really correct
	scanMira->aperture = 0.0f;
	scanMira->stdError = 0.0f;
	scanMira->minimumRange = scanRos.range_min;
	scanMira->maximumRange = scanRos.range_max;

	scanMira->range.resize(scanRos.ranges.size());
	for(unsigned int i = 0; i < scanRos.ranges.size(); ++i)
	    scanMira->range[i] = scanRos.ranges[i];

	// Mask rays originating by the robot scannig itself. I just assume
	// that the first and last 10 degrees are problematic, we should
	// measure this correctly!
	scanMira->valid.resize(scanRos.ranges.size());
	for(unsigned int i = 0; i < scanRos.ranges.size(); ++i)
	  if(i < numberOfOuterRaysToMask || i > scanRos.ranges.size() - numberOfOuterRaysToMask)
	    scanMira->valid[i] = mira::robot::RangeScan::Masked;
	  else
	    scanMira->valid[i] = mira::robot::RangeScan::Valid;

	scanMira->reflectance.resize(scanRos.intensities.size());
	for(unsigned int i = 0; i < scanRos.intensities.size(); ++i)
	    scanMira->reflectance[i] = scanRos.intensities[i];
    }

    static void mira2ros(const mira::maps::OccupancyGrid& mapMira, nav_msgs::OccupancyGrid* const mapRos, const mira::Pose2& poseMap)
    {
	mapRos->info.map_load_time = ros::Time::now();
	mapRos->info.resolution = mapMira.getCellSize(); // meters per cell
	mapRos->info.width = mapMira.width();
	mapRos->info.height = mapMira.height();
	mapRos->info.origin.position.x = /*mapMira.getWorldOffset().x() +*/ poseMap.x(); // check whether the former is included in poseMap
	mapRos->info.origin.position.y = /*mapMira.getWorldOffset().y() +*/ poseMap.y(); // check whether the former is included in poseMap
	mapRos->info.origin.position.z = 0.0f;

	// create quaternion from MIRA's poseMap.phi()
	mapRos->info.origin.orientation = tf::createQuaternionMsgFromYaw(poseMap.phi());

	const unsigned int numberOfCells = mapMira.width() * mapMira.height();
	mapRos->data.resize(numberOfCells);

	// MIRA stores a gridcell as a
	//	unsigned 	int 8: 0-254 is probability of occupancy, unknown is 255
	// ROS stores a gridcell as a
	//	signed 		int 8: 0-100 is probability of occupancy, unknown is -1
	// It turns out that int8 -1 = uint8 255, so we can just copy the data.
	//memcpy(&mapRos->data.at(0), mapMira.data(), numberOfCells);

	// cold be parallelized, but is called just once per session
	for(unsigned int i=0;i<numberOfCells;i++)
	{
	  if(mapMira.data()[i] == 255)
	    mapRos->data[i] = -1;
	  else
	    mapRos->data[i] = mapMira.data()[i] / 254;
	}
    }

    static void ros2mira(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, mira::PoseCov2* const poseCov)
    {
      poseCov->x() = msg->pose.pose.position.x;
      poseCov->y() = msg->pose.pose.position.y;

      double roll, pitch, yaw;
      tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

      poseCov->phi() = yaw;

      // covariances
      poseCov->cov.setZero(3,3);
      poseCov->cov(0,0) = msg->pose.covariance[6*0+0];
      poseCov->cov(1,1) = msg->pose.covariance[6*1+1];
      poseCov->cov(2,2) = msg->pose.covariance[6*5+5];
    }
};

#endif