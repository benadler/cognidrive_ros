#ifndef CONVERTER_H
#define CONVERTER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <pr2_msgs/BatteryState2.h>

#include <fw/Framework.h>
#include <robot/BatteryState.h>
#include <robot/RangeScan.h>
#include <robot/Odometry.h>

struct Converter
{
    static mira::robot::BatteryState ros2mira(const pr2_msgs::BatteryState2& batteryRos)
    {
        bool present = batteryRos.present;
	mira::robot::BatteryState batteryMira;

	return batteryMira;
    }

    static pr2_msgs::BatteryServer2 mira2ros(const mira::robot::BatteryState& batteryMira)
    {
	pr2_msgs::BatteryServer2 batteryRos;

	batteryRos.id = 1;
	batteryRos.last_system_update = ros::Time::now();
	batteryRos.time_left = ros::Duration(batteryMira.lifeTime * 60, 0);
	batteryRos.average_charge = batteryMira.lifePercent;

	if(batteryMira.voltage > 24.0f)
	  batteryRos.message = "Battery good.";
	else if(batteryMira.voltage > 22.1f)
	  batteryRos.message = "Starting shutdown when voltage drops by another " + boost::lexical_cast<std::string>(batteryMira.voltage - 22.0f) + " volts";
	else
	  batteryRos.message = "Shutting down due to low battery voltage.";

	batteryRos.last_controller_update = ros::Time::now();
	batteryRos.battery.resize(1);
	batteryRos.battery[0].present = true;
	batteryRos.battery[0].charging = batteryMira.charging;
	batteryRos.battery[0].discharging = batteryMira.current > 0.5f; // battery charging: current < 0, discharging: current > 0
	batteryRos.battery[0].power_present = batteryMira.voltage > 22.0f;
	batteryRos.battery[0].power_no_good = batteryMira.voltage < 22.0f;
	batteryRos.battery[0].inhibited = false;
	batteryRos.battery[0].last_battery_update = ros::Time::now();

	std::fill(batteryRos.battery[0].battery_register.begin(), batteryRos.battery[0].battery_register.end(), 0);

	// the registers have never been updated
	std::fill(batteryRos.battery[0].battery_update_flag.begin(), batteryRos.battery[0].battery_update_flag.end(), false);

	// I hope ros doesn't choke when the registers have never been updated, but the last update was just now
	std::fill(batteryRos.battery[0].battery_register_update.begin(), batteryRos.battery[0].battery_register_update.end(), ros::Time::now());

	return batteryRos;
    }

    static nav_msgs::Odometry mira2ros(const mira::robot::Odometry2& odomMira)
    {
	nav_msgs::Odometry odomRos;

	// convert!

	return odomRos;
    }

    static mira::robot::Odometry2 ros2mira(const nav_msgs::Odometry& odomRos)
    {
	// mira::robot::Odometry2 only has pose.{x,y} and velocity.{x,y}
	mira::robot::Odometry2 odomMira;

	// this conversion is almost certainly wrong.
	odomMira.pose.x() = odomRos.pose.pose.position.x;
	odomMira.pose.y() = odomRos.pose.pose.position.y;

	odomMira.velocity.x() = odomRos.twist.twist.linear.x;
	odomMira.velocity.y() = odomRos.twist.twist.linear.y;

	return odomMira;
    }

    static sensor_msgs::LaserScan mira2ros(const mira::robot::RangeScan& scanMira)
    {
	static unsigned int seq = 0;
	sensor_msgs::LaserScan scanRos;

	scanRos.header.seq = seq++;

	// the caller can replace this with the timestamp from the mira channel
	scanRos.header.stamp = ros::Time::now();

	// the caller should replace this with the frame from the mira channel
	scanRos.header.frame_id = "laser_frame";

	scanRos.angle_min = scanMira.startAngle.value();
	scanRos.angle_max = scanMira.startAngle.value() + (scanMira.range.size() * scanMira.deltaAngle.rad());
	scanRos.angle_increment = scanMira.deltaAngle.rad();

	scanRos.time_increment = 0.0f; // mira doesn't know :|
	scanRos.scan_time = 0.0f; // mira doesn't know :|

	scanRos.range_min = scanMira.minimumRange;
	scanRos.range_max = scanMira.minimumRange;

	scanRos.ranges.resize(scanMira.range.size());
	for(unsigned int i = 0; i < scanMira.range.size(); ++i)
	    scanRos.ranges[i] = scanMira.range[i];

	scanRos.intensities.resize(scanMira.reflectance.size());
	for(unsigned int i = 0; i < scanMira.reflectance.size(); ++i)
	    scanRos.intensities[i] = scanMira.reflectance[i];

	return scanRos;
    }

    static mira::robot::RangeScan ros2mira(const sensor_msgs::LaserScan& scanRos)
    {
	mira::robot::RangeScan scanMira;

	scanMira.startAngle.setValue(scanRos.angle_min);
	scanMira.deltaAngle.setValue(scanRos.angle_increment);
	scanMira.coneAngle.setValue(scanRos.angle_increment); // not really correct
	scanMira.aperture = 0.0f;
	scanMira.stdError = 0.0f;
	scanMira.minimumRange = scanRos.range_min;
	scanMira.maximumRange = scanRos.range_max;

	scanMira.range.resize(scanRos.ranges.size());
	for(unsigned int i = 0; i < scanRos.ranges.size(); ++i)
	    scanMira.range[i] = scanRos.ranges[i];

	// Mask rays originating by the robot scannig itself. I just assume
	// that the first and last 10 degrees are problematic, we should
	// measure this correctly!
	scanMira.valid.resize(scanRos.ranges.size());
	for(unsigned int i = 0; i < scanRos.ranges.size(); ++i)
	  if(i < 20 || i > scanRos.ranges.size() - 20)
	    scanMira.valid[i] = mira::robot::RangeScan::Masked;
	  else
	    scanMira.valid[i] = mira::robot::RangeScan::Valid;

	scanMira.reflectance.resize(scanRos.intensities.size());
	for(unsigned int i = 0; i < scanRos.intensities.size(); ++i)
	    scanMira.reflectance[i] = scanRos.intensities[i];

	return scanMira;
    }
};


#endif