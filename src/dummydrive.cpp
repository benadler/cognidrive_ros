#include <dummydrive.h>

DummyDrive::DummyDrive(ros::Publisher publisher)
{
    mRosPublisherDriveCommand = publisher;
}

void DummyDrive::setVelocity(const mira::Velocity2& v)
{
    // Called each time Pilot sends a drive command
    // v.x() contains translational part
    // v.phi() contains rotational part
    geometry_msgs::Twist msg;
    msg.linear.x = v.x();
    msg.angular.z = v.phi();
    mRosPublisherDriveCommand.publish(msg);
}