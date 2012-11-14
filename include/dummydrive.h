#ifndef DUMMYDRIVE_H
#define DUMMYDRIVE_H

#include <ros/ros.h>
#include <fw/Framework.h>
#include <robot/IDrive.h>
#include <transform/Velocity.h>
#include <geometry_msgs/Twist.h>

class DummyDrive : public mira::robot::IDrive
{
private:
    ros::Publisher mRosPublisherDriveCommand;

public:
    DummyDrive(ros::Publisher publisher);

    template <typename Reflector>
    void reflect(Reflector& r)
    {
	mira::robot::IDrive::reflect(r);
    }

    virtual void setVelocity(const mira::Velocity2& v);
};

#endif