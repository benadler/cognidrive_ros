#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>
#include <cognidrive_ros/MoveBaseAction.h>
#include <fw/Framework.h>
#include <navigation/tasks/PositionTask.h>
#include <navigation/tasks/OrientationTask.h>
#include <navigation/tasks/PreferredDirectionTask.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

class MoveBaseAction
{
public:

  MoveBaseAction(ros::NodeHandle* nodeHandle, mira::Authority* miraAuthority);
  ~MoveBaseAction(void);

  // Called when a new goal is set, simply accepts the goal
  void goalCB();

  // This action is event driven, the action code only runs when the callbacks occur therefore
  // a preempt callback is created to ensure that the action responds promptly to a cancel request.
  // The callback function takes no arguments and sets preempted on the action server.
  void preemptCB();

  // Takes format of subscribed data, puts relevant data into actionserver's feedback channel
//   void analysisCB(const std_msgs::Float32::ConstPtr& msg);

  void onCogniDriveStatus(mira::ChannelRead<std::string> data);

protected:
  ros::NodeHandle* mRosNodeHandle;
  actionlib::SimpleActionServer<cognidrive_ros::MoveBaseAction>* mActionServer;
  std::string mActionName;
  mira::Authority* mMiraAuthority;
  geometry_msgs::PoseStamped mGoal;

  // it seems move_base2 has no feedback. I find this to be strange...
  //cognidrive_ros::MoveBaseFeedback mMoveBaseFeedback;
  cognidrive_ros::MoveBaseResult mMoveBaseResult;

  ros::Subscriber mSubscriber;
};