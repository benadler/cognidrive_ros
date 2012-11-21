#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <fw/Framework.h>
#include <navigation/tasks/PositionTask.h>
#include <navigation/tasks/OrientationTask.h>
#include <navigation/tasks/PreferredDirectionTask.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

class MoveBaseAction
{
public:

  MoveBaseAction(mira::Authority* miraAuthority);
  ~MoveBaseAction(void);

  // Called when a new goal is set, simply accepts the goal
  void actionGoalCallBack();

  // Called when e.g. rviz sends us a simple goal.
  void simplePoseCallBack(const geometry_msgs::PoseStamped::ConstPtr& goal);

  // This action is event driven, the action code only runs when the callbacks occur therefore
  // a preempt callback is created to ensure that the action responds promptly to a cancel request.
  // The callback function takes no arguments and sets preempted on the action server.
  void preemptCallBack();

  // Takes format of subscribed data, puts relevant data into actionserver's feedback channel
//   void analysisCB(const std_msgs::Float32::ConstPtr& msg);

  void onCogniDriveStatus(mira::ChannelRead<std::string> data);

protected:
//   ros::NodeHandle* mRosNodeHandle;
  MoveBaseActionServer* mActionServer;
  ros::Publisher mActionGoalPublisher;
  mira::Authority* mMiraAuthority;
  geometry_msgs::PoseStamped mGoal;

  ros::Subscriber mRosSubSimplePose;

  // it seems move_base2 has no feedback. I find this to be strange...
  //cognidrive_ros::MoveBaseFeedback mMoveBaseFeedback;
  move_base_msgs::MoveBaseResult mMoveBaseResult;

  ros::Subscriber mSubscriber;
};