#include <movebaseaction.h>

MoveBaseAction::MoveBaseAction(std::string name, mira::Authority* miraAuthority) :
  mRosNodeHandle(),
  mActionServer(mRosNodeHandle, name, false),
  mActionName(name)
{
    mMiraAuthority = miraAuthority;
    //register the goal and feeback callbacks
    mActionServer.registerGoalCallback(boost::bind(&MoveBaseAction::goalCB, this));
    mActionServer.registerPreemptCallback(boost::bind(&MoveBaseAction::preemptCB, this));

    mMiraAuthority->subscribe<std::string>("PilotEvent", &MoveBaseAction::onCogniDriveStatus, this);

    // we could subscribe to a ROS topic of interest for generating feedback
    //mSubscriber = mRosNodeHandle.subscribe("/random_number", 1, &MoveBaseAction::analysisCB, this);
    mActionServer.start();
}

  MoveBaseAction::~MoveBaseAction(void)
  {
  }

  // Called when a new goal is set, simply accepts the goal
  void MoveBaseAction::goalCB()
  {
    // accept the new goal
    mGoal = mActionServer.acceptNewGoal()->target_pose;

    // actually tell MIRA/cognidrive to move - see http://www.mira-project.org/MIRA-doc/domains/navigation/Pilot/
    boost::shared_ptr<mira::navigation::Task> task(new mira::navigation::Task());

    // Tell MIRA where to go.
    // The tolerances describe the radii around the target region. MinTolerance defines
    // the radius around the target where the pathplanner deems the robot to have
    // arrived. But then, the OrientationTask might still move the robot at the target,
    // again moving it away from the target further than MaxTolerance. This would cause
    // the pathplanner to become active again, leading to an endless oscillation.
    task->addSubTask(
      mira::navigation::SubTaskPtr(
	new mira::navigation::PositionTask(
	  mira::Point2f(
	    mGoal.pose.position.x,
	    mGoal.pose.position.y
	  ),
	  0.1f, // minimum tolerance??
	  0.2f  // maximum tolerance??
	)
      )
    );

    // Tell MIRA how to turn
    task->addSubTask(
      mira::navigation::SubTaskPtr(
	new mira::navigation::OrientationTask(
	  mira::deg2rad(tf::getYaw(mGoal.pose.orientation)),
	  mira::deg2rad(2.0f) // tolerance
	)
      )
    );

    // Tell MIRA to go forward only (optional, test it!)
    task->addSubTask(
      mira::navigation::SubTaskPtr(
	new mira::navigation::PreferredDirectionTask(
	  mira::navigation::PreferredDirectionTask::FORWARD,
	  0.5f // the cost for driving into the not-preferred direction
	)
      )
    );

    mMiraAuthority->callService<void>("/robot/navigation/Pilot", "setTask", task);
  }

  // This action is event driven, the action code only runs when the callbacks occur therefore
  // a preempt callback is created to ensure that the action responds promptly to a cancel request.
  // The callback function takes no arguments and sets preempted on the action server.
  // I wonder whether this needs to be mutexed.
  void MoveBaseAction::preemptCB()
  {
    ROS_INFO("%s: Preempted", mActionName.c_str());

    // tell MIRA to cancel driving - is it ok to just set an empty task?
    mMiraAuthority->callService<void>("/robot/navigation/Pilot", "setTask", boost::shared_ptr<mira::navigation::Task>());

    // set the action state to preempted
    mActionServer.setPreempted();
  }

  void MoveBaseAction::onCogniDriveStatus(mira::ChannelRead<std::string> data)
  {
    // make sure that the action hasn't been canceled
    if (!mActionServer.isActive())
      return;

    std::string status = data->value();

    if(status.compare("GoalReached") != 0)
    {
        ROS_INFO("%s: Succeeded", mActionName.c_str());

        // set the action state to succeeded - there is no result
        mActionServer.setSucceeded();
    }
    else if(status.compare("NoPathPlannable") != 0 || status.compare("NoValidMotionCommand") != 0 || status.compare("NoData") != 0)
    {
	ROS_INFO("%s: Aborted, reason: %s", mActionName.c_str(), status.c_str());
        // set the action state to aborted
        mActionServer.setAborted();
    }
  }
/*
  // Takes format of subscribed data, puts relevant data into actionserver's feedback channel
  void MoveBaseAction::analysisCB(const std_msgs::Float32::ConstPtr& msg)
  {
    // make sure that the action hasn't been canceled
    if (!mActionServer.isActive())
      return;

    data_count_++;
    mMoveBaseFeedback.sample = data_count_;
    mMoveBaseFeedback.data = msg->data;
    //compute the std_dev and mean of the data
    sum_ += msg->data;
    mMoveBaseFeedback.mean = sum_ / data_count_;
    sum_sq_ += pow(msg->data, 2);
    mMoveBaseFeedback.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(mMoveBaseFeedback.mean, 2)));
    mActionServer.publishFeedback(mMoveBaseFeedback);

    if(data_count_ > mGoal)
    {
      mMoveBaseResult.mean = mMoveBaseFeedback.mean;
      mMoveBaseResult.std_dev = mMoveBaseFeedback.std_dev;

      if(mMoveBaseResult.mean < 5.0)
      {
        ROS_INFO("%s: Aborted", mActionName.c_str());
        //set the action state to aborted
        mActionServer.setAborted(mMoveBaseResult);
      }
      else
      {
        ROS_INFO("%s: Succeeded", mActionName.c_str());
        // set the action state to succeeded
        mActionServer.setSucceeded(mMoveBaseResult);
      }
    }
  }
  */