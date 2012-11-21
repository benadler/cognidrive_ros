#include <movebaseaction.h>

MoveBaseAction::MoveBaseAction(mira::Authority* miraAuthority)
{
    mMiraAuthority = miraAuthority;
    ros::NodeHandle nodeHandle("move_base");
    mActionServer = new MoveBaseActionServer(ros::NodeHandle(), "move_base", /*autostart*/false);

    //register the goal and feeback callbacks
    mActionServer->registerGoalCallback(boost::bind(&MoveBaseAction::actionGoalCallBack, this));
    mActionServer->registerPreemptCallback(boost::bind(&MoveBaseAction::preemptCallBack, this));

    mActionServer->start();


    // we can subscribe to a ROS topic or MIRA channel of interest for generating feedback
    //mSubscriber = mRosNodeHandle.subscribe("/random_number", 1, &MoveBaseAction::analysisCB, this);
    mMiraAuthority->subscribe<std::string>("/navigation/PilotEvent", &MoveBaseAction::onCogniDriveStatus, this);

    // This is mildly confusing: Normally, we offer an actionlib interface. But tool slike rviz
    // want to send simple PoseStamped messages over a topic to send goals, because they don't
    // care about progress and the ability to cancel (wtf?). So, we offer the move_base_simple
    // topic to receive those messages. In the callback receiving those messages, we simply
    // reformat those PoseStamped messages to be ActionGoals and send them to ourself as goals.
    ros::NodeHandle nodeHandleSimpleMove("/move_base_simple");
    //nodeHandleSimpleMove.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBaseAction::simplePoseCallBack, this, _1));
    mRosSubSimplePose = nodeHandleSimpleMove.subscribe("goal", 1, &MoveBaseAction::simplePoseCallBack, this);
    mActionGoalPublisher = nodeHandle.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);

    ROS_INFO("MoveBaseAction::MoveBaseAction(): started action server.");
}

MoveBaseAction::~MoveBaseAction(void)
{
  // tell MIRA to cancel driving by setting an empty task
  auto services = mMiraAuthority->queryServicesForInterface("INavigation");
  if(!services.empty())
  {
    auto result = mMiraAuthority->callService<void>(services.front(), "setTask", boost::shared_ptr<mira::navigation::Task>());
    result.timedWait(mira::Duration::seconds(1));
    result.get(); // causes exception if something went wrong.
  }

  // set the action state to preempted
  mActionServer->setPreempted();

  mActionServer->shutdown();
  delete mActionServer;
}

void MoveBaseAction::simplePoseCallBack(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
    move_base_msgs::MoveBaseActionGoal actionGoal;
    actionGoal.header.stamp = ros::Time::now();
    actionGoal.goal.target_pose = *goal;

    ROS_INFO("MoveBaseAction::simplePoseCallBack(): wrapping the PoseStamped (%2.2f/%2.2f/%2.2f deg) in an action message (%2.2f/%2.2f/%2.2f deg) and re-sending as goal to the server.",
	     goal->pose.position.x,
	     goal->pose.position.y,
	     mira::rad2deg(tf::getYaw(goal->pose.orientation)),
	     actionGoal.goal.target_pose.pose.position.x,
	     actionGoal.goal.target_pose.pose.position.y,
	     mira::rad2deg(tf::getYaw(actionGoal.goal.target_pose.pose.orientation))
	    );

    mActionGoalPublisher.publish(actionGoal);
}

  // Called when a new goal is set, simply accepts the goal
  void MoveBaseAction::actionGoalCallBack()
  {
    // accept the new goal - do I have to cancel a pre-existing one first?
    mGoal = mActionServer->acceptNewGoal()->target_pose;

    ROS_INFO("MoveBaseAction::actionGoalCallBack(): driving to %2.2f/%2.2f/%2.2f", mGoal.pose.position.x, mGoal.pose.position.y, mira::rad2deg(tf::getYaw(mGoal.pose.orientation)));

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
	  tf::getYaw(mGoal.pose.orientation),
	  mira::deg2rad(10.0f) // tolerance
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

    // Find the service responsible for driving
    auto services = mMiraAuthority->queryServicesForInterface("INavigation");
    if(!services.empty())
    {
      /*auto result = */mMiraAuthority->callService<void>(services.front(), "setTask", task);
      ROS_INFO("MoveBaseAction::actionGoalCallBack(): MIRA-task set.");
//       result.timedWait(mira::Duration::seconds(10));
//       result.get(); // causes exception if something went wrong.
      ROS_INFO("MoveBaseAction::actionGoalCallBack(): MIRA-task set successfully.");
    }
    else
    {
      ROS_ERROR("MoveBaseAction::actionGoalCallBack(): couldn't find MIRA Navigation service, cannot drive.");
    }
  }

  // This action is event driven, the action code only runs when the callbacks occur therefore
  // a preempt callback is created to ensure that the action responds promptly to a cancel request.
  // The callback function takes no arguments and sets preempted on the action server.
  // I wonder whether this needs to be mutexed.
  void MoveBaseAction::preemptCallBack()
  {
    ROS_ERROR("MoveBaseAction::preemptCallBack(): preempted.");

    // tell MIRA to cancel driving by setting an empty task
    auto services = mMiraAuthority->queryServicesForInterface("INavigation");
    if(!services.empty())
    {
      auto result = mMiraAuthority->callService<void>(services.front(), "setTask", boost::shared_ptr<mira::navigation::Task>());
      result.timedWait(mira::Duration::seconds(1));
      result.get(); // causes exception if something went wrong.
    }

    // set the action state to preempted
    mActionServer->setPreempted();
  }

  void MoveBaseAction::onCogniDriveStatus(mira::ChannelRead<std::string> data)
  {
    std::string status = data->value();
    ROS_INFO("MoveBaseAction::onCogniDriveStatus(): status changed to %s.", status.c_str());
    // make sure that the action hasn't been canceled
    if (!mActionServer->isActive())
      return;


    if(status == "GoalReached")
    {
        ROS_INFO("MoveBaseAction::onCogniDriveStatus(): driving succeeded.");

        // set the action state to succeeded - there is no result
        mActionServer->setSucceeded();
    }
    else if(status == "NoPathPlannable" || status == "NoValidMotionCommand" || status == "NoData")
    {
	ROS_INFO("MoveBaseAction::onCogniDriveStatus(): driving aborted, reason: %s", status.c_str());
        // set the action state to aborted
        mActionServer->setAborted();
    }
  }

/* old, from tutorial:
  // Takes format of subscribed data, puts relevant data into actionserver's feedback channel
  void MoveBaseAction::analysisCB(const std_msgs::Float32::ConstPtr& msg)
  {
    // make sure that the action hasn't been canceled
    if (!mActionServer->isActive())
      return;

    data_count_++;
    mMoveBaseFeedback.sample = data_count_;
    mMoveBaseFeedback.data = msg->data;
    //compute the std_dev and mean of the data
    sum_ += msg->data;
    mMoveBaseFeedback.mean = sum_ / data_count_;
    sum_sq_ += pow(msg->data, 2);
    mMoveBaseFeedback.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(mMoveBaseFeedback.mean, 2)));
    mActionServer->publishFeedback(mMoveBaseFeedback);

    if(data_count_ > mGoal)
    {
      mMoveBaseResult.mean = mMoveBaseFeedback.mean;
      mMoveBaseResult.std_dev = mMoveBaseFeedback.std_dev;

      if(mMoveBaseResult.mean < 5.0)
      {
        ROS_INFO("%s: Aborted", mActionName.c_str());
        //set the action state to aborted
        mActionServer->setAborted(mMoveBaseResult);
      }
      else
      {
        ROS_INFO("%s: Succeeded", mActionName.c_str());
        // set the action state to succeeded
        mActionServer->setSucceeded(mMoveBaseResult);
      }
    }
  }
  */