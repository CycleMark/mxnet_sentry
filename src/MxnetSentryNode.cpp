#include "MxnetSentryNode.h"
#include <memory.h>

using namespace mxnet_actionlib;
using json = nlohmann::json;

auto MxnetSentryNode::waitForTransform (double& xStartingPos, double& yStartingPos, double& startingRotation )
{
    tf::TransformListener lListener;
    tf::StampedTransform lTransform;
	bool lWaitingForTransform = true;

    while (lWaitingForTransform == true)
    {
        try
        {
            lListener.lookupTransform("/map", "/base_footprint", ros::Time(0), lTransform);
            xStartingPos = lTransform.getOrigin().x();
            yStartingPos = lTransform.getOrigin().y();
            startingRotation = tf::getYaw(lTransform.getRotation());

            ROS_INFO("****** STARTING LOCATION ******\r\n Starting PosX: %f\r\n Starting PosY: %f\r\n Starting Orientation: %f\r\n *******************************\r\n", xStartingPos, yStartingPos, startingRotation);

            // Break out of this now we have starting position.
            lWaitingForTransform = false;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    return lWaitingForTransform;
};

move_base_msgs::MoveBaseGoal MxnetSentryNode::calculateHomeGoal()
{   	
	double xStartingPos = 0.0;
	double yStartingPos = 0.0;
	double startingRotation = 0.0;

    waitForTransform(xStartingPos, yStartingPos, startingRotation);

    ROS_INFO("Getting Starting Location...");

    // Minus Pi (180 degrees) so that we know the component is behind the robot
    double lHorizontalComponent = lDistanceToReverse * cos(startingRotation - PI);
    double lVerticalComponent = lDistanceToReverse * sin(startingRotation - PI);

    xTargetPos = xStartingPos + lHorizontalComponent;
    yTargetPos = yStartingPos + lVerticalComponent;

    ROS_INFO("****** TARGET HOME LOCATION ******\r\n Target PosX: %f\r\n Target PosY: %f\r\n Target Orientation: %f\r\n *******************************\r\n", xTargetPos, yTargetPos, startingRotation);
 
    // Now set the return to base location. We will make this 0.5m back from the starting
    // starting locaton (idea being that it will give the robot a chance to doc sucessfully

    double chargeBaseAngle = startingRotation;

    geometry_msgs::Quaternion chargeBase_quant = tf::createQuaternionMsgFromYaw(chargeBaseAngle);

    mHomeGoal.target_pose.pose.position.x = xTargetPos;
    mHomeGoal.target_pose.pose.position.y = yTargetPos;
    mHomeGoal.target_pose.pose.orientation = chargeBase_quant;

    return mHomeGoal;
}

bool MxnetSentryNode::buildNextGoal(move_base_msgs::MoveBaseGoal& goal, int &WaypointID, bool commandStringDirective)
{
	ROS_INFO("buildNextGoal");

	// If we can come to the end of the Waypoint list - retrun the home goal	
	if (mRobotNavigationJSON["waypoints"]["waypoint"].size() == WaypointID)
	{
		// Determine Navigation Mode

		if (mRobotNavigationJSON["navigationMode"] == "ONCE")
		{
			ROS_INFO("Waypoints Completed - Returning Home Goal");
			goal =  mHomeGoal;

			// Each waypoint needs to be in the frame ID of the map
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();
			return true;
		}
		else
		{
			ROS_INFO("Continious Navigation Mode Detected...");
			WaypointID = 0;
		}

	}

	// Check is this is an active waypoint - if not then just return
	if (mRobotNavigationJSON["waypoints"]["waypoint"][WaypointID]["active"] == false && commandStringDirective == false)
	{
		ROS_INFO("Inactive Waypoint No. %i", WaypointID, " - Skipping");
		return false;
	}
	else
	{
		ROS_INFO("Inactive Waypoint No. %i", WaypointID, " - Command Override");
	}

	
	goal.target_pose.pose.position.x = mRobotNavigationJSON["waypoints"]["waypoint"][WaypointID]["xposition"];
	goal.target_pose.pose.position.y = mRobotNavigationJSON["waypoints"]["waypoint"][WaypointID]["yposition"];

	double goal_angle = mRobotNavigationJSON["waypoints"]["waypoint"][WaypointID]["angle"];
	geometry_msgs::Quaternion goal_quant = tf::createQuaternionMsgFromYaw(goal_angle);
	goal.target_pose.pose.orientation = goal_quant;

	// Each waypoint needs to be in the frame ID of the map
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	ROS_INFO("Waypoint No. %i", WaypointID);
	std::cout << "Waypoint Name. %s" << mRobotNavigationJSON["waypoints"]["waypoint"][WaypointID]["description"] << std::endl;

	ROS_INFO("Sending goal");

	ROS_INFO("WayPoint PosX: %f", goal.target_pose.pose.position.x);
	ROS_INFO("Waypoint PosY: %f", goal.target_pose.pose.position.y);
	ROS_INFO("Waypoint Orientation: %f", goal.target_pose.pose.orientation);

	return true;
}

bool MxnetSentryNode::loadWayPoints(std::string waypointsFile)
{
	ROS_INFO("Loading Robot Navigation Waypoints");

	std::ifstream jsonWayPointFile(waypointsFile);

	try
	{
      	mRobotNavigationJSON = nlohmann::json::parse(jsonWayPointFile);
		ROS_INFO("Waypoints Parsed.");
        std::cout << mRobotNavigationJSON << std::endl;
	}
     catch (json::exception& e)
     {
         // output exception information
         std::cout << "message: " << e.what() << '\n'
                   << "exception id: " << e.id << std::endl;

        return false;
	 }

	// Get the number of Waypoints in the file
	std::cout << "WayPoints Loaded: " << mRobotNavigationJSON["waypoints"]["waypoint"].size() << std::endl;

    mNoOfWaypoints = mRobotNavigationJSON["waypoints"]["waypoint"].size();

    return true;
}

bool MxnetSentryNode::setInitialPose(ros::Publisher initialPosePublisher)
{
	ROS_INFO("Setting Initial Pose");

	double theta = 0;

    std::string fixed_frame = "map";
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = fixed_frame;
    pose.header.stamp = ros::Time::now();

    // set x,y coord
    pose.pose.pose.position.x = mRobotNavigationJSON["startingLocation"]["initial_pose_x"];
    pose.pose.pose.position.y = mRobotNavigationJSON["startingLocation"]["initial_pose_y"];
    pose.pose.pose.position.z = mRobotNavigationJSON["startingLocation"]["initial_pose_a"];

    std::cout << "pose.pose.pose.position.x: " << pose.pose.pose.position.x << std::endl;
	std::cout << "pose.pose.pose.position.y: " << pose.pose.pose.position.y << std::endl;
	std::cout << "pose.pose.pose.position.z: " << pose.pose.pose.position.z << std::endl;

	geometry_msgs::Quaternion goal_quant = tf::createQuaternionMsgFromYaw(mRobotNavigationJSON["startingLocation"]["initial_pose_a"] );
	pose.pose.pose.orientation = goal_quant;

    initialPosePublisher.publish(pose);
	ros::spinOnce();
}


void doneCb(const actionlib::SimpleClientGoalState &state,
			const AutoDockingResultConstPtr &result)
{
	//ROS_INFO("Docking Finished in state [%s]", state.toString().c_str());
	//ROS_INFO("Docking Answer: %s", result->text.c_str());
	//  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
	ROS_INFO("Docking Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const AutoDockingFeedbackConstPtr &feedback)
{
	//ROS_INFO("Docking Got Feedback state %s", feedback->state.c_str());
	//ROS_INFO("Docking Got Feedback text %s", feedback->text.c_str());
}


bool MxnetSentryNode::dockRobot(MoveBaseClient* ac, int* chargingStatus)
{
	int lRedockingCtr = 0;
	bool lRobotDockedAndCharging = false;
	actionlib::SimpleActionClient<mxnet_actionlib::AutoDockingAction> chargingStationClient("mxnet", true);
	move_base_msgs::MoveBaseGoal goal;
	
	sleep (2);

	while (lRobotDockedAndCharging == false)
	{
		ROS_INFO("Returning To Charging Station...Docking In Progress");

		mxnet_actionlib::AutoDockingGoal chargingStationGoal;
		chargingStationClient.sendGoal(chargingStationGoal, &doneCb, &activeCb, &feedbackCb);

		bool finished_before_timeout = chargingStationClient.waitForResult(ros::Duration(240.0));

		ROS_INFO("Docking Goal Sent...");

		actionlib::SimpleClientGoalState dockingState = chargingStationClient.getState();
		ROS_INFO("State: %s", dockingState.toString().c_str());

		ROS_INFO("Returning To Charging Station...Docking Started");
		
		if (finished_before_timeout == false ||
			dockingState != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			//actionlib::SimpleClientGoalState state = chargingStationClient.getState();
			//if (state == RobotDockingState::LOST)
			if (dockingState != actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Docking Lost - Attempting Redocking");
			}

			// The robot hasn't docked. Therefore call the planner to
			// move back and try again.
			if (*chargingStatus != 2)
			{
				bool lAtUndockLocation = false;

				while (lAtUndockLocation == false)
				{
					//Reverse out of the charging station to the return position
					goal = getHomeGoal();
					ROS_INFO("Attempting re-Docking Procedure");
					lRedockingCtr++;

					//Each waypoint needs to be in the frame ID of the map
					goal.target_pose.header.frame_id = "map";
					goal.target_pose.header.stamp = ros::Time::now();

					ROS_INFO("Sending goal");
					ac->sendGoal(goal);
					ac->waitForResult();

					if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					{
						ROS_INFO("******************************");
						ROS_INFO("Successfully Returned to Undock Location...");
						ROS_INFO("WayPoint PosX: %d", goal.target_pose.pose.position.x);
						ROS_INFO("Waypoint PosY: %d", goal.target_pose.pose.position.y);
						ROS_INFO("Waypoint Orientation: %d", goal.target_pose.pose.orientation);
						ROS_INFO("Stabalisation Pause...");
						ROS_INFO("******************************");

						sleep(mWaypointSleepWait);
						lAtUndockLocation = true; // Make sure we're back at the undocking location.
					}
					else
					{
						ROS_INFO("Failed to get to undock location for reattempt - will try again");
						lAtUndockLocation = false;
					}
				}
			} //  if (lChargeStatus != 2)
			else
			{
				// In this case we didn't dock within the timeout - however the robot appears to be
				// charging so don't try re-docking procedure
				ROS_INFO("Docking finished & Charging: %s", dockingState.toString().c_str());
				lRobotDockedAndCharging = true;
			}

		} // (finished_before_timeout == true)
		else
		{
			ROS_INFO("Docking Completed Sucessfully.");
			lRobotDockedAndCharging = true;
		}
	} // while (lRobotDockedAndCharging == false)

	ROS_INFO("Redocking Attempts: %i", lRedockingCtr);

	return lRobotDockedAndCharging;
}