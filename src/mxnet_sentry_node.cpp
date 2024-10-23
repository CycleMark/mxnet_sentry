// Mxnet Robotics - 2020

#include <ros/ros.h>
#include <iostream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include </home/mark/ros_ws/devel_isolated/kobuki_msgs/include/kobuki_msgs/SensorState.h>
#include <mxnet_actionlib/AutoDockingAction.h>
#include <tf/transform_listener.h>
#include <actionlib/client/terminal_state.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <regex>
#include <nlohmann/json.hpp>
#include <fstream>
#include "turtlesim/Pose.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "MxnetSentryNode.h"
#include "MxnetConfigLoader.h"
#include <memory.h>
#include <chrono>
#include <signal.h>

using namespace mxnet_actionlib;
using json = nlohmann::json;

int * lChargeStatus = new(int);//-1;
//std::unique_ptr<int> lChargeStatus = new(-1);
//*lChargeStatus = -1;
double lRobotChargeLevel = 0;
float lWaypointSleepWait = 5.0;
std::string cmdString = "STOP";
move_base_msgs::MoveBaseGoal HomeStationGoal;

const double PI = (double)3.14159265358979;

void mxnetSigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

double Deg2Rad(double Degrees)
{
	return (Degrees / 180) * PI;
}

double Rad2Deg(double Radians)
{
	return (Radians / PI) * 180;
}

// Callback function for
void commandCallback(const std_msgs::String::ConstPtr &cmdMsg)
{
	ROS_INFO("Command Recieved...: [%s]", cmdMsg->data.c_str());
	cmdString = cmdMsg->data.c_str();
}

// Callback Methods for Goal Planning

void goalCompletePlanCallBack(const actionlib::SimpleClientGoalState &state, 
						  const move_base_msgs::MoveBaseActionResultConstPtr &result)
{
	//move_base_msgs::MoveBaseGoal goal;
	ROS_INFO("Goal Plan Complete");
}

void goalActiveCallBack()
{
	ROS_INFO("Planning Goal Active");
}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{

}


// Callback Methods for Auto Docking Planning

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	double lScanDistance = 0.0;
	int lScanAngle = 0;

	for (int laserPointCtr = 0; lScanAngle <= scan->angle_max; laserPointCtr++)
	{
		lScanAngle += scan->angle_increment;

		if (std::isnan(scan->ranges[laserPointCtr]))
		{
			continue;
		}

		// Work out the average free space behind the robot
		//if (lScanAngle < && lScanAngle <)
		{
			lScanDistance += scan->ranges[laserPointCtr];
		}
	}
}

// Called every time feedback is received for the goal
void kobukiSensorsCoreCallback(const kobuki_msgs::SensorState::ConstPtr &msg)
{
	//ROS_INFO("Charger Status: [%i]", msg->charger);
	*lChargeStatus = msg->charger;
	lRobotChargeLevel = msg->battery;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simple_navigation_goals");

	std::unique_ptr<MxnetSentryNode> pMxnetSentryNode(new MxnetSentryNode);
	std::unique_ptr<MxnetConfigLoader> pMxnetConfigLoader(new MxnetConfigLoader);
	std::unique_ptr<MxnetNavigationConfig> pMxnetNavigationConfig(new MxnetNavigationConfig);

	*lChargeStatus =-1;
	//tell the action client that we want to spin a thread by default

	ros::NodeHandle nodeHandle;

	signal(SIGINT, mxnetSigintHandler);

	ros::Subscriber sub = nodeHandle.subscribe("/mobile_base/sensors/core", 10, kobukiSensorsCoreCallback);
	ros::Subscriber laserScanSub = nodeHandle.subscribe<sensor_msgs::LaserScan>("base_scan", 10, scanCallback);
	ros::Subscriber commandSubcriber = nodeHandle.subscribe("/robotCommand", 10, commandCallback);
	ros::Publisher vel_pub = nodeHandle.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
    ros::Publisher intialPosePublisher = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1);
	ros::AsyncSpinner spinner(4);
	spinner.start();

	MoveBaseClient ac("move_base", true);

	tf::TransformListener listener;

	ros::ServiceClient stopMotorServiceClient = nodeHandle.serviceClient<std_srvs::Empty>("/stop_motor");
	ros::ServiceClient startMotorServiceClient = nodeHandle.serviceClient<std_srvs::Empty>("/start_motor");
	std_srvs::Empty srv;

	//wait for the action server to come up
	while (!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}
 
	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	
	// Find out starting position
	double goal_angle = 0.0;
	double RobotRadius = 0.16;
	const double PI = (double)3.14159265358979;
	
	tf::StampedTransform transform;
	geometry_msgs::Quaternion goal_quant;

	// Keep looping. Initially checking if the command has been sent to start a patrol sequence
	ROS_INFO("Robot Online...");
	stopMotorServiceClient.call(srv);

	pMxnetSentryNode->loadWayPoints();
	pMxnetConfigLoader->loadRobotConfiguration();
	pMxnetSentryNode->setInitialPose(intialPosePublisher);

	double lBatteryLowLevel = pMxnetConfigLoader->getBatteryLowLevel();
	double lMaxRobotChargeLevel = pMxnetConfigLoader->getMaxRobotChargeLevel();

	ROS_INFO("Battery Low Level Set @ %f", lBatteryLowLevel);
	ROS_INFO("Battery Max Charge Level Set @ %f", lMaxRobotChargeLevel);

	while (ros::ok())
	{
		// Wait until the command string to start it given
		if (cmdString == "STOP")
		{
			while (cmdString == "STOP")
			{
//				stopMotorServiceClient.call(srv);
				ROS_INFO("Waiting for Start Command");
				sleep(1);
			}
		}
		// Kill the process
		else if (cmdString == "TERMINATE")
		{
			ROS_INFO("Mxnet Sentry Node Terminating...");
			stopMotorServiceClient.call(srv);

			sleep(5);

			ros::shutdown();
			exit(0);
		}
		// Enter Test Method
		else if (cmdString == "TEST")
		{
			ROS_INFO("Mxnet Sentry Node Entered Test Mode...");

		//	bool lGoalStatus = testMethod();			
		}
		else if (cmdString == "SETPOSE")
		{
			ROS_INFO("Setting Pose");

			pMxnetSentryNode->loadWayPoints();
			pMxnetSentryNode->setInitialPose(intialPosePublisher);
			cmdString = "STOP"; // Clear the command string (Else we will keep looping)
		}
		else if (cmdString == "HOME")
		{
			ROS_INFO("Returning To Base");
			pMxnetSentryNode->returnToBase(&ac, HomeStationGoal);
			cmdString = "STOP"; // Clear the command string (Else we will keep looping)
		}
		// Have we been sent an explicit instruction to move to a given Waypoint
		else if (pMxnetSentryNode->extractWaypointID(cmdString) != -1)
		{
			int lCommandWayPointNumber = pMxnetSentryNode->extractWaypointID(cmdString);

			if (lCommandWayPointNumber != -1)
			{
				ROS_INFO("Starting LIDAR Motor");
				startMotorServiceClient.call(srv);	
				std::chrono::seconds(2);

				pMxnetSentryNode->executeWaypointCommand(&ac, cmdString, lChargeStatus, lCommandWayPointNumber, HomeStationGoal);

				ROS_INFO("Stopping LIDAR Motor");
				stopMotorServiceClient.call(srv);
			}

		}
		else if (cmdString == "GO")
		{
			// Command has been sent to start moving - so load waypoints and config file
			// (ensures dynamic update of waypoints)
			
			pMxnetSentryNode->loadWayPoints();
			pMxnetConfigLoader->loadRobotConfiguration();
			lBatteryLowLevel = pMxnetConfigLoader->getBatteryLowLevel();
			lMaxRobotChargeLevel = pMxnetConfigLoader->getMaxRobotChargeLevel();

			ROS_INFO("Battery Low Level Set @ %f", lBatteryLowLevel);
			ROS_INFO("Battery Max Charge Level Set @ %f", lMaxRobotChargeLevel);

			// Once the command is sent to start moving - get the starting location, save it as the HOME
			// location, back out of the docking station and run WayPoint sqeuence.
			startMotorServiceClient.call(srv);			

			pMxnetSentryNode->setInitialPose(intialPosePublisher);


			//////////////////////////////////////////////////
			//
			// Way Points for Patroling from Charging Unit (CU) route is:
			//
			// (CU)) -> Waypoint 1 -> Waypoint n+1 -> Waypoint x -> 
			// CU & Charge
			//
			// Charge docking location is calculated as 30cm
			// back from charging station
			//
			//////////////////////////////////////////////////
			int lNoOfWaypoints = pMxnetSentryNode->getNoOfWaypoints();
			ROS_INFO("Waypoints Loaded: %i", lNoOfWaypoints);

			// Return to base position (before executing docking request)

			int lWayPointNumber = 0;
			
			int lNoOfWayPoints = lNoOfWaypoints +1;

			//Reverse out of the charging station to the return position
			if (pMxnetSentryNode->undockRobot(&ac, HomeStationGoal))
			{
				ROS_INFO("Command Received and Undocked OK");
			}

			sleep(lWaypointSleepWait);

			bool lBatteryTooLow = false;

			while (ros::ok() && cmdString != "STOP")
			{
				double lBatteryCapacity = (lRobotChargeLevel /  pMxnetConfigLoader->getMaxRobotChargeLevel()) * 100.0;

				try
				{
					listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
					ROS_INFO("Robots Current Location: ");
					ROS_INFO("PosX: %f", transform.getOrigin().x());
					ROS_INFO("PosY: %f", transform.getOrigin().y());
					ROS_INFO("Orientation: %f", tf::getYaw(transform.getRotation()));
				}
				catch (tf::TransformException ex)
				{
					ROS_ERROR("%s", ex.what());
					ros::Duration(1.0).sleep();
				}

				// If we're in the charging station reverse out
				if (*lChargeStatus == 2)
				{

				}

				ROS_INFO("Battery Capacity: %f", lBatteryCapacity);

				// Itterate over the stored WayPoints

				if (pMxnetSentryNode->buildNextGoal(goal, lWayPointNumber) == false)
				{
					// This waypoint was set to inactive - skipping
					lWayPointNumber++;
					continue;
				}

				ac.sendGoal(goal);

				actionlib::SimpleClientGoalState navigationState = ac.getState();
				ROS_INFO("Navigation State: %s", navigationState.toString().c_str());

				// Monitor the state of the Navigation. If there is a change in plan/command
				// i.e. cmdString changes then this will enable the robot to react immediately 
				// rather than wait for the goal to complete.
				while (navigationState == actionlib::SimpleClientGoalState::ACTIVE ||
					   navigationState == actionlib::SimpleClientGoalState::PENDING)
				{
					navigationState = ac.getState();
					ROS_DEBUG("Navigation State: %s", navigationState.toString().c_str());
					
					//Respond to changes sent across the wire
					int lCommandWayPointNumber = pMxnetSentryNode->extractWaypointID(cmdString);
					if (lCommandWayPointNumber != -1)
					{
						cmdString = "STOP"; // Clear the command string (Else we will keep looping)
						ac.cancelGoal();

						pMxnetSentryNode->buildNextGoal(goal, lCommandWayPointNumber);
						ROS_INFO("Received Command Waypoint %i", lCommandWayPointNumber);

						ac.sendGoal(goal);
					}

					sleep(1); // Putting sleep here serves two purposes 1) We're not running round this loop the while time 
							  // 2) If there is a WayPoint Change/Redirect then it will give time for the ActionClient to 
							  // send the goal to the server before the loop loops again

					if (cmdString == "HOME")
					{
						ac.cancelGoal();
					}
				}

				// For now just pause 5 seconds at each waypoint
				if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					ROS_INFO("Waypoint Reached Sucessfully");
					
					sleep(lWaypointSleepWait);
					ROS_INFO("Pausing at WayPoint...");
				}
				else
				{
					ROS_INFO("Failed to Reach Waypoint");
				}

				if (lBatteryCapacity < lBatteryLowLevel || cmdString == "HOME")
				{
					if (lBatteryCapacity < lBatteryLowLevel)
					{
						ROS_WARN("Battery Too Low - Returning To Base");						
					}
					else
					{
						ROS_WARN("HOME Command Issued - Returning To Base");
					}

					lBatteryTooLow = true;

					pMxnetSentryNode->returnToBase(&ac, HomeStationGoal);
				}
				else if (lWayPointNumber < (lNoOfWayPoints - 1))
				{
					lWayPointNumber++;
					ROS_INFO("Next Waypoint: %i of %i", lWayPointNumber, lNoOfWayPoints );

				}
				else
				{
					ROS_INFO("Waypoint Set to Zero");
					lWayPointNumber = 0;
				}

				// Have we arrived back at the docking station. If so then proceed to dock
				ROS_INFO("Executing WayPoint Number %i:", lWayPointNumber);

				// If we have completed a route or the battery to too low return to base and dock
				if ((lWayPointNumber == 0 && (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)) 
					|| lBatteryTooLow == true)
				{
					pMxnetSentryNode->dockRobot(&ac, lChargeStatus);

					ROS_INFO("Stopping LIDAR Motor");
					stopMotorServiceClient.call(srv);

					ROS_INFO("Mxnet Sentry Node Completed Run. Waiting until next wake command...");

					// This will put the process back into a wait state.
					cmdString = "STOP";

				}
			}
		}
	}

	ROS_INFO("Robot Going Offline");
	sleep(1);
	return 0;
}
