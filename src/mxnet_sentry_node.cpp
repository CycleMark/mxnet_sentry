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


using namespace mxnet_actionlib;
using json = nlohmann::json;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int lChargeStatus = -1;
double lRobotChargeLevel = 0;
double lMaxRobotChargeLevel = 164;
int lHomeWayPoint = 0;
float lWaypointSleepWait = 5.0;
std::string cmdString = "STOP";
nlohmann::json robot_navigation_json;
nlohmann::json robot_config_json;
move_base_msgs::MoveBaseGoal HomeStationGoal;

const double PI = (double)3.14159265358979;

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

void doneCb(const actionlib::SimpleClientGoalState &state,
			const AutoDockingResultConstPtr &result)
{
	ROS_INFO("Docking Finished in state [%s]", state.toString().c_str());
	ROS_INFO("Docking Answer: %s", result->text.c_str());
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
	ROS_INFO("Docking Got Feedback state %s", feedback->state.c_str());
	ROS_INFO("Docking Got Feedback text %s", feedback->text.c_str());
}

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
	lChargeStatus = msg->charger;
	lRobotChargeLevel = msg->battery;
}

// Load Waypoints from JSON file
bool loadWayPoints()
{
	ROS_INFO("Loading Robot Navigation Waypoints");

	std::ifstream jsonWayPointFile("/home/mark/mxnetRobotics/kobukirobot/waypoints/robot_waypoints.json");

	try
	{
		json jsonWayPointData;
       	robot_navigation_json = nlohmann::json::parse(jsonWayPointFile);

		ROS_INFO("Waypoints Parsed.");

        std::cout << robot_navigation_json << std::endl;
	}
     catch (json::exception& e)
     {
         // output exception information
         std::cout << "message: " << e.what() << '\n'
                   << "exception id: " << e.id << std::endl;
	 }

	// Get the number of Waypoints in the file
	std::cout << "WayPoints Loaded: " << robot_navigation_json["waypoints"]["waypoint"].size() << std::endl;
}

// Load Waypoints from JSON file
bool loadRobotConfiguration()
{
	ROS_INFO("Loading Robot Configuration");

	std::ifstream jsonWayPointFile("/home/mark/mxnetRobotics/kobukirobot/config/robot_config.json");
	
	try
	{
		json jsonConfigData;
       	robot_config_json = nlohmann::json::parse(jsonWayPointFile);

		ROS_INFO("Configuration Parsed.");
        std::cout << robot_config_json << std::endl;
	}
     catch (json::exception& e)
     {
         // output exception information
         std::cout << "message: " << e.what() << '\n'
                   << "exception id: " << e.id << std::endl;
	 }
}


bool buildNextGoal(move_base_msgs::MoveBaseGoal& goal, int WaypointID, nlohmann::json robotdata_json)
{
	ROS_INFO("buildNextGoal");

	// If we can come to the end of the Waypoint list - retrun the home goal	
	if (robot_navigation_json["waypoints"]["waypoint"].size() == WaypointID)
	{
		ROS_INFO("Waypoints Completed - Returning Home Goal");
		goal =  HomeStationGoal;

		// Each waypoint needs to be in the frame ID of the map
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		return true;
	}
	
	goal.target_pose.pose.position.x = robotdata_json["waypoints"]["waypoint"][WaypointID]["xposition"];
	goal.target_pose.pose.position.y = robotdata_json["waypoints"]["waypoint"][WaypointID]["yposition"];

	double goal_angle = robotdata_json["waypoints"]["waypoint"][WaypointID]["angle"];
	geometry_msgs::Quaternion goal_quant = tf::createQuaternionMsgFromYaw(goal_angle);
	goal.target_pose.pose.orientation = goal_quant;

	// Each waypoint needs to be in the frame ID of the map
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	ROS_INFO("Waypoint No. %i", WaypointID);
	std::cout << "Waypoint Name. %s" << robotdata_json["waypoints"]["waypoint"][WaypointID]["description"] << std::endl;

	ROS_INFO("Sending goal");

	ROS_INFO("WayPoint PosX: %f", goal.target_pose.pose.position.x);
	ROS_INFO("Waypoint PosY: %f", goal.target_pose.pose.position.y);
	ROS_INFO("Waypoint Orientation: %f", goal.target_pose.pose.orientation);

	return true;
}

// https://answers.ros.org/question/361940/ros-robot-moves-to-certain-coordinate-point-but-how-to-stop-and-set-multiple-goals/
// http://wiki.ros.org/p2os-purdue/Tutorials/C%2B%2B%20Velocity%20Controller%20for%20P2OS%20Robots
// https://www.programcreek.com/python/example/70235/geometry_msgs.msg.Point

bool toGoal(ros::Publisher vel_pub, geometry_msgs::Point goal_point, double tolerance)
{

	geometry_msgs::Twist vel_msg;

	tf::TransformListener localListener;
	tf::StampedTransform localTransform;
	geometry_msgs::Point currentLocation;
	geometry_msgs::Point orginalLocation;

	double currentRotation;
	double lDistanceToGoal = 0.0, angle_to_goal, theta = 0.0, lLastDistanceToGoal = 0.0;
	bool lWaitingForTransform = true;

	ros::Rate loop_rate(10);

	while (lWaitingForTransform == true)
	{
		try
		{
			ROS_INFO("Waiting For Transform...");
			localListener.lookupTransform("/map", "/base_footprint", ros::Time(0), localTransform);
			lWaitingForTransform = false;

		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	currentLocation.x = localTransform.getOrigin().x();
	currentLocation.y = localTransform.getOrigin().y();
	orginalLocation.x = currentLocation.x;
	orginalLocation.y = currentLocation.y;

	currentRotation = tf::getYaw(localTransform.getRotation());
	

	double xDifference = goal_point.x - currentLocation.x;
	double yDiffernece = goal_point.y - currentLocation.y;

	angle_to_goal = atan2(yDiffernece, xDifference); // this is our "bearing to goal" as I can guess

	lDistanceToGoal = sqrt(xDifference * xDifference + yDiffernece * yDiffernece);

	lWaitingForTransform = true;
	ROS_INFO("Distance to Goal - abs(): %d", fabs(lDistanceToGoal));
	ROS_INFO("Distance to Goal : %d", (lDistanceToGoal));

	ROS_INFO("Pausing...");
	sleep(10);

	if (!ros::ok())
	{
		exit(1);
	}

	while (lWaitingForTransform == true)
	{
		try
		{
			lLastDistanceToGoal = lDistanceToGoal;
			
			while ((fabs(lDistanceToGoal) >= 0.1 && ros::ok()))
			{
				localListener.lookupTransform("/map", "/base_footprint", ros::Time(0), localTransform);

				currentLocation.x = localTransform.getOrigin().x();
				currentLocation.y = localTransform.getOrigin().y();
				currentRotation = tf::getYaw(localTransform.getRotation());

				xDifference = goal_point.x - currentLocation.x;
				yDiffernece = goal_point.y - currentLocation.y;

				angle_to_goal = atan2(yDiffernece, xDifference); // this is our "bearing to goal" as I can guess

				lDistanceToGoal = sqrt(xDifference * xDifference + yDiffernece * yDiffernece);
				lWaitingForTransform = false;
					
				ROS_INFO("******Charging Reversing LOCATION... ******\r\n Goal PosX: %f\r\n Goal PosY: %f\r\n \r\n Orginal PosX: %f\r\n Orginal PosY: %f\r\n AngleToGoal: %f\r\n Current PosX: %f\r\n Current PosY: %f\r\n X Diff: %f\r\n Y Diff: %f\r\n Distance To Goal: %f\r\n *******************************\r\n", 
						goal_point.x, goal_point.y, orginalLocation.x, orginalLocation.y, angle_to_goal, currentLocation.x, currentLocation.y, xDifference, yDiffernece, lDistanceToGoal);

				ROS_INFO("Distance to Goal - fabs(): %f", lDistanceToGoal);
				std::cout << "cout Distance To Goal: " << lDistanceToGoal << std::endl;

				if (lDistanceToGoal >= 0.1)
				{
					ROS_INFO("Distance to Goal - fabs()...NOT met:");
				}
				else
				{
					ROS_INFO("Distance to Goal - fabs()...MET:");
				}

				if (lDistanceToGoal >= 0.1)
				{
					/*if (abs(angle_to_goal - theta) > 0.1)
					{
						vel_msg.linear.x = 0.0;
						vel_msg.angular.z = 0.1;
					}
					else */
					{
						vel_msg.linear.x = -0.1;
						vel_msg.linear.y = 0;
						vel_msg.linear.z = 0;
						vel_msg.angular.x = 0;
						vel_msg.angular.y = 0;
						//vel_msg.angular.z = Kh * (atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta);
						vel_msg.angular.z = 0.0;
						ROS_INFO("Reversing...\r\n");
						vel_pub.publish(vel_msg);
					}
				}
				if (lDistanceToGoal > lLastDistanceToGoal)
				{
					ROS_INFO("DISTANCE MEASUREMENT ERROR...\r\n");
					break;					
				}
					
				lLastDistanceToGoal = lDistanceToGoal;

				ros::spinOnce();
				loop_rate.sleep();
			}
			return true;
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	return false;
}


// Test Procedure - Initial purpose to work out why, when backing out of the charging station it keeps miss calcuating 
// the distance travelled - and missing the point.
bool testMethod()
{

	geometry_msgs::Twist vel_msg;

	tf::TransformListener localListener;
	tf::StampedTransform localTransform;
	geometry_msgs::Point currentLocation;
	geometry_msgs::Point orginalLocation;

	double currentRotation;
	double lDistanceToGoal, angle_to_goal, theta = 0.0;
	bool lWaitingForTransform = true;

	ros::Rate loop_rate(10);

	while (lWaitingForTransform == true)
	{
		try
		{
			ROS_INFO("Waiting For Transform...");
			localListener.lookupTransform("/map", "/base_footprint", ros::Time(0), localTransform);
			lWaitingForTransform = false;

			currentLocation.x = localTransform.getOrigin().x();
			currentLocation.y = localTransform.getOrigin().y();

			// Record Starting Position so we can calculate how far the robot has travelled.
			orginalLocation.x = currentLocation.x;
			orginalLocation.y = currentLocation.y;

			currentRotation = tf::getYaw(localTransform.getRotation());
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	double xDifference = currentLocation.x - orginalLocation.x;
	double yDiffernece = currentLocation.y - orginalLocation.y;

	angle_to_goal = atan2(yDiffernece, xDifference); // this is our "bearing to goal" as I can guess

	lDistanceToGoal = sqrt(xDifference * xDifference + yDiffernece * yDiffernece);

	lWaitingForTransform = true;
	ROS_INFO("Distance to Goal - abs(): %d", fabs(lDistanceToGoal));
	ROS_INFO("Distance to Goal : %d", (lDistanceToGoal));

	while (lWaitingForTransform == true)
	{
		try
		{
			while ((cmdString != "STOP" && ros::ok()))
			{
				localListener.lookupTransform("/map", "/base_footprint", ros::Time(0), localTransform);

				currentLocation.x = localTransform.getOrigin().x();
				currentLocation.y = localTransform.getOrigin().y();
				currentRotation = tf::getYaw(localTransform.getRotation());

				xDifference = orginalLocation.x - currentLocation.x;
				yDiffernece = orginalLocation.y- currentLocation.y;

				angle_to_goal = atan2(yDiffernece, xDifference); // this is our "bearing to goal" as I can guess

				lDistanceToGoal = sqrt(xDifference * xDifference + yDiffernece * yDiffernece);
				lWaitingForTransform = false;
					
				ROS_INFO("******Data Points... ****** Orginal PosX: %f\r\n Orginal PosY: %f\r\n AngleToGoal: %f\r\n Current PosX: %f\r\n Current PosY: %f\r\n X Diff: %f\r\n Y Diff: %f\r\n Distance To Goal: %f\r\n *******************************\r\n", 
						orginalLocation.x, orginalLocation.y, angle_to_goal, currentLocation.x, currentLocation.y, xDifference, yDiffernece, lDistanceToGoal);

				ROS_INFO("Distance Travelled - fabs(): %f", lDistanceToGoal);
				std::cout << "cout Distance To Goal: " << lDistanceToGoal << std::endl;

				sleep(1);
				ros::spinOnce();
				loop_rate.sleep();
			}
			return true;
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	return false;
}


// Backing out of the docking station is (for now) a special case as we don't want
// the path planning and obstical avoidance kicking in. Here we just reverse up but a
// desired amount and stop
bool backoutFromDockingStation(int distanceToReverse)
{
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simple_navigation_goals");

	//tell the action client that we want to spin a thread by default

	ros::NodeHandle nodeHandle;

	ros::Subscriber sub = nodeHandle.subscribe("/mobile_base/sensors/core", 10, kobukiSensorsCoreCallback);
	ros::Subscriber laserScanSub = nodeHandle.subscribe<sensor_msgs::LaserScan>("base_scan", 10, scanCallback);
	ros::Subscriber commandSubcriber = nodeHandle.subscribe("/robotCommand", 10, commandCallback);

	ros::Publisher vel_pub = nodeHandle.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

	ros::AsyncSpinner spinner(4);
	spinner.start();

	MoveBaseClient ac("move_base", true);
	actionlib::SimpleActionClient<mxnet_actionlib::AutoDockingAction> chargingStationClient("fibonacci", true);

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
	bool lWaitingForTransform = true;
	double xStartingPos = 0.0;
	double yStartingPos = 0.0;
	double startingRotation = 0.0;
	double goal_angle = 0.0;
	double RobotRadius = 0.16;
	const double PI = (double)3.14159265358979;
	double lDistanceToReverse = 1.20; //Measure in meters
	double xTargetPos = 0.0;
	double yTargetPos = 0.0;
	
	tf::StampedTransform transform;
	geometry_msgs::Quaternion goal_quant;

	// Keep looping. Initially checking if the command has been sent to start a patrol sequence
	ROS_INFO("Robot Online...");
	stopMotorServiceClient.call(srv);
	
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
		// Kill the process
		else if (cmdString == "TEST")
		{
			ROS_INFO("Mxnet Sentry Node Entered Test Mode...");

			bool lGoalStatus = testMethod();			
		}
		else
		{
			// Command has been sent to start moving - so load waypoints and config file
			// (ensures dynamic update of waypoints)
			
			loadWayPoints();
			loadRobotConfiguration();

			double lBatteryLowLevel = robot_config_json["config"]["batteryLowLevel"];
			double lMaxRobotChargeLevel = robot_config_json["config"]["maxRobotChargeLevel"];
			ROS_INFO("Battery Low Level Set @ %f", lBatteryLowLevel);
			ROS_INFO("Battery Max Charge Level Set @ %f", lMaxRobotChargeLevel);

			// Once the command is sent to start moving - get the starting location, save it as the HOME
			// location, back out of the docking station and run WayPoint sqeuence.
			startMotorServiceClient.call(srv);			

			ROS_INFO("Getting Starting Location...");

			while (lWaitingForTransform == true)
			{
				try
				{
					listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
					xStartingPos = transform.getOrigin().x();
					yStartingPos = transform.getOrigin().y();
					startingRotation = tf::getYaw(transform.getRotation());

					/* ROS_INFO("****** STARTING LOCATION ******");
					ROS_INFO("Starting PosX: %f", xStartingPos);
					ROS_INFO("Starting PosY: %f", yStartingPos);
					ROS_INFO("Starting Orientation: %f", startingRotation);
					ROS_INFO("*******************************\n");*/

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

			// Minus Pi (180 degrees) so that we know the component is behind the robot
			double lHorizontalComponent = lDistanceToReverse * cos(startingRotation - PI);
			double lVerticalComponent = lDistanceToReverse * sin(startingRotation - PI);

			xTargetPos = xStartingPos + lHorizontalComponent;
			yTargetPos = yStartingPos + lVerticalComponent;

			

			ROS_INFO("****** TARGET UNDOCKED LOCATION ******\r\n Target PosX: %f\r\n Target PosY: %f\r\n Target Orientation: %f\r\n *******************************\r\n", xTargetPos, yTargetPos, startingRotation);
			sleep(lWaypointSleepWait);

			// Now set the return to base location. We will make this 0.5m back from the starting
			// starting locaton (idea being that it will give the robot a chance to doc sucessfully

			// Waypoint to store revers from base position
			double xChargeBase = xTargetPos;
			double yChargeBase = yTargetPos;
			double chargeBaseAngle = startingRotation;

			geometry_msgs::Quaternion chargeBase_quant = tf::createQuaternionMsgFromYaw(chargeBaseAngle);

			HomeStationGoal.target_pose.pose.position.x = xChargeBase;
			HomeStationGoal.target_pose.pose.position.y = yChargeBase;
			HomeStationGoal.target_pose.pose.orientation = chargeBase_quant;

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
			int lCtr = robot_navigation_json["waypoints"]["waypoint"].size() ;

			lHomeWayPoint = robot_navigation_json["waypoints"]["waypoint"].size();
			ROS_INFO("Waypoints Loaded: %i", lHomeWayPoint);

			// Return to base position (before executing docking request)

			int lWayPointNumber = 0;
			int lNoOfWayPoints = lCtr +1;

			//Reverse out of the charging station to the return position
			goal = HomeStationGoal;

			ROS_INFO("Backing out of Charging Station");

			//Each waypoint needs to be in the frame ID of the map
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();

			ROS_INFO("Sending goal");

			//bool lGoalStatus = toGoal(vel_pub, goal.target_pose.pose.position, 0.5);

			ac.sendGoal(goal);

			//ac.sendGoal(goal, &goalCompletePlanCallBack, &goalActiveCallBack, &goalFeedbackCallback);
			//ac.getState();

			ac.waitForResult();

			/*if (lGoalStatus == true)
			{
				ROS_INFO("Undocked Sucessfully");
			}
			else
			{

			}*/

			if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Undocked Sucessfully");
				ROS_INFO("WayPoint PosX: %f", HomeStationGoal.target_pose.pose.position.x);
				ROS_INFO("Waypoint PosY: %f", HomeStationGoal.target_pose.pose.position.y);
				ROS_INFO("Waypoint Orientation: %f", HomeStationGoal.target_pose.pose.orientation);
			}
			else
			{
				ROS_INFO("Failed to Undock");
			}

			sleep(lWaypointSleepWait);

			bool lBatteryTooLow = false;

			while (ros::ok() && cmdString != "STOP")
			{
				double lBatteryCapacity = (lRobotChargeLevel / lMaxRobotChargeLevel) * 100.0;

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
				if (lChargeStatus == 2)
				{

				}

				ROS_INFO("Battery Capacity: %f", lBatteryCapacity);

				// Itterate over the stored WayPoints

				buildNextGoal(goal, lWayPointNumber, robot_navigation_json);
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
					
					std::regex rgx("Waypoint:[^0-9]*([0-9]+).*");
					std::smatch match;
					const std::string s = cmdString;

					if (std::regex_search(s.begin(), s.end(), match, rgx))
					{
						cmdString = ""; // Clear the command string (Else we will keep looping)
						
						lWayPointNumber = std::stoi(match[1]);
						ROS_DEBUG("Waypoint Redirect Received. New Waypoint %i", lWayPointNumber);

						ac.cancelGoal();

						buildNextGoal(goal, lWayPointNumber, robot_navigation_json);

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

				//ac.waitForResult();
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

					// Each waypoint needs to be in the frame ID of the map
					goal = HomeStationGoal;
					goal.target_pose.header.frame_id = "map";
					goal.target_pose.header.stamp = ros::Time::now();					

					ROS_INFO("Sending goal");

					// Issue Waypoint command to go home/return to base
					ROS_INFO("WayPoint PosX: %f", goal.target_pose.pose.position.x );
					ROS_INFO("Waypoint PosY: %f", goal.target_pose.pose.position.y);
					ROS_INFO("Waypoint Orientation: %f", goal.target_pose.pose.orientation);

					ac.sendGoal(goal);

					ac.waitForResult();

					if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					{
						ROS_INFO("Charge - Waypoint Reached Sucessfully");
					}
					else
					{
						ROS_INFO("Charge - Failed to Reach Waypoint");
					}
				}
				else if (lWayPointNumber < (lNoOfWayPoints - 1))
				{
					lWayPointNumber++;
				}
				else
				{
					lWayPointNumber = 0;
				}

				bool lRobotDockedAndCharging = false;

				// Have we arrived back at the docking station. If so then proceed to dock
				ROS_INFO("Executing WayPoint Number %i:", lWayPointNumber);

				int lRedockingCtr = 0;

				// If we have completed a route or the battery to too low return to base and dock
				if ((lWayPointNumber == 0 && (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)) 
					|| lBatteryTooLow == true)
				{
					while (lRobotDockedAndCharging == false)
					{
						ROS_INFO("Returning To Charging Station...Docking In Progress");

						mxnet_actionlib::AutoDockingGoal chargingStationGoal;
						//  	    chargingStationClient.sendGoal(chargingStationGoal);
						chargingStationClient.sendGoal(chargingStationGoal, &doneCb, &activeCb, &feedbackCb);

						bool finished_before_timeout = chargingStationClient.waitForResult(ros::Duration(240.0));

						actionlib::SimpleClientGoalState dockingState = chargingStationClient.getState();
						ROS_INFO("State: %s", dockingState.toString().c_str());

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
							if (lChargeStatus != 2)
							{
								bool lAtUndockLocation = false;

								while (lAtUndockLocation == false)
								{
									//Reverse out of the charging station to the return position
									goal = HomeStationGoal;
									ROS_INFO("Attempting re-Docking Procedure");
									lRedockingCtr++;

									//Each waypoint needs to be in the frame ID of the map
									goal.target_pose.header.frame_id = "map";
									goal.target_pose.header.stamp = ros::Time::now();

									ROS_INFO("Sending goal");
									ac.sendGoal(goal);
									ac.waitForResult();

									if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
									{
										ROS_INFO("******************************");
										ROS_INFO("Successfully Returned to Undock Location...");
										ROS_INFO("WayPoint PosX: %d", HomeStationGoal.target_pose.pose.position.x);
										ROS_INFO("Waypoint PosY: %d", HomeStationGoal.target_pose.pose.position.y);
										ROS_INFO("Waypoint Orientation: %d", HomeStationGoal.target_pose.pose.orientation);
										ROS_INFO("Stabalisation Pause...");
										ROS_INFO("******************************");

										sleep(lWaypointSleepWait);
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

					//sleep(3600); //sleep to charge up (TODO - sense charge status
					ROS_INFO("Redocking Attempts: %i", lRedockingCtr);
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
