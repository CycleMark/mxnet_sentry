#include <ros/ros.h>
#include <iostream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <../../../devel_isolated/kobuki_msgs/include/kobuki_msgs/SensorState.h>
#include <mxnet_actionlib/AutoDockingAction.h>
#include <tf/transform_listener.h>
#include <actionlib/client/terminal_state.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>

using namespace mxnet_actionlib;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int lChargeStatus = -1;
double lRobotChargeLevel = 0;
double lMaxRobotChargeLevel = 164;
int lHomeWayPoint = 4;
float lWaypointSleepWait = 5.0;
std::string cmdString = "STOP";

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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simple_navigation_goals");

	//tell the action client that we want to spin a thread by default

	ros::NodeHandle nodeHandle;

	ros::Subscriber sub = nodeHandle.subscribe("/mobile_base/sensors/core", 10, kobukiSensorsCoreCallback);
	ros::Subscriber laserScanSub = nodeHandle.subscribe<sensor_msgs::LaserScan>("base_scan", 10, scanCallback);
	ros::Subscriber commandSubcriber = nodeHandle.subscribe("/robotCommand", 10, commandCallback);

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

	move_base_msgs::MoveBaseGoal lWayPoints[10];

	// Find out starting position
	bool lWaitingForTransform = true;
	double xStartingPos = 0.0;
	double yStartingPos = 0.0;
	double startingRotation = 0.0;
	double goal_angle = 0.0;
	double RobotRadius = 0.16;
	const double PI = (double)3.14159265358979;
	double lDistanceToReverse = 0.60; //Measure in meters
	double xTargetPos = 0.0;
	double yTargetPos = 0.0;

	tf::StampedTransform transform;
	geometry_msgs::Quaternion goal_quant;

	// Keep looping. Initially checking if the command has been sent to start a patrol sequence

	while (ros::ok())
	{
		// Wait until the command string to start it given
		if (cmdString == "STOP")
		{
			while (cmdString == "STOP")
			{
				stopMotorServiceClient.call(srv);
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
		else
		{
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

			/*	ROS_INFO("****** TARGET LOCATION ******");
			ROS_INFO("Target PosX: %f", xTargetPos);
			ROS_INFO("Target PosY: %f", yTargetPos);
			ROS_INFO("Target Orientation: %f", startingRotation);
			ROS_INFO("*******************************");
			*/

			ROS_INFO("****** TARGET LOCATION ******\r\n Target PosX: %f\r\n Target PosY: %f\r\n Target Orientation: %f\r\n *******************************\r\n", xTargetPos, yTargetPos, startingRotation);
			sleep(lWaypointSleepWait);

			// Now set the return to base location. We will make this 0.5m back from the starting
			// starting locaton (idea being that it will give the robot a chance to doc sucessfully

			// Waypoint to store revers from base position
			double xChargeBase = xTargetPos;
			double yChargeBase = yTargetPos;
			double chargeBaseAngle = startingRotation;

			geometry_msgs::Quaternion chargeBase_quant = tf::createQuaternionMsgFromYaw(chargeBaseAngle);

			//////////////////////////////////////////////////
			//
			// Way Points for Patroling from Study route is:
			//
			// Study -> Kitchen Windows -> Oven -> Hall ->
			// Loung (facing Window) -> Study & Charge
			//
			// Charge docking location is calculated as 30cm
			// back from charging station
			//
			//////////////////////////////////////////////////
			lWayPoints[0].target_pose.pose.position.x = -2.63; //-2.384;
			lWayPoints[0].target_pose.pose.position.y = 9.24;  //8.440;

			goal_angle = 1.441;
			goal_quant = tf::createQuaternionMsgFromYaw(goal_angle);

			lWayPoints[0].target_pose.pose.orientation = goal_quant;

			lWayPoints[1].target_pose.pose.position.x = -0.751;
			lWayPoints[1].target_pose.pose.position.y = 5.578;

			goal_angle = -1.573;
			goal_quant = tf::createQuaternionMsgFromYaw(goal_angle);

			lWayPoints[1].target_pose.pose.orientation = goal_quant;

			lWayPoints[2].target_pose.pose.position.x = -2.641;
			lWayPoints[2].target_pose.pose.position.y = -0.744;

			goal_angle = -1.679;
			goal_quant = tf::createQuaternionMsgFromYaw(goal_angle);
			lWayPoints[2].target_pose.pose.orientation = goal_quant;

			lWayPoints[3].target_pose.pose.position.x = -4.510;
			lWayPoints[3].target_pose.pose.position.y = -4.066;

			goal_angle = -1.511;
			goal_quant = tf::createQuaternionMsgFromYaw(goal_angle);
			lWayPoints[3].target_pose.pose.orientation = goal_quant;

			// Return to base position (before executing docking request)
			lWayPoints[lHomeWayPoint].target_pose.pose.position.x = xChargeBase;
			lWayPoints[lHomeWayPoint].target_pose.pose.position.y = yChargeBase;
			lWayPoints[lHomeWayPoint].target_pose.pose.orientation = chargeBase_quant;

			int lWayPointNumber = 0;
			int lNoOfWayPoints = 5;

			//Reverse out of the chargins station to the return position
			goal = lWayPoints[4];

			ROS_INFO("Backing out of Charging Station");

			//Each waypoint needs to be in the frame ID of the map
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();

			ROS_INFO("Sending goal");

			ac.sendGoal(goal);

			ac.waitForResult();

			if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Undocked Sucessfully");
				ROS_INFO("WayPoint PosX: %f", lWayPoints[lHomeWayPoint].target_pose.pose.position.x);
				ROS_INFO("Waypoint PosY: %f", lWayPoints[lHomeWayPoint].target_pose.pose.position.y);
				ROS_INFO("Waypoint Orientation: %f", lWayPoints[lHomeWayPoint].target_pose.pose.orientation);
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
				goal = lWayPoints[lWayPointNumber];

				ROS_INFO("Waypoint No. %i", lWayPointNumber);

				// Each waypoint needs to be in the frame ID of the map
				goal.target_pose.header.frame_id = "map";
				goal.target_pose.header.stamp = ros::Time::now();

				ROS_INFO("Sending goal");

				ROS_INFO("WayPoint PosX: %f", lWayPoints[lWayPointNumber].target_pose.pose.position.x);
				ROS_INFO("Waypoint PosY: %f", lWayPoints[lWayPointNumber].target_pose.pose.position.y);
				ROS_INFO("Waypoint Orientation: %f", lWayPoints[lWayPointNumber].target_pose.pose.orientation);

				ac.sendGoal(goal);

				ac.waitForResult();

				if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					ROS_INFO("Waypoint Reached Sucessfully");
				}
				else
				{
					ROS_INFO("Failed to Reach Waypoint");
				}

				// For now just pause 5 seconds at each waypoint
				if (1)
				{
					sleep(lWaypointSleepWait);
					ROS_INFO("Pausing at WayPoint...");
				}

				if (lBatteryCapacity < 80.0 || cmdString == "HOME")
				{
					if (lBatteryCapacity < 80.0)
					{
						ROS_WARN("Battery Too Low - Returning To Base");						
					}
					else
					{
						ROS_WARN("HOME Command Issued - Returning To Base");
					}

					lBatteryTooLow = true;

					// Each waypoint needs to be in the frame ID of the map
					goal = lWayPoints[lHomeWayPoint];
					goal.target_pose.header.frame_id = "map";
					goal.target_pose.header.stamp = ros::Time::now();
					

					ROS_INFO("Sending goal");

					// Issue Waypoint command to go home/return to base
					ROS_INFO("WayPoint PosX: %f", lWayPoints[lHomeWayPoint].target_pose.pose.position.x);
					ROS_INFO("Waypoint PosY: %f", lWayPoints[lHomeWayPoint].target_pose.pose.position.y);
					ROS_INFO("Waypoint Orientation: %f", lWayPoints[lHomeWayPoint].target_pose.pose.orientation);

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
									goal = lWayPoints[lHomeWayPoint];
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
										ROS_INFO("WayPoint PosX: %f", lWayPoints[lHomeWayPoint].target_pose.pose.position.x);
										ROS_INFO("Waypoint PosY: %f", lWayPoints[lHomeWayPoint].target_pose.pose.position.y);
										ROS_INFO("Waypoint Orientation: %f", lWayPoints[lHomeWayPoint].target_pose.pose.orientation);
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
	return 0;
}
