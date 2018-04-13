#include <ros/ros.h>
#include <iostream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <kobuki_msgs/SensorState.h>
#include <mxnet_actionlib/AutoDockingAction.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int lChargeStatus = -1;
double lRobotChargeLevel = 0;
double lMaxRobotChargeLevel = 164;

void kobukiSensorsCoreCallback(const kobuki_msgs::SensorState::ConstPtr& msg)
 { 
  //ROS_INFO("Charger Status: [%i]", msg->charger);
  lChargeStatus = msg->charger;
  lRobotChargeLevel  = msg->battery;
 }

const double PI = (double)3.14159265358979;

double Deg2Rad (double Degrees)
{
	return (Degrees / 180) * PI;
}

double Rad2Deg (double Radians)
{	
	return (Radians / PI) * 180;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals"); 

//tell the action client that we want to spin a thread by default

  ros::NodeHandle nodeHandle;
	
  ros::Subscriber sub = nodeHandle.subscribe("/mobile_base/sensors/core", 10, kobukiSensorsCoreCallback);

    //ros::spin();
    ros::AsyncSpinner spinner(4);
    spinner.start();

  MoveBaseClient ac("move_base", true);
  actionlib::SimpleActionClient<mxnet_actionlib::AutoDockingAction> chargingStationClient("fibonacci", true);

  tf::TransformListener listener;

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
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
	double lDistanceToReverse = 0.35; //Measure in meters
	double xTargetPos = 0.0;
	double yTargetPos = 0.0;

	tf::StampedTransform transform;
	geometry_msgs::Quaternion  goal_quant;

	ROS_INFO("Getting Starting Location...");

	while (lWaitingForTransform == true)
	{
		try
    {
      listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
			xStartingPos = transform.getOrigin().x();
			yStartingPos = transform.getOrigin().y();
			startingRotation = tf::getYaw(transform.getRotation());	

			ROS_INFO("****** STARTING LOCATION ******");
      ROS_INFO("Starting PosX: %f", xStartingPos);
      ROS_INFO("Starting PosY: %f", yStartingPos);
			ROS_INFO("Starting Orientation: %f", startingRotation);
			ROS_INFO("*******************************\n");

			// Break out of this now we have starting position.
			lWaitingForTransform = false;
				
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
	}

	
	// Minus Pi (180 degrees) so that we know the component is behind the robot
	double lHorizontalComponent = lDistanceToReverse * cos(startingRotation-PI);
	double lVerticalComponent = lDistanceToReverse * sin(startingRotation-PI);
	
	xTargetPos = xStartingPos + lHorizontalComponent;
	yTargetPos = yStartingPos + lVerticalComponent;


	ROS_INFO("****** TARGET LOCATION ******");
	ROS_INFO("Target PosX: %f", xTargetPos);
	ROS_INFO("Target PosY: %f", yTargetPos);
	ROS_INFO("Starting Orientation: %f", startingRotation);
	ROS_INFO("*******************************");

	sleep(2);

	// Now set the return to base location. We will make this 0.5m back from the starting
	// starting locaton (idea being that it will give the robot a chance to doc sucessfully


	// Waypoint to store revers from base position
	double xChargeBase = xTargetPos;
	double yChargeBase = yTargetPos;
  double chargeBaseAngle = startingRotation;

	geometry_msgs::Quaternion  chargeBase_quant = tf::createQuaternionMsgFromYaw(chargeBaseAngle);


  lWayPoints[0].target_pose.pose.position.x = -0.336;
  lWayPoints[0].target_pose.pose.position.y = 1.483;

  goal_angle = 1.565;
  goal_quant = tf::createQuaternionMsgFromYaw(goal_angle);

  lWayPoints[0].target_pose.pose.orientation = goal_quant;


  lWayPoints[1].target_pose.pose.position.x = 2.005;
  lWayPoints[1].target_pose.pose.position.y = 0.105;

	goal_angle = -1.733;
  goal_quant = tf::createQuaternionMsgFromYaw(goal_angle);

  lWayPoints[1].target_pose.pose.orientation = goal_quant;

  // Kitchen - Oven Area
  lWayPoints[2].target_pose.pose.position.x = -0.469;
  lWayPoints[2].target_pose.pose.position.y = -0.071;

	//	goal_angle = -2.054;
	 goal_angle = -1.533;
  goal_quant = tf::createQuaternionMsgFromYaw(goal_angle);
  lWayPoints[2].target_pose.pose.orientation = goal_quant;

  // Kitchen - Oven Area
  lWayPoints[2].target_pose.pose.position.x = 0.270;
  lWayPoints[2].target_pose.pose.position.y = -6.271;

	//  goal_angle = -2.054;
   goal_angle = -1.619;
  goal_quant = tf::createQuaternionMsgFromYaw(goal_angle);
  lWayPoints[2].target_pose.pose.orientation = goal_quant;

  // Return to base position (before executing docking request)
  lWayPoints[3].target_pose.pose.position.x = xChargeBase;
  lWayPoints[3].target_pose.pose.position.y = yChargeBase;
  lWayPoints[3].target_pose.pose.orientation = chargeBase_quant;

  int lWayPointNumber = 0;
  int lNoOfWayPoints  = 4;

  //Revers out of the chargins station to the return position
    goal = lWayPoints[3];

    ROS_INFO("Backing out of Charging Station");

    //Each waypoint needs to be in the frame ID of the map
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    ROS_INFO("Sending goal");

    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
   {
      ROS_INFO("Undocked Sucessfully");
		  ROS_INFO("WayPoint PosX: %f", lWayPoints[3].target_pose.pose.position.x);
    	ROS_INFO("Waypoint PosY: %f", lWayPoints[3].target_pose.pose.position.y);
    	ROS_INFO("Waypoint Orientation: %f", lWayPoints[3].target_pose.pose.orientation);
    }
    else
    {
      ROS_INFO("Failed to Undock");
    }

	sleep(2);

  while(ros::ok())
	{
		double lBatteryCapacity = (lRobotChargeLevel/lMaxRobotChargeLevel)*100.0;

    try
    {
      	listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
	    	ROS_INFO("PosX: %f", transform.getOrigin().x());
    		ROS_INFO("PosY: %f", transform.getOrigin().y());
    		ROS_INFO("Orientation: %f", tf::getYaw(transform.getRotation()));
		}
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // If we're in the charging station reverse out
    if (lChargeStatus == 2)
    {
			
			
    }

		ROS_INFO("Battery Capacity: %f", lBatteryCapacity);
		
		//std::cout << "Battery Capacity: " << lBatteryCapacity << std::endl;


    //Itterate over the stored WayPoints 
    goal = lWayPoints[lWayPointNumber];

    ROS_INFO("Waypoint No. %i", lWayPointNumber);

    //Each waypoint needs to be in the frame ID of the map
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    ROS_INFO("Sending goal");

		ROS_INFO("WayPoint PosX: %f",  lWayPoints[lWayPointNumber].target_pose.pose.position.x);
		ROS_INFO("Waypoint PosY: %f",   lWayPoints[lWayPointNumber].target_pose.pose.position.y);
		ROS_INFO("Waypoint Orientation: %f",   lWayPoints[lWayPointNumber].target_pose.pose.orientation);

    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
      ROS_INFO("Waypoint Reached Sucessfully");
		}
    else
		{
      ROS_INFO("Failed to Reach Waypoint");
		}

	// For now just pause 5 seconds at each waypoint
		if (1 )
    {
      sleep(5.0);
      ROS_INFO("Pausing at WayPoint...");
    }

    if ( lWayPointNumber < (lNoOfWayPoints-1) )
		{
    	lWayPointNumber++;
		}
    else
		{
      lWayPointNumber = 0;
		}

   // Have we arrived back at the docking station. If so then proceed to dock
   ROS_INFO("Way Point Number %i:", lWayPointNumber );
   if(lWayPointNumber == 0 && (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
   {
      ROS_INFO("Returning To Charging Station...Docking In Progress");

      mxnet_actionlib::AutoDockingGoal chargingStationGoal;
      chargingStationClient.sendGoal(chargingStationGoal);
      bool finished_before_timeout = chargingStationClient.waitForResult(ros::Duration(480.0));

      if (finished_before_timeout)
      {
         actionlib::SimpleClientGoalState state = chargingStationClient.getState();
         ROS_INFO("Docking finished: %s",state.toString().c_str());
      }
      else
      {
        ROS_INFO("Action did not finish before the time out.");
      }

      sleep(3600); //sleep to charge up (TODO - sense charge status
    }
  
  }
  return 0;
}
