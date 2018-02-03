#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <kobuki_msgs/SensorState.h>
#include <mxnet_actionlib/AutoDockingAction.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int lChargeStatus = -1;

void kobukiSensorsCoreCallback(const kobuki_msgs::SensorState::ConstPtr& msg)
 { 
  //ROS_INFO("Charger Status: [%i]", msg->charger);
  lChargeStatus = msg->charger;
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
//	}
//  goal.target_pose.pose.position.x = -1.0;
//  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter backward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  move_base_msgs::MoveBaseGoal lWayPoints[10];

  // Waypoint to store revers from base position
  lWayPoints[0].target_pose.pose.position.x = 1.965664;
  lWayPoints[0].target_pose.pose.position.y = 0.865989; 
 
  double goal_angle = 1.602710;
  geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(goal_angle);

  lWayPoints[0].target_pose.pose.orientation = goal_quat;


  lWayPoints[1].target_pose.pose.position.x = 0.335181713104;
  lWayPoints[1].target_pose.pose.position.y = 0.0227855145931;
  lWayPoints[1].target_pose.pose.orientation = goal_quant;

  // Kitchen - Oven Area
  lWayPoints[2].target_pose.pose.position.x = 2.15797305107;
  lWayPoints[2].target_pose.pose.position.y = -0.200772881508;
  lWayPoints[2].target_pose.pose.orientation = goal_quant;

 // Kitchen - Charger
  lWayPoints[3].target_pose.pose.position.x = 2.05471038818;
  lWayPoints[3].target_pose.pose.position.y = 0.556651353836;
  lWayPoints[3].target_pose.pose.orientation = goal_quant;

	// Return to base position (before executing docking request)
  lWayPoints[4].target_pose.pose.position.x = 1.965664;
  lWayPoints[4].target_pose.pose.position.y = 0.865989;
  lWayPoints[4].target_pose.pose.orientation = goal_quat;

  int lWayPointNumber = 0;
  int lNoOfWayPoints  = 5;

  while(ros::ok())
  {

		tf::StampedTransform transform;

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

    //Itterate over the stored WayPoints 
    goal = lWayPoints[lWayPointNumber];

    ROS_INFO("Waypoint No. %i", lWayPointNumber);

    //Each waypoint needs to be in the frame ID of the map
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    ROS_INFO("Sending goal");

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

		if ( lWayPointNumber == 0 )
    {
      sleep(5.0);
      ROS_INFO("Pausing after leaving Docking Station");
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

      sleep(20);//3600); //sleep to charge up (TODO - sense charge status
    }
  
  }
  return 0;
}
