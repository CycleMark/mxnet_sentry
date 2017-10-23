#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <kobuki_msgs/SensorState.h>
#include <mxnet_actionlib/AutoDockingAction.h>


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


  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  move_base_msgs::MoveBaseGoal lWayPoints[10];




  // Kitchen - Shower room Area
  lWayPoints[0].target_pose.pose.position.x = 0.887773931026;
  lWayPoints[0].target_pose.pose.position.y = 2.80906915665;
  lWayPoints[0].target_pose.pose.orientation.w = 1.0;

  // Kitchen - Oven Area
  lWayPoints[1].target_pose.pose.position.x = 1.75210952759;
  lWayPoints[1].target_pose.pose.position.y = -0.0273013114929;
  lWayPoints[1].target_pose.pose.orientation.w = 1.0;

 // Kitchen - Charger
  lWayPoints[2].target_pose.pose.position.x = -1.25932836533;
  lWayPoints[2].target_pose.pose.position.y = 1.28973519802;
  lWayPoints[2].target_pose.pose.orientation.w = 1.0;

  // Kitchen - Shower room Area
  lWayPoints[0].target_pose.pose.position.x = 0.335181713104;
  lWayPoints[0].target_pose.pose.position.y = 0.0227855145931;
  lWayPoints[0].target_pose.pose.orientation.w = 1.0;

  // Kitchen - Oven Area
  lWayPoints[1].target_pose.pose.position.x = 2.15797305107;
  lWayPoints[1].target_pose.pose.position.y = -0.200772881508;
  lWayPoints[1].target_pose.pose.orientation.w = 1.0;

 // Kitchen - Charger
  lWayPoints[2].target_pose.pose.position.x = 2.05471038818;
  lWayPoints[2].target_pose.pose.position.y = 0.556651353836;
  lWayPoints[2].target_pose.pose.orientation.w = 1.0;


  lWayPoints[2].target_pose.pose.position.x = 2.00464200974;
  lWayPoints[2].target_pose.pose.position.y = 1.00800693035;
  lWayPoints[2].target_pose.pose.orientation.w = 1.0;




/*
  lWayPoints[0].target_pose.pose.position.x = 0.0714530944824;
  lWayPoints[0].target_pose.pose.position.y = 0.0327291488647;
  lWayPoints[0].target_pose.pose.orientation.w = 1.0;

  // Kitchen - Oven Area
  lWayPoints[1].target_pose.pose.position.x = 2.39645719528;
  lWayPoints[1].target_pose.pose.position.y = -0.19243490696;
  lWayPoints[1].target_pose.pose.orientation.w = 1.0;

 // Kitchen - Charger
  lWayPoints[2].target_pose.pose.position.x = 1.7021894455;
  lWayPoints[2].target_pose.pose.position.y = 0.918084144592;
  lWayPoints[2].target_pose.pose.orientation.w = 1.0;
*/
  int lWayPointNumber = 0;
  int lNoOfWayPoints  = 3;

  while(ros::ok())
  {

    // If we're in the charging station reverse out
    if (lChargeStatus == 2)
    {


    }

     //Move to the Oven
    goal = lWayPoints[lWayPointNumber];
    ROS_INFO("Waypoint No. %i", lWayPointNumber);
    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Waypoint Reached Sucessfully");
    else
      ROS_INFO("Failed to Reach Waypoint");

    if (lWayPointNumber < (lNoOfWayPoints-1))
      lWayPointNumber++;
    else
      lWayPointNumber = 0;

   // Have we arrived back at the docking station. If so then proceed to dock
   ROS_INFO("Way Point Number %i:", lWayPointNumber );
   if(lWayPointNumber == 0 && (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
   {
      ROS_INFO("Returning To Charging Station...Docking In Progress");

      mxnet_actionlib::AutoDockingGoal chargingStationGoal;
      chargingStationClient.sendGoal(chargingStationGoal);
      bool finished_before_timeout = chargingStationClient.waitForResult(ros::Duration(120.0));

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
