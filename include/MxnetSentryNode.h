#include <ros/ros.h>
#include <iostream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nlohmann/json.hpp>
#include <memory.h>
#include <fstream>
#include <mxnet_actionlib/AutoDockingAction.h>
#include <string>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MxnetSentryNode
{
private:
    const double PI = (double)3.14159265358979;

    nlohmann::json mRobotNavigationJSON;
    nlohmann::json mRobotConfigJSON;

    static const std::string mWaypointsFile;
    static const std::string mConfigurationFile;

    int mNoOfWaypoints = 0;

    move_base_msgs::MoveBaseGoal mHomeGoal;

    double lDistanceToReverse = 1.20; //Measure in meters
    double xTargetPos = 0.0;
	double yTargetPos = 0.0;
    float mWaypointSleepWait = 5.0;

    tf::TransformListener listener;
    tf::StampedTransform transform;

public:
    MxnetSentryNode(/* args */);
    ~MxnetSentryNode();

    bool buildNextGoal(move_base_msgs::MoveBaseGoal& goal, int &WaypointID, bool commandStringDirective = false);
    bool loadRobotConfiguration(std::string configurationFile = mConfigurationFile);
    bool loadWayPoints(std::string waypointsFile = mWaypointsFile);
    bool setInitialPose(ros::Publisher initialPosePublisher);
    move_base_msgs::MoveBaseGoal  calculateHomeGoal();

    move_base_msgs::MoveBaseGoal getHomeGoal(){return mHomeGoal;}
    int getNoOfWaypoints(){return mNoOfWaypoints;}
    static auto waitForTransform (double& xStartingPos, double& yStartingPos, double& startingRotation );
    bool dockRobot(MoveBaseClient* ac, int* chargingStatus);
    int extractWaypointID(std::string cmdString);
    bool undockRobot(MoveBaseClient* ac, move_base_msgs::MoveBaseGoal& HomeStationGoal);
    bool returnToBase(MoveBaseClient* ac, move_base_msgs::MoveBaseGoal& HomeStationGoal);
};

MxnetSentryNode::MxnetSentryNode(/* args */)
{
}

MxnetSentryNode::~MxnetSentryNode()
{
}

const std::string MxnetSentryNode::mWaypointsFile = "/home/mark/mxnetRobotics/kobukirobot/waypoints/robot_waypoints.json";
const std::string MxnetSentryNode::mConfigurationFile = "/home/mark/mxnetRobotics/kobukirobot/config/robot_config.json";


