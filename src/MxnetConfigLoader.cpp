#include "MxnetConfigLoader.h"
#include <iostream>
using json = nlohmann::json;

//std::string lHomeDir = getenv("HOME") ;
std::string lHomeDir = "/home/mark";
std::string MxnetNavigationConfig::mWaypointsFile = "/home/mark/mxnetRobotics/kobukirobot/waypoints/robot_waypoints.json";
std::string MxnetConfigLoader::mConfigurationFile = "/home/mark/mxnetRobotics/kobukirobot/config/robot_config.json";

// Load Configuration from JSON file
bool MxnetConfigLoader::loadRobotConfiguration(std::string configurationFile)
{
	ROS_INFO("Loading Robot Configuration");
    ROS_INFO("LConfiguration File: %s", configurationFile);
 
	std::ifstream jsonWayPointFile(configurationFile);
	
	try
	{
       	mRobotConfigJSON = nlohmann::json::parse(jsonWayPointFile);

		ROS_INFO("Configuration Parsed.");
        std::cout << mRobotConfigJSON << std::endl;
	}
     catch (json::exception& e)
     {
        ROS_INFO("Configuration Parse Failed.");
         // output exception information
         std::cout << "message: " << e.what() << '\n'
                   << "exception id: " << e.id << std::endl;

        return false;
	 }

    mMaxRobotChargeLevel = mRobotConfigJSON["config"]["maxRobotChargeLevel"];
    mBatteryLowLevel = mRobotConfigJSON["config"]["batteryLowLevel"];

     return true;
}

bool MxnetNavigationConfig::loadWayPoints(std::string waypointsFile)
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
