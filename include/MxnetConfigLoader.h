#include <ros/ros.h>
#include <nlohmann/json.hpp>
#include <memory.h>
#include <fstream>

class MxnetLoader
{
private:
public:
    MxnetLoader(/* args */);
    ~MxnetLoader();
};

MxnetLoader::MxnetLoader(/* args */)
{
}

MxnetLoader::~MxnetLoader()
{
}

class MxnetConfigLoader : public MxnetLoader
{
private:

    nlohmann::json mRobotConfigJSON;
    double mBatteryLowLevel = 0;
    double mMaxRobotChargeLevel = 0;
    
    static  std::string mConfigurationFile;    
    public:
    bool loadRobotConfiguration(std::string configurationFile = mConfigurationFile);
    int getBatteryLowLevel(){return mBatteryLowLevel;}
    int getMaxRobotChargeLevel(){return mMaxRobotChargeLevel;}
    MxnetConfigLoader(/* args */);
    ~MxnetConfigLoader();
};



MxnetConfigLoader::MxnetConfigLoader(/* args */)
{
}

MxnetConfigLoader::~MxnetConfigLoader()
{
}

class MxnetNavigationConfig : public MxnetLoader
{
private:
    static  std::string mWaypointsFile;
    int mNoOfWaypoints = 0;

    nlohmann::json mRobotNavigationJSON;


public:
    bool loadWayPoints(std::string waypointsFile = mWaypointsFile);
    int getNoOfWaypoints(){return mNoOfWaypoints;}

    MxnetNavigationConfig(/* args */);
    ~MxnetNavigationConfig();
};

MxnetNavigationConfig::MxnetNavigationConfig(/* args */)
{
}

MxnetNavigationConfig::~MxnetNavigationConfig()
{
}
