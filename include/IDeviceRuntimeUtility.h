#ifndef __MISSION_SCHEDULE_IDEVICERUNTIMEUTILITY_HH__
#define __MISSION_SCHEDULE_IDEVICERUNTIMEUTILITY_HH__

#include "common/ScheduleUtils.h"

namespace cti
{
  namespace missionSchedule
  {
    class IDeviceRuntimeUtility
    {
      public:
        IDeviceRuntimeUtility() = default;
        virtual ~IDeviceRuntimeUtility() = default;

        virtual bool isStillMode() = 0;

        virtual bool isHiveLoaded() = 0;

        virtual bool isManualMode() = 0;

        virtual bool isDeviceIdle() = 0;

        virtual bool  isZigbeeModuleFault() = 0;

        virtual bool isDeviceStatusOk() = 0;

        virtual bool isDeviceStatusFault() = 0;

        virtual bool isDeviceSignalGood() = 0;

        virtual bool isDirectedCommandAllowedInStillMode() = 0;

        virtual bool isRobotShouldCancelOrdersWhenDisabled() = 0;

        virtual bool isRobotShouldCancelOrdersWhenFault() = 0;

        virtual bool isInfraryPaired() = 0;

        virtual std::string getHubId() = 0;

        virtual std::string getBuildingId() = 0;

        virtual std::string getCurrentFloor() = 0;

        virtual bool isHiveAttachedOnRobot() = 0;

        virtual std::string getRobotCloudId() = 0;

        virtual bool updateStationOccupyRelationship() = 0;

        virtual bool setRobotState(const RobotState& state) = 0;

        virtual bool isStationValid(const std::string& stationId) = 0;

        virtual bool isStationOccupied(const std::string& stationId) = 0;

        virtual std::string getStationDockId(const std::string& stationId) = 0;

        virtual void useInfraryToSendMsg(const std::string& msg) = 0;

        virtual std::string getStationOccupiedRobotId(const std::string& stationId) = 0;

        virtual bool isDevicePowerBad(std::string robotId = "") = 0;

        virtual bool isDeviceChargingToBasicPower(std::string robotId = "") = 0;

        virtual bool isDevicePowerNice(std::string robotId = "") = 0;

        virtual bool isDevicePowerGood(std::string robotId = "") = 0;

        virtual bool getHiveInfo(std::string hiveId, HiveUtility& hive) = 0;

        virtual std::string getHiveIdByHiveQr(const std::string& hiveQr) = 0;

        virtual bool isPositionNearNarrowArea(const Position& position) = 0;

        virtual bool isPositionInsideNarrowArea(const Position& position) = 0;

        virtual void setStillMode(bool stillMode, bool allowDirectCommand, int duration) = 0;

        virtual std::shared_ptr<RobotUtility> getRobotInfo(std::string robotId = "") = 0;

        virtual bool getWaypointInfo(std::string waypointId, StationModel& waypoint) = 0;

        virtual std::shared_ptr<StationModel> getStationInfo(std::string robotId = "") = 0;

        virtual bool getParkingStationDistribution(std::string floor, StationDistributionInfo& info) = 0;

        virtual bool getChargingStationDistribution(std::string floor, StationDistributionInfo& info) = 0;

        virtual void foreachWaypoint(std::function<bool(const StationModel& waypoint)> handler) = 0;

        virtual void foreachSystemRobot(std::function<bool(std::shared_ptr<RobotUtility>)> handler) = 0;

        virtual void foreachParkingStation(std::function<bool(std::shared_ptr<StationModel>)> handler) = 0;

        virtual void foreachChargingStation(std::function<bool(std::shared_ptr<StationModel>)> handler) = 0;

        virtual void foreachElevatorDodgeStation(std::function<bool(std::shared_ptr<StationModel>)> handler) = 0;
    };
  }
}

#endif
