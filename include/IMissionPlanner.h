#ifndef __MISSION_SCHEDULE_IMISSIONPLANNER_HH__
#define __MISSION_SCHEDULE_IMISSIONPLANNER_HH__

#include "IStationCostFunction.h"
#include "common/ScheduleUtils.h"
#include "common/ContainerInjector.h"

namespace cti
{
  namespace missionSchedule
  {
    class IMissionPlanner
    {
      public:
        IMissionPlanner() = default;

        virtual ~IMissionPlanner() = default;

        virtual bool resetRecoveryFlag() = 0;

        virtual bool isSwitchFloorNeed() = 0;

        virtual bool allocateElevatorPlan() = 0;

        virtual bool findParkingLocation() = 0;

        virtual bool findChargingLocation() = 0;

        virtual bool pickPlatformMission() = 0;

        virtual bool isCurrentParkingLocationGood() = 0;

        virtual bool executePlatformMission() = 0;

        virtual bool executeCloudDirectCommand() = 0;

        virtual bool executeChargingMission() = 0;

        virtual bool executeParkingMission() = 0;

        virtual bool executeRobotStillBehavior() = 0;

        virtual bool executeRobotRecoverBehavior() = 0;

        virtual bool isChargingTobeContinue() = 0;

        virtual bool executeManualOrder() = 0;

        virtual bool abortAllPlatformMissions() = 0;

        virtual bool cancelAllPlatformMissions(std::string reason) = 0;

        virtual bool isRobotFreeOfSystemPowerManagement() = 0;

        virtual bool broadcastDesiredDestination() = 0;

        virtual bool publishAbortCommand(const std::string orderId, const bool densityCtrl) = 0;

        virtual std::string pickBetterChargingStation(const std::string& robotId, const std::vector<std::string> chargingStations) = 0;

        virtual std::string allocateChargingStationToRobot(const std::string& robotId) = 0;

        virtual bool addBanedStation(std::string& stationId) = 0;

        virtual bool cancelManualOrder() = 0;

        virtual bool isAppropriateToReboot() = 0;

        virtual bool robotAutonomicReboot() = 0;

        virtual nlohmann::json readRelocateFromLocalStorage() = 0;

        virtual bool publishRelocateCommand(const nlohmann::json& relocateJson) = 0;

        virtual bool bindLocalHive(bool up, const std::string& hiveId) = 0;

        virtual bool publishClearGoal(const bool clearGoal) = 0;

        virtual road_control::density_srvResponse callDensityServer(std::shared_ptr<cti::missionSchedule::RobotUtility> robot, Position destination) = 0;
    };
  }
}

#endif
