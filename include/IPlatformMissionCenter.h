#ifndef __MISSION_SCHEDULE_IPLATFORMMISSIONCENTER_HH__
#define __MISSION_SCHEDULE_IPLATFORMMISSIONCENTER_HH__

#include "common/ScheduleUtils.h"

namespace cti
{
  namespace missionSchedule
  {
    class IPlatformMissionCenter
    {
      public:
        IPlatformMissionCenter() = default;
        ~IPlatformMissionCenter() = default;

        virtual bool loadOrderLocalStorage() = 0;

        virtual bool allocateHiveOrderToRobot() = 0;

        virtual bool isPlatformMissionRunning() = 0;

        virtual bool isCloudDirectCommandEmpty() = 0;

        virtual nlohmann::json computeManualOrderCommand() = 0;

        virtual nlohmann::json computeOrderCommand() = 0;

        virtual nlohmann::json computeCloudDirectCommand() = 0;

        virtual bool updatePlatformMissionState(const nlohmann::json& commandResult) = 0;

        virtual bool reportOrderSelectionProgress(const std::string& orderId, const std::string& waypointId, const std::string& dockId = "") = 0;

        virtual bool reportOrderProgress(const std::string& orderId, const std::string& progressState, const std::string& reason = "", const bool& authenticTarget = false) = 0;

        virtual bool reportOrderAllocationResult(const std::string& orderAllocationResult) = 0;

        virtual bool reportElevatorAllocationResult(const std::string& orderId, const std::string& elevatorId, double cost) = 0;

        virtual bool cancelAllPlatformOrders(const std::string& reason) = 0;

        virtual bool cancelManualOrder() = 0;

        virtual bool allocateManualOrderToRobot() = 0;

        virtual bool isManualOrderEmpty() = 0;

        virtual bool isDeviceCharging() = 0;

        virtual bool uploadLocalScheduleOrder(const nlohmann::json& orderJson, std::string& responseBody, std::string& errorMessage) = 0;

        virtual bool isSterilizeOrderReachedStation() = 0;

        virtual bool cancelHiveSterilize() = 0;

        virtual bool executeSterilizeNextStep() = 0;

        virtual void showAllPlatformOrders() = 0;

        virtual void updatePlatformCommand(std::string oldCommandId, nlohmann::json command) = 0;
    };
  }
}


#endif
