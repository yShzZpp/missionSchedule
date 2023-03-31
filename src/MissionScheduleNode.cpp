#include "MissionScheduleNode.h"

namespace cti
{
  namespace missionSchedule
  {
    BT::NodeStatus forceFailure()
    {
      return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus forceSuccess()
    {
      return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus isStillMode()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return deviceUtility->isStillMode() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isZigbeeModuleFault()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return deviceUtility->isZigbeeModuleFault() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isRobotShouldCancelOrdersWhenDisabled()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return deviceUtility->isRobotShouldCancelOrdersWhenDisabled() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isRobotShouldCancelOrdersWhenFault()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return deviceUtility->isRobotShouldCancelOrdersWhenFault() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isManualMode()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return deviceUtility->isManualMode() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isDeviceStatusOk()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return deviceUtility->isDeviceStatusOk() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isDeviceStatusFault()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return deviceUtility->isDeviceStatusFault() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus updateStationOccupyRelationship()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return deviceUtility->updateStationOccupyRelationship() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus updateElevatorGraph()
    {
      auto elevatorPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IElevatorPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return elevatorPlanner->updateElevatorGraph() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isCloudDirectCommandEmpty()
    {
      auto platformMissionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return platformMissionCenter->isCloudDirectCommandEmpty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus loadOrderLocalStorage()
    {
      auto platformMissionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return platformMissionCenter->loadOrderLocalStorage() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isPlatformMissionRunning()
    {
      auto platformMissionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return platformMissionCenter->isPlatformMissionRunning() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isManualOrderEmpty()
    {
      auto platformMissionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return platformMissionCenter->isManualOrderEmpty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isDeviceCharging()
    {
      auto platformMissionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return platformMissionCenter->isDeviceCharging() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus allocateManualOrderToRobot()
    {
      auto platformMissionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return platformMissionCenter->allocateManualOrderToRobot() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isDirectedCommandAllowedInStillMode()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return deviceUtility->isDirectedCommandAllowedInStillMode() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus executeRobotStillBehavior()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->executeRobotStillBehavior() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isHiveLoaded()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return deviceUtility->isHiveLoaded() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isDevicePowerBad()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return deviceUtility->isDevicePowerBad() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isDeviceChargingToBasicPower()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return deviceUtility->isDeviceChargingToBasicPower() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isDevicePowerGood()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return deviceUtility->isDevicePowerGood() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isDevicePowerNice()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return deviceUtility->isDevicePowerNice() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isCurrentParkingLocationGood()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->isCurrentParkingLocationGood() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus allocateHiveOrderToRobot()
    {
      auto platformMissionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return platformMissionCenter->allocateHiveOrderToRobot() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isChargingTobeContinue()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->isChargingTobeContinue() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus schedulePlatformMission()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->pickPlatformMission() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus findChargingLocation()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->findChargingLocation() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus findParkingLocation()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->findParkingLocation() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isSwitchFloorNeed()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->isSwitchFloorNeed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus allocateElevatorPlan()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->allocateElevatorPlan() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isRobotFreeOfSystemPowerManagement()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->isRobotFreeOfSystemPowerManagement() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus resetRecoveryFlag()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->resetRecoveryFlag() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus executeRobotRecoverBehavior()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->executeRobotRecoverBehavior() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus executeCloudDirectCommand()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->executeCloudDirectCommand() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus executePlatformMission()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->executePlatformMission() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus executeChargingMission()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->executeChargingMission() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus executeParkingMission()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->executeParkingMission() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus cancelAllPlatformMissions(BT::TreeNode &self)
    {
      auto platformMissionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      std::string reason;
      auto msg = self.getInput<std::string>("reason");
      if (msg)
      {
        reason = msg.value();
      }
      return platformMissionCenter->cancelAllPlatformOrders(reason) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus broadcastDesiredDestination()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->broadcastDesiredDestination() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus abortAllPlatformMissions()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->abortAllPlatformMissions() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus executeManualOrder()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->executeManualOrder() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus cancelManualOrder()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->cancelManualOrder() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isAppropriateToReboot()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->isAppropriateToReboot() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus robotAutonomicReboot()
    {
      auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return missionPlanner->robotAutonomicReboot() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus isSterilizeOrderReachedStation()
    {
      auto platformMissionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return platformMissionCenter->isSterilizeOrderReachedStation() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus cancelHiveSterilize()
    {
      auto platformMissionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>();
      SPDLOG_INFO("MissionSchedule tree hit {}", __func__);
      return platformMissionCenter->cancelHiveSterilize() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }


    void registerNodes(BT::BehaviorTreeFactory& factory)
    {
      factory.registerSimpleCondition("isPlatformMissionRunning", std::bind(isPlatformMissionRunning));
      factory.registerSimpleCondition("loadOrderLocalStorage", std::bind(loadOrderLocalStorage));
      factory.registerSimpleCondition("isHiveLoaded", std::bind(isHiveLoaded));
      factory.registerSimpleCondition("isStillMode", std::bind(isStillMode));
      factory.registerSimpleCondition("isZigbeeModuleFault", std::bind(isZigbeeModuleFault));
      factory.registerSimpleCondition("isManualMode", std::bind(isManualMode));
      factory.registerSimpleCondition("isCloudDirectCommandEmpty", std::bind(isCloudDirectCommandEmpty));
      factory.registerSimpleCondition("isDeviceStatusOk", std::bind(isDeviceStatusOk));
      factory.registerSimpleCondition("isDeviceStatusFault", std::bind(isDeviceStatusFault));
      factory.registerSimpleCondition("isSwitchFloorNeed", std::bind(isSwitchFloorNeed));
      factory.registerSimpleCondition("isDevicePowerBad", std::bind(isDevicePowerBad));
      factory.registerSimpleCondition("isDeviceChargingToBasicPower", std::bind(isDeviceChargingToBasicPower));
      factory.registerSimpleCondition("isDevicePowerGood", std::bind(isDevicePowerGood));
      factory.registerSimpleCondition("isDevicePowerNice", std::bind(isDevicePowerNice));
      factory.registerSimpleCondition("isRobotShouldCancelOrdersWhenDisabled", std::bind(isRobotShouldCancelOrdersWhenDisabled));
      factory.registerSimpleCondition("isRobotShouldCancelOrdersWhenFault", std::bind(isRobotShouldCancelOrdersWhenFault));
      factory.registerSimpleCondition("isChargingTobeContinue", std::bind(isChargingTobeContinue));
      factory.registerSimpleCondition("isCurrentParkingLocationGood", std::bind(isCurrentParkingLocationGood));
      factory.registerSimpleCondition("isRobotFreeOfSystemPowerManagement", std::bind(isRobotFreeOfSystemPowerManagement));
      factory.registerSimpleCondition("isDirectedCommandAllowedInStillMode", std::bind(isDirectedCommandAllowedInStillMode));
      factory.registerSimpleCondition("isManualOrderEmpty", std::bind(isManualOrderEmpty));
      factory.registerSimpleCondition("isDeviceCharging", std::bind(isDeviceCharging));
      factory.registerSimpleCondition("isAppropriateToReboot", std::bind(isAppropriateToReboot));
      factory.registerSimpleCondition("isSterilizeOrderReachStation", std::bind(isSterilizeOrderReachedStation));

      factory.registerSimpleAction("updateStationOccupyRelationship", std::bind(updateStationOccupyRelationship));
      factory.registerSimpleAction("executeRobotStillBehavior", std::bind(executeRobotStillBehavior));
      factory.registerSimpleAction("allocateHiveOrderToRobot", std::bind(allocateHiveOrderToRobot));
      factory.registerSimpleAction("forceFailure", std::bind(forceFailure));
      factory.registerSimpleAction("forceSuccess", std::bind(forceSuccess));
      BT::PortsList cancelAllPlatformMissionsReasonPort = { BT::InputPort<std::string>("reason") };
      factory.registerSimpleAction("cancelAllPlatformMissions", cancelAllPlatformMissions, cancelAllPlatformMissionsReasonPort);
      factory.registerSimpleAction("broadcastDesiredDestination", std::bind(broadcastDesiredDestination));
      factory.registerSimpleAction("updateElevatorGraph", std::bind(updateElevatorGraph));
      factory.registerSimpleAction("findParkingLocation", std::bind(findParkingLocation));
      factory.registerSimpleAction("findChargingLocation", std::bind(findChargingLocation));
      factory.registerSimpleAction("allocateElevatorPlan", std::bind(allocateElevatorPlan));
      factory.registerSimpleAction("executeParkingMission", std::bind(executeParkingMission));
      factory.registerSimpleAction("executePlatformMission", std::bind(executePlatformMission));
      factory.registerSimpleAction("executeChargingMission", std::bind(executeChargingMission));
      factory.registerSimpleAction("schedulePlatformMission", std::bind(schedulePlatformMission));
      factory.registerSimpleAction("resetRecoveryFlag", std::bind(resetRecoveryFlag));
      factory.registerSimpleAction("executeRobotRecoverBehavior", std::bind(executeRobotRecoverBehavior));
      factory.registerSimpleAction("executeCloudDirectCommand", std::bind(executeCloudDirectCommand));
      factory.registerSimpleAction("executeManualOrder", std::bind(executeManualOrder));
      factory.registerSimpleAction("abortAllPlatformMissions", std::bind(abortAllPlatformMissions));
      factory.registerSimpleAction("allocateManualOrderToRobot", std::bind(allocateManualOrderToRobot));
      factory.registerSimpleAction("cancelManualOrder", std::bind(cancelManualOrder));
      factory.registerSimpleAction("robotAutonomicReboot", std::bind(robotAutonomicReboot));
      factory.registerSimpleAction("cancelHiveSterilize", std::bind(cancelHiveSterilize));

    }
  }
}
