#ifndef __MISSION_SCHEDULE_MISSIONSCHEDULENODE_HH__
#define __MISSION_SCHEDULE_MISSIONSCHEDULENODE_HH__

#include "cti_spdlog.h"
#include "IMissionPlanner.h"
#include "IElevatorPlanner.h"
#include "IDeviceRuntimeUtility.h"
#include "IPlatformMissionCenter.h"
#include "common/ContainerInjector.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace cti
{
  namespace missionSchedule
  {
    BT::NodeStatus forceFailure();

    BT::NodeStatus forceSuccess();

    BT::NodeStatus isDeviceStatusOk();

    BT::NodeStatus updateElevatorGraph();

    BT::NodeStatus isPlatformMissionRunning();

    BT::NodeStatus isDevicePowerBad();

    BT::NodeStatus isDeviceChargingToBasicPower();

    BT::NodeStatus isDevicePowerGood();

    BT::NodeStatus isDevicePowerNice();

    BT::NodeStatus isPlatformMissionEmpty();

    BT::NodeStatus schedulePlatformMission();

    BT::NodeStatus findChargingLocation();

    BT::NodeStatus findParkingLocation();

    BT::NodeStatus findParkingLocation();

    BT::NodeStatus isSwitchFloorNeed();

    BT::NodeStatus allocateElevatorPlan();

    BT::NodeStatus executePlatformMission();

    BT::NodeStatus executeChargingMission();

    BT::NodeStatus executeParkingMission();

    BT::NodeStatus isCurrentParkingLocationGood();

    BT::NodeStatus isRobotFreeOfSystemPowerManagement();

    void registerNodes(BT::BehaviorTreeFactory& factory);
  }
}

#endif
