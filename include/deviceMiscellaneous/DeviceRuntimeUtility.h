#ifndef __MISSION_SCHEDULE_DEVICERUNTIMEUTILITY_HH__
#define __MISSION_SCHEDULE_DEVICERUNTIMEUTILITY_HH__

#include <memory>
#include <shared_mutex>
#include <unordered_map>
#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>

#include "ros/ros.h"
#include "cti_spdlog.h"
#include "IDeviceRuntimeUtility.h"
#include "IElevatorPlanner.h"
#include "common/ContainerInjector.h"

#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "mission_schedule/batteryMsg.h"
#include "cti_msgs/ZoneArray.h"
#include "cti_msgs/HealthState.h"
#include "cti_msgs/CloudRobotInfo.h"
#include "cti_msgs/BaseSensors.h"
#include <cti_msgs/BaseControl.h>
#include "cti_msgs/RobotNotifyState.h"
#include "cti_msgs/BaseBatteryState.h"
#include "cti_msgs/CloudBuildingInfo.h"
#include "cti_msgs/BuildingRobotState.h"
#include "cti_msgs/BackgardenDownloadInfo.h"
#include "cti_msgs/CtiCommonService.h"
#include "schedule_message.pb.h"
#include "IPathPlanner.h"
#include "IMissionPlanner.h"
#include "IPlatformMissionCenter.h"

namespace cti
{
  namespace missionSchedule
  {
    std::shared_ptr<IDeviceRuntimeUtility> createRuntimeStatusMonitor();
  }
}


#endif
