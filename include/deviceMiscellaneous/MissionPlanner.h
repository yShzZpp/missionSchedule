#ifndef __MISSION_SCHEDULE_MISSIONPLANNER_HH__
#define __MISSION_SCHEDULE_MISSIONPLANNER_HH__

#include <memory>
#include <limits>
#include <sys/sysinfo.h>
#include <fstream>
#include <unistd.h>
#include "nlohmann/json.hpp"
#include "IPathPlanner.h"
#include "IMissionPlanner.h"
#include "IElevatorPlanner.h"
#include "IPlatformMissionCenter.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "common/Hungarian.h"
#include "common/ScheduleOrder.h"
#include "mission_schedule/coordinatesMsg.h"
// #include "cti_msgs/SystemReboot.h"

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/random_generator.hpp>

#include "mission_schedule/rebootTimeMsg.h"
#include "stationCostFunction/PathCostFunction.h"
#include "stationCostFunction/PowerCostFunction.h"
#include "stationCostFunction/ParkingCostFunction.h"
#include "stationCostFunction/ChargingCostFunction.h"
#include "stationCostFunction/ElevatorCostFunction.h"
#include "stationCostFunction/RobotConflictCostFunction.h"
#include "stationCostFunction/StationDistributionCostFunction.h"

namespace cti
{
  namespace missionSchedule
  {
    std::shared_ptr<IMissionPlanner> createMissionPlanner();
  }
}

#endif
