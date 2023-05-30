#ifndef __MISSION_SCHEDULE_IELEVATORPLANNER_HH__
#define __MISSION_SCHEDULE_IELEVATORPLANNER_HH__

#include "common/ScheduleUtils.h"
#include "cti_msgs/CloudBuildingInfo.h"

namespace cti
{
  namespace missionSchedule
  {
    class IElevatorPlanner
    {
      public:
        IElevatorPlanner() = default;
        virtual ~IElevatorPlanner() = default;

        virtual bool updateElevatorGraph() = 0;

        virtual bool isRobotLocationOccupyElevator(const std::string& floor, const Position& robotLocation) = 0;

        virtual Position getElevatorEnterPosition(const std::string& elevatorId, const std::string& floor) = 0;

        virtual Position getElevatorInsidePosition(const std::string& elevatorId, const std::string& floor) = 0;

        virtual ElevatorPlan calculateBestElevatorPlan(const Position& currentPosition,
                                                       const std::string& currentFloor,
                                                       const StationModel& targetStation,
                                                       bool usePathCost = false,
                                                       const std::vector<std::string>& tags = std::vector<std::string>(),
                                                       const std::string& preferElevatorId = std::string()) = 0;
    };
  }
}

#endif
