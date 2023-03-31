#ifndef __MISSION_SCHEDULE_IPATHPLANNER_HH__
#define __MISSION_SCHEDULE_IPATHPLANNER_HH__

#include "common/ScheduleUtils.h"

namespace cti
{
  namespace missionSchedule
  {
    class IPathPlanner
    {
      public:
        IPathPlanner() = default;
        virtual ~IPathPlanner() = default;

        virtual std::vector<Position> calculatePath(const Position& start, const Position& goal, const std::string& floor) = 0;

        virtual double calculatePathCost(const Position& start, const Position& goal, const std::string& floor) = 0;
    };
  }
}


#endif