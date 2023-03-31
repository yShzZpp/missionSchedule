#include "stationCostFunction/PowerCostFunction.h"

namespace cti
{
  namespace missionSchedule
  {
    bool PowerCostFunction::prepare()
    {
      return true;
    }

    double PowerCostFunction::scoreStation(const StationModel& station)
    {
      double cost = 0.0;
      return cost;
    }

    double PowerCostFunction::scoreStation(const RobotUtility& robot, const StationModel& station)
    {
      double cost = 0.0;
      cost += robot.chassisPower();
      // if (RobotState::CHARGING == robot.robotState())
      // {
      //   cost = std::max(0.0, std::min(cost - 10.0, 100.0));
      // }
      return cost;
    }
  }
}