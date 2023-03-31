#include "stationCostFunction/ChargingCostFunction.h"

namespace cti
{
  namespace missionSchedule
  {
    bool ChargingCostFunction::prepare()
    {
      return true;
    }

    double ChargingCostFunction::scoreStation(const StationModel& station)
    {
      double cost = 0.0;
      bool nearCharger = false;

      return nearCharger? cost : cost + 10;
    }

    double ChargingCostFunction::scoreStation(const RobotUtility& robot, const StationModel& station)
    {
      double cost = 0.0;
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      if (robot.robotState() == RobotState::CHARGING && !robot.currentStation().expired())
      {
        auto chargingStation =  robot.currentStation().lock();
        if (station.id() != chargingStation->id())
        {
          cost += 100;
        }
      }
      if (!station.robotOccupyThisStation().expired())
      {
        auto chargingRobot = station.robotOccupyThisStation().lock();
        if (robot.robotId() != chargingRobot->robotId())
        {
          cost += 25;
          // if (std::abs(robot.chassisPower() - chargingRobot->chassisPower()) < 9.0f)
          // {
          //   cost += 20;
          // }
        }
      }

      return cost;
    }
  }
}