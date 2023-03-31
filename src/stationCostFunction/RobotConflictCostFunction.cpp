#include "stationCostFunction/RobotConflictCostFunction.h"

namespace cti
{
  namespace missionSchedule
  {
    bool RobotConflictCostFunction::prepare()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      robotId_ = deviceUtility->getRobotInfo()->robotId();
      return true;
    }

    double RobotConflictCostFunction::scoreStation(const RobotUtility& robot, const StationModel& station)
    {
      double cost = 0.0;
      auto deviceUtility = common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      auto robotId = robot.robotId();
      deviceUtility->foreachSystemRobot([scoreStation = station, selfId = robotId, &cost](std::shared_ptr<RobotUtility> robot)
      {
        if (robot->robotId() == selfId)
        {
          return false;
        }
        if (scoreStation.floor() == robot->currentFloor() && scoreStation.isInsideOccupyArea(robot->localDestination()))
        {
          cost += 1000;
        }
        if (scoreStation.floor() == robot->currentFloor() && scoreStation.isInsideOccupyArea(robot->platformDestination()))
        {
          cost += 1000;
        }
        return false;
      });

      return cost;
    }

    double RobotConflictCostFunction::scoreStation(const StationModel& station)
    {
      double cost = 0.0;
      auto deviceUtility = common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      deviceUtility->foreachSystemRobot([scoreStation = station, selfId = robotId_, &cost](std::shared_ptr<RobotUtility> robot)
      {
        if (robot->robotId() == selfId)
        {
          return false;
        }
        if (scoreStation.floor() == robot->currentFloor() && scoreStation.isInsideOccupyArea(robot->localDestination()))
        {
          cost += 1000;
        }
        if (scoreStation.floor() == robot->currentFloor() && scoreStation.isInsideOccupyArea(robot->platformDestination()))
        {
          cost += 1000;
        }
        return false;
      });

      return cost;
    }
  }
}