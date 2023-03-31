#include "stationCostFunction/ElevatorCostFunction.h"

namespace cti
{
  namespace missionSchedule
  {
    bool ElevatorCostFunction::prepare()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      currentFloor_ = deviceUtility->getRobotInfo()->currentFloor();
      currentLocation_ = deviceUtility->getRobotInfo()->location();
      return true;
    }

    double ElevatorCostFunction::scoreStation(const StationModel& station)
    {
      ElevatorPlan elevatorPlan;
      if (station.floor() == currentFloor_)
      {
        return 0;
      }
      auto elevatorPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IElevatorPlanner>();
      elevatorPlan = elevatorPlanner->calculateBestElevatorPlan(currentLocation_, currentFloor_, station);
      return elevatorPlan.cost;
    }

    double ElevatorCostFunction::scoreStation(const RobotUtility& robot, const StationModel& station)
    {
      ElevatorPlan elevatorPlan;
      auto robotLocation = robot.location();
      auto robotFloor = robot.currentFloor();
      if (station.floor() == robotFloor)
      {
        return 0;
      }
      auto elevatorPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IElevatorPlanner>();
      elevatorPlan = elevatorPlanner->calculateBestElevatorPlan(robotLocation, robotFloor, station);
      return elevatorPlan.cost;
    }
  }
}