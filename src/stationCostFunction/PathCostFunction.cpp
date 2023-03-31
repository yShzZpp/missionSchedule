#include "stationCostFunction/PathCostFunction.h"

namespace cti
{
  namespace missionSchedule
  {
    bool PathCostFunction::prepare()
    {
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      currentFloor_ = deviceUtility->getRobotInfo()->currentFloor();
      currentLocation_ = deviceUtility->getRobotInfo()->location();
      currentBuilding_ = deviceUtility->getRobotInfo()->buildingName();
      return true;
    }

    double PathCostFunction::scoreStation(const RobotUtility& robot, const StationModel& station)
    {
      return scoreStation(station);
    }

    double PathCostFunction::scoreStation(const StationModel& station)
    {
      double movingCost = 0.0;
      ElevatorPlan elevatorPlan;
      auto pathPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IPathPlanner>();
      auto elevatorPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IElevatorPlanner>();
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      elevatorPlan = elevatorPlanner->calculateBestElevatorPlan(currentLocation_, currentFloor_, station);
      Position startLocation = currentLocation_;
      std::string startFloor = currentFloor_;

      for (auto elevatorStep = elevatorPlan.planSteps.begin(); elevatorStep != elevatorPlan.planSteps.end(); elevatorStep++)
      {
        if (startFloor == elevatorStep->elevatorFloor)
        {
          double pathCost = pathPlanner->calculatePathCost(startLocation, elevatorStep->elevatorLocation, startFloor);
          movingCost += pathCost;
        }

        startLocation = elevatorStep->elevatorLocation;
        startFloor = elevatorStep->elevatorFloor;
      }

      if (startFloor != station.floor())
      {
        return 1000;
      }
      double pathCost = pathPlanner->calculatePathCost(startLocation, station.coordinate(), startFloor);
      movingCost += pathCost;
      return movingCost;
    }
  }
}