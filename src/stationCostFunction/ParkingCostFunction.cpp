#include "stationCostFunction/ParkingCostFunction.h"

namespace cti
{
  namespace missionSchedule
  {
    bool ParkingCostFunction::prepare()
    {
      return true;
    }

    double ParkingCostFunction::scoreStation(const RobotUtility& robot, const StationModel& station)
    {
      return scoreStation(station);
    }

    double ParkingCostFunction::scoreStation(const StationModel& station)
    {
      double cost = 0.0;
      double minOtherRobotDistance = 10.0;
      bool nearCharger = false;

      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      deviceUtility->foreachChargingStation([scoreStation = station, &nearCharger](std::shared_ptr<StationModel> chargingStation)
      {
        if (scoreStation.floor() != chargingStation->floor())
        {
          return false;
        }

        nearCharger = true;
        return true;
      });

      if (!nearCharger)
      {
        cost += 10.0;
      }

      deviceUtility->foreachParkingStation([scoreStation = station, &minOtherRobotDistance](std::shared_ptr<StationModel> parkingStation)
      {
        if (scoreStation.floor() != parkingStation->floor() || parkingStation->robotOccupyThisStation().expired())
        {
          return false;
        }
        double distance = scoreStation.coordinate().distanceOf(parkingStation->coordinate());
        minOtherRobotDistance = distance < minOtherRobotDistance? distance : minOtherRobotDistance;
        return false;
      });
      cost += minOtherRobotDistance;
      return cost;
    }
  }
}