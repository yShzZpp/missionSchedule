#include "stationCostFunction/StationDistributionCostFunction.h"

namespace cti
{
  namespace missionSchedule
  {
    bool StationDistributionCostFunction::prepare()
    {
      return true;
    }

    double StationDistributionCostFunction::scoreStation(const StationModel& station)
    {
      double cost = 0.0;
      StationDistributionInfo stationDistribution;
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      if ("DOCK" == station.type() && deviceUtility->getChargingStationDistribution(station.floor(), stationDistribution))
      {
        auto projectionPosition = calculatePointProjectionPositionToLine(stationDistribution.line.first, stationDistribution.line.second, station.coordinate());
        // SPDLOG_INFO("MissionPlanner evaluate charging station projection position {}", projectionPosition.toString());
        // SPDLOG_INFO("MissionPlanner evaluate charging station line projection position {}", stationDistribution.lineAnchor.toString());
        cost += projectionPosition.distanceOf(stationDistribution.lineAnchor);
      }
      else if (deviceUtility->getParkingStationDistribution(station.floor(), stationDistribution))
      {
        auto projectionPosition = calculatePointProjectionPositionToLine(stationDistribution.line.first, stationDistribution.line.second, station.coordinate());
        // SPDLOG_INFO("MissionPlanner evaluate parking station projection position {}", projectionPosition.toString());
        // SPDLOG_INFO("MissionPlanner evaluate parking station line projection position {}", stationDistribution.lineAnchor.toString());
        cost += projectionPosition.distanceOf(stationDistribution.lineAnchor);
      }
      return cost;
    }

    double StationDistributionCostFunction::scoreStation(const RobotUtility& robot, const StationModel& station)
    {
      return scoreStation(station);
    }
  }
}