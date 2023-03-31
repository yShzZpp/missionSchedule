#ifndef __MISSION_SCHEDULE_PATHCOSTFUNCTION_HH__
#define __MISSION_SCHEDULE_PATHCOSTFUNCTION_HH__

#include <png.h>
#include <queue>
#include "cti_spdlog.h"
#include "IPathPlanner.h"
#include "yaml-cpp/yaml.h"
#include "IElevatorPlanner.h"
#include "IStationCostFunction.h"
#include "IDeviceRuntimeUtility.h"
#include "common/ContainerInjector.h"

namespace cti
{
  namespace missionSchedule
  {
    class PathCostFunction : public IStationCostFunction
    {
      public:
        PathCostFunction() { IStationCostFunction::name_ = "PathCostFunction"; }

        ~PathCostFunction() {}

        bool prepare() override;

        double scoreStation(const StationModel& station) override;

        double scoreStation(const RobotUtility& robot, const StationModel& station) override;
      private:
        Position currentLocation_;
        std::string currentFloor_;
        std::string currentBuilding_;
    };
  }
}

#endif