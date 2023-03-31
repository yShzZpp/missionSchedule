#ifndef __MISSION_SCHEDULE_ELEVATORCOSTFUNCTION_HH__
#define __MISSION_SCHEDULE_ELEVATORCOSTFUNCTION_HH__

#include "cti_spdlog.h"
#include "IElevatorPlanner.h"
#include "IStationCostFunction.h"
#include "IDeviceRuntimeUtility.h"
#include "common/ContainerInjector.h"

namespace cti
{
  namespace missionSchedule
  {
    class ElevatorCostFunction : public IStationCostFunction
    {
      public:
        ElevatorCostFunction() { IStationCostFunction::name_ = "ElevatorCostFunction"; }

        ~ElevatorCostFunction() {}

        bool prepare() override;

        double scoreStation(const StationModel& station) override;

        double scoreStation(const RobotUtility& robot, const StationModel& station) override;
      private:
        Position currentLocation_;
        std::string currentFloor_;
    };
  }
}

#endif