#ifndef __MISSION_SCHEDULE_POWERCOSTFUNCTION_HH__
#define __MISSION_SCHEDULE_POWERCOSTFUNCTION_HH__

#include "IStationCostFunction.h"
#include "IDeviceRuntimeUtility.h"
#include "common/ContainerInjector.h"

namespace cti
{
  namespace missionSchedule
  {
    class PowerCostFunction : public IStationCostFunction
    {
      public:
        PowerCostFunction() { IStationCostFunction::name_ = "PowerCostFunction"; }

        ~PowerCostFunction() {}

        bool prepare() override;

        double scoreStation(const StationModel& station) override;

        double scoreStation(const RobotUtility& robot, const StationModel& station) override;
    };
  }
}

#endif