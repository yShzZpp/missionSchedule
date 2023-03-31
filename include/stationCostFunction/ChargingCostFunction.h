#ifndef __MISSION_SCHEDULE_CHARGINGCOSTFUNCTION_HH__
#define __MISSION_SCHEDULE_CHARGINGCOSTFUNCTION_HH__

#include "IStationCostFunction.h"
#include "IDeviceRuntimeUtility.h"
#include "common/ContainerInjector.h"

namespace cti
{
  namespace missionSchedule
  {
    class ChargingCostFunction : public IStationCostFunction
    {
      public:
        ChargingCostFunction() { IStationCostFunction::name_ = "ChargingCostFunction"; }

        ~ChargingCostFunction() {}

        bool prepare() override;

        double scoreStation(const StationModel& station) override;

        double scoreStation(const RobotUtility& robot, const StationModel& station) override;
    };
  }
}

#endif