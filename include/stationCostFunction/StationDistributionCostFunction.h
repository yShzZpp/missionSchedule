#ifndef __MISSION_SCHEDULE_STATIONDISTRIBUTIONCOSTFUNCTION_HH__
#define __MISSION_SCHEDULE_STATIONDISTRIBUTIONCOSTFUNCTION_HH__

#include "cti_spdlog.h"
#include "IStationCostFunction.h"
#include "common/ContainerInjector.h"
#include "IDeviceRuntimeUtility.h"

namespace cti
{
  namespace missionSchedule
  {
    class StationDistributionCostFunction : public IStationCostFunction
    {
      public:
        StationDistributionCostFunction() { IStationCostFunction::name_ = "StationDistributionCostFunction"; }

        ~StationDistributionCostFunction() {}

        bool prepare() override;

        double scoreStation(const StationModel& station) override;

        double scoreStation(const RobotUtility& robot, const StationModel& station) override;
    };
  }
}

#endif