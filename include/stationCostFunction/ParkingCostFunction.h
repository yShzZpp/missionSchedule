#ifndef __MISSION_SCHEDULE_PARKINGCOSTFUNCTION_HH__
#define __MISSION_SCHEDULE_PARKINGCOSTFUNCTION_HH__

#include "IStationCostFunction.h"
#include "IDeviceRuntimeUtility.h"
#include "common/ContainerInjector.h"

namespace cti
{
  namespace missionSchedule
  {
    class ParkingCostFunction : public IStationCostFunction
    {
      public:
        ParkingCostFunction() { IStationCostFunction::name_ = "ParkingCostFunction"; }

        ~ParkingCostFunction() {}

        bool prepare() override;

        double scoreStation(const StationModel& station) override;

        double scoreStation(const RobotUtility& robot, const StationModel& station) override;
    };
  }
}

#endif