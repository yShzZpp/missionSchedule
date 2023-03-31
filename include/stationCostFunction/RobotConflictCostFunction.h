#ifndef __MISSION_SCHEDULE_ROBOTCONFLICTCOSTFUNCTION_HH__
#define __MISSION_SCHEDULE_ROBOTCONFLICTCOSTFUNCTION_HH__

#include "common/ContainerInjector.h"
#include "IStationCostFunction.h"
#include "IDeviceRuntimeUtility.h"

namespace cti
{
  namespace missionSchedule
  {
    class RobotConflictCostFunction : public IStationCostFunction
    {
      public :
        RobotConflictCostFunction() { IStationCostFunction::name_ = "RobotConflictCostFunction"; }

        ~RobotConflictCostFunction() {}

        bool prepare() override;

        double scoreStation(const StationModel& station) override;

        double scoreStation(const RobotUtility& robot, const StationModel& station) override;
      private :
        std::string robotId_;
    };
  }
}








#endif
