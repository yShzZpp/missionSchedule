#ifndef __MISSION_SCHEDULE_ISTATIONCOSTFUNCTION_HH__
#define __MISSION_SCHEDULE_ISTATIONCOSTFUNCTION_HH__

#include "common/ScheduleUtils.h"

namespace cti
{
  namespace missionSchedule
  {
    class IStationCostFunction {
    public:

      /**
       *
       * General updating of context values if required.
       * Subclasses may overwrite. Return false in case there is any error.
       */
      virtual bool prepare() = 0;

      /**
       * return a score for Station
       */
      virtual double scoreStation(const StationModel& station) = 0;

      virtual double scoreStation(const RobotUtility& robot, const StationModel& station) = 0;

      double getScale() {
        return scale_;
      }

      std::string getName() {
        return name_;
      }

      void setScale(double scale) {
        scale_ = scale;
      }

      void setName(const std::string& name)
      {
        name_ = name;
      }

      virtual ~IStationCostFunction() {}

      IStationCostFunction(double scale = 1.0): scale_(scale) {}

    protected:
      double scale_;
      std::string name_;
    };
  }
}


#endif