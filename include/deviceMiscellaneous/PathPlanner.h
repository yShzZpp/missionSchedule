#ifndef __MISSION_SCHEDULE_PATHPLANNER_HH__
#define __MISSION_SCHEDULE_PATHPLANNER_HH__

#include <png.h>
#include "cti_spdlog.h"
#include "yaml-cpp/yaml.h"
#include "IPathPlanner.h"
#include "common/io/io.h"
#include "common/proto/topomap.pb.h"
#include "common/ScheduleUtils.h"
#include "IDeviceRuntimeUtility.h"
#include "common/ContainerInjector.h"
#include <boost/graph/adjacency_list.hpp>

namespace cti
{
  namespace missionSchedule
  {
    std::shared_ptr<IPathPlanner> createPathPlanner();
  }
}

#endif