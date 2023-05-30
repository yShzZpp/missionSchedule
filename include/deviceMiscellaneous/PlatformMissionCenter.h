#ifndef __MISSION_SCHEDULE_PLATFORMMISSIONCENTER_HH__
#define __MISSION_SCHEDULE_PLATFORMMISSIONCENTER_HH__

#include <queue>
#include <fstream>
#include "ros/ros.h"
#include "cti_spdlog.h"
#include "std_msgs/String.h"
#include "nlohmann/json.hpp"
#include "common/Hungarian.h"
#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include "IPlatformMissionCenter.h"
#include "IMissionPlanner.h"
#include "common/ScheduleOrder.h"
#include "common/ContainerInjector.h"
#include "cti_msgs/DispathRequest.h"
#include "IDeviceRuntimeUtility.h"
#include "IElevatorPlanner.h"
#include "IPathPlanner.h"

namespace cti
{
  namespace missionSchedule
  {
    static std::map<OrderType, int> orderTypeCompareTable = { {OrderType::ROBOT_FORCE_CHARGE, 1},
                                                              {OrderType::HIVE_AVOID_DOCK, 2},
                                                              {OrderType::HIVE_FORCE_CHARGE, 3},
                                                              {OrderType::HIVE_DECOUPLING, 4},
                                                              {OrderType::HIVE_COLLECT, 5},
                                                              {OrderType::HIVE_DISPATCH, 5},
                                                              {OrderType::HIVE_STERILIZATION, 6},
                                                              {OrderType::HIVE_CHARGE, 6},
                                                              {OrderType::ROBOT_AVOID_DOCK, 7},
                                                              {OrderType::HIVE_RETURN, 8},
                                                              {OrderType::ROBOT_CHARGE, 9},
                                                              {OrderType::ROBOT_RETURN, 10}};
    class HiveOrderSet
    {
      private:
        std::shared_timed_mutex mutex_;
        std::shared_ptr<ScheduleOrder> currentOrder_;
        std::string hiveOrderSetId_, hiveId_, hiveQr_;
        std::vector<std::string> hiveTags_;
        std::vector<std::string> insertOrderTable_;
        std::map<std::string, std::shared_ptr<ScheduleOrder>> deliverOrders_;
        std::shared_ptr<ScheduleOrder> pickBetterOrder(std::shared_ptr<ScheduleOrder> first,
                                                       std::shared_ptr<ScheduleOrder> second);
        std::chrono::system_clock::time_point lastUpdateSec_;
        std::chrono::system_clock::time_point lastPlatformUpdateSec_;
        std::map<std::string, std::chrono::system_clock::time_point> orderFirstObserveTimeMap_;

        bool partialDensity_ = true;
        double densityRange_ = 10.0;
        ros::ServiceClient densityInfoClient_;

      public:
        HiveOrderSet(const std::string& id, const std::string& hiveId) : hiveOrderSetId_(id), hiveId_(hiveId) 
        {
          if (auto nodeHandle = cti::missionSchedule::common::getContainer()->resolveOrNull<ros::NodeHandle>())
          {
            densityInfoClient_ = nodeHandle->serviceClient<road_control::density_srv>("/road_control/density_info");
            // nodeHandle->getParam("/partial_density", partialDensity_);
            // nodeHandle->getParam("/density_range", densityRange_);
            nodeHandle->param("/mission_schedule/partial_density", partialDensity_, true);
            nodeHandle->param("/mission_schedule/density_range", densityRange_, 10.0);
          }
        }
        nlohmann::json toJson();
        inline const auto & hiveId() {return hiveId_; }
        inline auto & hiveTags() { return hiveTags_; }
        inline auto & lastUpdateSec() { return lastUpdateSec_; }
        inline auto & hiveOderSetId() { return hiveOrderSetId_; }
        inline auto & lastPlatformUpdateSec() { return lastPlatformUpdateSec_; }
        bool isAllOrderFinished();
        size_t getHiveScheduleOrderSize();
        nlohmann::json computeBestDeliverOrderCommand();
        bool addScheduleOrder(const std::shared_ptr<ScheduleOrder>& order);
        void updateOrderSetProgress(const nlohmann::json& progressJson);
        std::shared_ptr<ScheduleOrder> getScheduleOrder(const std::string& orderId);
        bool eraseOrder(const std::string& orderId);
        void foreachScheduleOrder(std::function<bool(std::shared_ptr<ScheduleOrder>)> handler);
        void addOrderFirstObserveTime(std::string order_id);
        void delOrderFirestObserveTime(std::string order_id);
        int getDurationOfCurrentTimeAndFirstObserveTime(std::string order_id);
        road_control::density_srvResponse callDensityServer(std::shared_ptr<cti::missionSchedule::RobotUtility> robot, Position orderDestination);
    };

    std::shared_ptr<IPlatformMissionCenter> createPlatformMissionCenter();
  }
}



#endif
