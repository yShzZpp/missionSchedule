#ifndef __MISSION_SCHEDULE_ORDER_HH__
#define __MISSION_SCHEDULE_ORDER_HH__

#include <shared_mutex>
#include "ScheduleUtils.h"
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>

#include <road_control/density_srv.h>

namespace cti
{
  namespace missionSchedule
  {
    typedef enum
    {
      HIVE_COLLECT,               // Deliver hive to customer to collect package
      HIVE_DISPATCH,              // Deliver package in hive to customer
      HIVE_STERILIZATION,         // Deliver sterilize hive to work
      HIVE_AVOID_DOCK,            // Move hive out of charging dock
      HIVE_FORCE_CHARGE,          // Force hive to charge
      HIVE_DECOUPLING,            // Drop off hive
      HIVE_CHARGE,                // Deliver hive to charge
      HIVE_RETURN,                // Move hive to parking
      ROBOT_FORCE_CHARGE,         // Force robot to charge
      ROBOT_AVOID_DOCK,           // Move robot out of charging dock
      ROBOT_CHARGE,               // Move robot to charge
      ROBOT_RETURN,               // Move robot to parking
      UNDEFINE,
    } OrderType;

    typedef enum
    {
      STATENOTFOUND = 0,
      PENDING,
      PICKING_UP,
      MOUNTING,
      DROPPING_OFF,
      DISMOUNTING,
      WAITING,
      COMPLETED,
      CANCELLED,
      FAILED,
      EXPIRED,
    } OrderState;

    static bool isCommandTypeInOrderConsideration(const std::string& commandType)
    {
      if (commandType == "PAUSE" || commandType == "JACK")
      {
        return false;
      }
      return true;
    }

    static bool isOrderStateOnFinished(const OrderState& orderState)
    {
      if (orderState == OrderState::COMPLETED || orderState == OrderState::CANCELLED ||
          orderState == OrderState::EXPIRED || orderState == OrderState::FAILED)
      {
        return true;
      }
      return false;
    }

    struct SterilizationStep
    {
      bool reached{false};
      bool finished{false};
      int waitingDuration{0};
      std::string stepId;
      std::string waypointId;
      std::string waypointName;
      std::string sterilizationMode;
      std::chrono::system_clock::time_point waitingStartSec;
    };

    class ScheduleOrder : public std::enable_shared_from_this<ScheduleOrder>
    {
      private:
        void updateOrderCommandSequence(const std::shared_ptr<CommandVertex>& command);
        void updateOrderProgressAtFailedState(const std::shared_ptr<CommandVertex>& commandProgress);
        void updateOrderProgressAtExpiredState(const std::shared_ptr<CommandVertex>& commandProgress);
        void updateOrderProgressAtPendingState(const std::shared_ptr<CommandVertex>& commandProgress);
        void updateOrderProgressAtWaitingState(const std::shared_ptr<CommandVertex>& commandProgress);
        void updateOrderProgressAtMountingState(const std::shared_ptr<CommandVertex>& commandProgress);
        void updateOrderProgressAtCancelledState(const std::shared_ptr<CommandVertex>& commandProgress);
        void updateOrderProgressAtCompletedState(const std::shared_ptr<CommandVertex>& commandProgress);
        void updateOrderProgressAtPickingUpState(const std::shared_ptr<CommandVertex>& commandProgress);
        void updateOrderProgressAtDroppingOffState(const std::shared_ptr<CommandVertex>& commandProgress);
        void updateOrderProgressAtDismountingState(const std::shared_ptr<CommandVertex>& commandProgress);
        nlohmann::json computeOrderCommandAtFailedState();
        nlohmann::json computeOrderCommandAtExpiredState();
        nlohmann::json computeOrderCommandAtPendingState();
        nlohmann::json computeOrderCommandAtWaitingState();
        nlohmann::json computeOrderCommandAtMountingState();
        nlohmann::json computeOrderCommandAtCancelledState();
        nlohmann::json computeOrderCommandAtCompletedState();
        nlohmann::json computeOrderCommandAtPickingUpState();
        nlohmann::json computeOrderCommandAtDroppingOffState();
        nlohmann::json computeOrderCommandAtDismountingState();

        void waitingStateCallback(const boost::system::error_code& ec);
        void sterilizeWaitCallback(const boost::system::error_code& ec);

        OrderType orderType_;
        std::shared_timed_mutex mutex_;
        std::vector<std::string> orderTags_;
        int orderPriority_{0}, failedTimesCount_{0}, retryThreshold_{2}, effectiveArrivalWait_{0}, stepCount_{0}, peekQrCount_{0};
        OrderState orderState_{OrderState::PENDING}, predecessorState_{OrderState::PENDING};
        std::string id_, hiveOrderSetId_, hiveId_, hiveQr_, sourceStationId_, targetStationId_, orderScheduleType_, taskId_;
        bool shouldPickHive_{true}, sourceArrived_{false}, targetArrived_{false}, hivePicked_{false}, hiveDropped_{false}, paused_{false}, robotChargingContinue_{false}, authenticDeliver_{false}, designatedRobot_{false}, isSterilizeOrderReachedStation_{false};

        nlohmann::json orderContexts_;
        std::string ignoreCommandIds_;
        std::vector<std::string> peekedQr_;
        std::vector<std::string> fallbackWaypointId_;
        std::vector<SterilizationStep> sterilizationSteps_;
        std::map<OrderState, std::shared_ptr<CommandVertex>> commandSequence_;
        std::map<OrderState, std::function<nlohmann::json()>> computeCommandHandler_;
        std::map<OrderState, std::function<void(const std::shared_ptr<CommandVertex>& commandProgress)>> updateProgressHandler_;
        std::chrono::system_clock::time_point lastUpdateSec_;
        std::chrono::system_clock::time_point orderCreateSec_;
        std::chrono::system_clock::time_point pauseCommandExpireSec_;
        std::shared_ptr<boost::asio::deadline_timer> waitingPollTimer_, sterilizeWaitTimer_;
        std::tuple<bool, double> densityInfo_{true, -1.0};
        std::tuple<bool, double, int> frontDensityInfo_{true, -1.0, 0};
      public:
        ScheduleOrder();

        bool isOrderShouldPickHive();
        bool isOrderShouldDropHive();
        bool setOrderState(const OrderState& state);
        void setOrderTags(const std::vector<std::string>& tags);
        void setDensityInfo(const road_control::density_srvResponse density);
        void setFirstDensityInfo(const bool moveable, const double value, const int times);
        std::string computeHiveChargeStationId();
        nlohmann::json toJson();
        nlohmann::json computeOrderExecuteCommand();
        bool setAuthenticDeliver(bool authentic);
        std::string calculateBestTargetStationId();
        void updateSterilizationOrderProgress(bool isFinished);
        void cancelWaitingPollTimer();
        void updateOrderProgress(const nlohmann::json& progressJson);
        void synchronizePlatformUpdate(const nlohmann::json& platformJson);
        static std::shared_ptr<ScheduleOrder> parse(const nlohmann::json& object);
        static OrderState convertOrderStateFromStr(const std::string& orderStateStr);
        static OrderType convertTransitModeStrToOrderType(const std::string& transitMode);
        static const char* toString(const OrderType& orderType);
        static const char* toString(const OrderState& orderState);

        inline auto& hiveId() {return hiveId_; }
        inline auto& hiveQr() {return hiveQr_; }
        inline const auto& id() const { return id_; }
        inline const auto& taskId() const { return taskId_; }
        inline auto& hiveOrderSetId() {return hiveOrderSetId_; }
        inline const auto& orderType() const { return orderType_; }
        inline const auto& orderState() const { return orderState_; }
        inline const auto& hivePicked() const { return hivePicked_; }
        inline const auto& designatedRobot() const { return designatedRobot_; }
        inline const auto& shouldPickHive() const { return shouldPickHive_; }
        inline const auto& hiveDropped() const { return hiveDropped_; }
        inline const auto& lastUpdateSec() const { return lastUpdateSec_; }
        inline const auto& orderCreateSec() const { return orderCreateSec_; }
        inline const auto& orderPriority() const { return orderPriority_; }
        inline const auto& sourceArrived() const { return sourceArrived_; }
        inline const auto& targetArrived() const { return targetArrived_; }
        inline const auto& sourceStationId() const { return sourceStationId_; }
        inline const auto& targetStationId() const { return targetStationId_; }
        inline const auto& authenticDeliver() const { return authenticDeliver_; }
        inline const auto& robotChargingContinue() const { return robotChargingContinue_; }
        inline const auto& getFallbackWaypointId() const {return fallbackWaypointId_; }
        inline const auto& isSterilizeOrderReachedStation() const { return isSterilizeOrderReachedStation_; }
        inline const auto& getValueOfDensity() const { return std::get<1>(densityInfo_); }
        inline const auto& getMoveableFromDensity() const { return std::get<0>(densityInfo_); }
        inline const auto& getFrontTimesOfDensity() const { return std::get<2>(frontDensityInfo_); }
        inline const auto& getFrontValueOfDensity() const { return std::get<1>(frontDensityInfo_); }
        inline const auto& getFrontMoveableFromDensity() const { return std::get<0>(frontDensityInfo_); }
    };
  }
}

#endif
