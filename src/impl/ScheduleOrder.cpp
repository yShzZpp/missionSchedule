#include <chrono>
#include <fstream>
#include "IMissionPlanner.h"
#include "IPlatformMissionCenter.h"
#include "IDeviceRuntimeUtility.h"
#include "common/ScheduleOrder.h"
#include "common/ContainerInjector.h"
#include "cti_spdlog.h"


/*
  HIVE_DIAPATCH
    Wankeyuncheng
      {"endWaypointId" : "dd6c1725-c37e-430a-9d68-f3049918f711",
      "startWaypointId": "17cf5185-f8b3-4207-8dc7-5b7590890247",
      "hiveId" : "76feec13-da97-4f17-8b94-fdd70d6fe039",
      "effectiveArrivalWait" : 60000,
      "priority" : 0,
      "zoneId" : "e90d7310-959c-4948-bf09-c67e93600848",
      "lockerIds":["95ddf51e-029e-464e-a13b-c5484f8517da"]
      }
    Tiananyungu
      {
        "zoneId": "8e530a0b-5eca-4440-a03a-307cb6bb9005",
        "buildingId": "dc00147a-04e3-4907-ac2a-35077ad6fce6",
        "startWaypointId": "7c895dab-454f-40f9-a02c-560f9e9e06f2",
        "endWaypointId": "dc2b3f81-59cb-48c2-9752-b97fa825dc50",
        "hiveId": "22c92c8e-f5e5-4d7b-a080-5c616f26be8e",
        "hive": {
          "id": "22c92c8e-f5e5-4d7b-a080-5c616f26be8e",
          "name": "#81柜",
          "qr": "081"
        },
        "priority": 0,
        "effectiveArrivalWait": 30000,
        "lockerIds":["a36b6d84-f70e-493d-b743-6784d1ad94c7"]
      }
      {
        "zoneId": "8e530a0b-5eca-4440-a03a-307cb6bb9005",
        "buildingId": "dc00147a-04e3-4907-ac2a-35077ad6fce6",
        "startWaypointId": "289eb851-251b-4a53-a422-c4bb54162a98",
        "endWaypointId": "3786e7ee-c2f0-4a11-af7e-ab2c2b5674c8",
        "hiveId": "5a652e2a-b879-4df4-b0a2-41c1bddc861d",
        "hive": {
          "id": "5a652e2a-b879-4df4-b0a2-41c1bddc861d",
          "name": "六格箱・00077",
          "qr": "700"
        },
        "priority": 0,
        "effectiveArrivalWait": 30000,
        "lockerIds":["5454e287-4fe2-4f4d-acb8-24b8ea83ad6f"]
      }

  HIVE_COLLECT
    Wankeyuncheng
      {"endWaypointId" : "06226240-169c-44b4-aa3f-7c8c4fd32aac",
      "startWaypointId": "59d5ba95-53a7-4074-a841-bc902d07ee97",
      "hiveId" : "76feec13-da97-4f17-8b94-fdd70d6fe039",
      "effectiveArrivalWait" : 60000,
      "priority" : 0,
      "zoneId" : "e90d7310-959c-4948-bf09-c67e93600848",
      "lockerIds":["95ddf51e-029e-464e-a13b-c5484f8517da"]
      }

  HIVE_CHARGE
  {"endWaypointId": "250e26e7-b255-4914-a674-edbd75274fe6",
  "startWaypointId": "8c656fcd-bcc6-494b-9761-0ed01aa274db",
  "hiveId" : "76feec13-da97-4f17-8b94-fdd70d6fe039",
  "zoneId" : "e90d7310-959c-4948-bf09-c67e93600848"
  }

  HIVE_RETURN
    Tiananyungu
      {
        "zoneId": "8e530a0b-5eca-4440-a03a-307cb6bb9005",
        "buildingId": "dc00147a-04e3-4907-ac2a-35077ad6fce6",
        "startWaypointId": "9271cf98-ffb1-4406-9cdb-52ab7d85674e",
        "endWaypointId": "7c895dab-454f-40f9-a02c-560f9e9e06f2",
        "hiveId": "22c92c8e-f5e5-4d7b-a080-5c616f26be8e",
        "hive": {
          "id": "22c92c8e-f5e5-4d7b-a080-5c616f26be8e",
          "name": "#81柜",
          "qr": "081"
        },
        "priority": 0,
        "lockerIds":["a36b6d84-f70e-493d-b743-6784d1ad94c7"]
      }
      {
        "zoneId": "8e530a0b-5eca-4440-a03a-307cb6bb9005",
        "buildingId": "dc00147a-04e3-4907-ac2a-35077ad6fce6",
        "startWaypointId": "2f9fc4f4-95d5-4ee9-afe5-fc4122f03b9e",
        "endWaypointId": "289eb851-251b-4a53-a422-c4bb54162a98",
        "hiveId": "5a652e2a-b879-4df4-b0a2-41c1bddc861d",
        "hive": {
          "id": "5a652e2a-b879-4df4-b0a2-41c1bddc861d",
          "name": "六格箱・00077",
          "qr": "700"
        },
        "priority": 0,
        "lockerIds":["5454e287-4fe2-4f4d-acb8-24b8ea83ad6f"]
      }

*/

namespace cti
{
  namespace missionSchedule
  {
    OrderType ScheduleOrder::convertTransitModeStrToOrderType(const std::string& transitMode)
    {
      if ("HIVE_COLLECT" == transitMode)
      {
        return OrderType::HIVE_COLLECT;
      }
      else if ("HIVE_DISPATCH" == transitMode)
      {
        return OrderType::HIVE_DISPATCH;
      }
      else if ("HIVE_STERILIZATION" == transitMode)
      {
        return OrderType::HIVE_STERILIZATION;
      }
      else if ("HIVE_AVOID_DOCK" == transitMode)
      {
        return OrderType::HIVE_AVOID_DOCK;
      }
      else if ("HIVE_FORCE_CHARGE" == transitMode)
      {
        return OrderType::HIVE_FORCE_CHARGE;
      }
      else if ("HIVE_DECOUPLING" == transitMode)
      {
        return OrderType::HIVE_DECOUPLING;
      }
      else if ("HIVE_CHARGE" == transitMode)
      {
        return OrderType::HIVE_CHARGE;
      }
      else if ("HIVE_RETURN" == transitMode)
      {
        return OrderType::HIVE_RETURN;
      }
      else if ("ROBOT_FORCE_CHARGE" == transitMode)
      {
        return OrderType::ROBOT_FORCE_CHARGE;
      }
      else if ("ROBOT_AVOID_DOCK" == transitMode)
      {
        return OrderType::ROBOT_AVOID_DOCK;
      }
      else if ("ROBOT_CHARGE" == transitMode)
      {
        return OrderType::ROBOT_CHARGE;
      }
      else if ("ROBOT_RETURN" == transitMode)
      {
        return OrderType::ROBOT_RETURN;
      }
      return OrderType::UNDEFINE;
    }

    OrderState ScheduleOrder::convertOrderStateFromStr(const std::string& orderState)
    {
      if ("PENDING" == orderState)
      {
        return OrderState::PENDING;
      }
      if ("PICKING_UP" == orderState)
      {
        return OrderState::PICKING_UP;
      }
      if ("MOUNTING" == orderState)
      {
        return OrderState::MOUNTING;
      }
      if ("DROPPING_OFF" == orderState)
      {
        return OrderState::DROPPING_OFF;
      }
      if ("DISMOUNTING" == orderState)
      {
        return OrderState::DISMOUNTING;
      }
      if ("WAITING" == orderState)
      {
        return OrderState::WAITING;
      }
      if ("COMPLETED" == orderState)
      {
        return OrderState::COMPLETED;
      }
      if ("CANCELLED" == orderState)
      {
        return OrderState::CANCELLED;
      }
      if ("FAILED" == orderState)
      {
        return OrderState::FAILED;
      }
      if ("EXPIRED" == orderState)
      {
        return OrderState::EXPIRED;
      }
      return OrderState::STATENOTFOUND;
    }

    const char* ScheduleOrder::toString(const OrderType& orderType)
    {
      switch (orderType)
      {
        case OrderType::HIVE_COLLECT :
          return "HIVE_COLLECT";
        case OrderType::HIVE_DISPATCH :
          return "HIVE_DISPATCH";
        case OrderType::HIVE_STERILIZATION :
          return "HIVE_STERILIZATION";
        case OrderType::HIVE_AVOID_DOCK :
          return "HIVE_AVOID_DOCK";
        case OrderType::HIVE_FORCE_CHARGE :
          return "HIVE_FORCE_CHARGE";
        case OrderType::HIVE_CHARGE :
          return "HIVE_CHARGE";
        case OrderType::HIVE_DECOUPLING :
          return "HIVE_DECOUPLING";
        case OrderType::HIVE_RETURN :
          return "HIVE_RETURN";
        case OrderType::ROBOT_FORCE_CHARGE :
          return "ROBOT_FORCE_CHARGE";
        case OrderType::ROBOT_AVOID_DOCK :
          return "ROBOT_AVOID_DOCK";
        case OrderType::ROBOT_CHARGE :
          return "ROBOT_CHARGE";
        case OrderType::ROBOT_RETURN :
          return "ROBOT_RETURN";
        default :
          return "UNDEFINE";
      }
    }

    const char* ScheduleOrder::toString(const OrderState& orderState)
    {
      switch (orderState)
      {
        case OrderState::PENDING :
          return "PENDING";
        case OrderState::PICKING_UP :
          return "PICKING_UP";
        case OrderState::MOUNTING :
          return "MOUNTING";
        case OrderState::DROPPING_OFF :
          return "DROPPING_OFF";
        case OrderState::DISMOUNTING :
          return "DISMOUNTING";
        case OrderState::WAITING :
          return "WAITING";
        case OrderState::COMPLETED :
          return "COMPLETED";
        case OrderState::CANCELLED :
          return "CANCELLED";
        case OrderState::FAILED :
          return "FAILED";
        case OrderState::EXPIRED:
          return "EXPIRED";
        default :
          return "STATENOTFOUND";
      }
    }

    std::shared_ptr<ScheduleOrder> ScheduleOrder::parse(const nlohmann::json& object)
    {
      if (!object.is_object())
      {
        return nullptr;
      }
      auto item = std::make_shared<ScheduleOrder>();
      if (object.contains("id") && object["id"].is_string())
      {
        item->id_ = object.at("id").get<std::string>();
      }
      if (object.contains("taskId") && object["taskId"].is_string())
      {
        item->taskId_ = object.at("taskId").get<std::string>();
      }
      if (object.contains("shouldPickHive") && object["shouldPickHive"].is_boolean())
      {
        item->shouldPickHive_ = object["shouldPickHive"].get<bool>();
      }
      if (object.contains("paused") && object["paused"].is_boolean())
      {
        item->paused_ = object["paused"].get<bool>();
      }
      if (object.contains("hiveId") && object["hiveId"].is_string())
      {
        item->hiveId_ = object.at("hiveId").get<std::string>();
      }
      if (object.contains("hive") && object["hive"].contains("qr"))
      {
        item->hiveQr_ = object.at("hive").at("qr").get<std::string>();
      }
      if (object.contains("priority") && object["priority"].is_number_integer())
      {
        item->orderPriority_ = object["priority"].get<int>();
      }
      if (object.contains("effectiveArrivalWait") && object["effectiveArrivalWait"].is_number_integer())
      {
        item->effectiveArrivalWait_ = object["effectiveArrivalWait"].get<int>();
      }
      if (object.contains("designatedRobot") && object["designatedRobot"].is_boolean())
      {
        item->designatedRobot_ = object["designatedRobot"].get<bool>();
      }
      if (object.contains("transitMode") && object["transitMode"].is_string())
      {
        item->orderType_ = convertTransitModeStrToOrderType(object["transitMode"].get<std::string>());
      }
      if (object.contains("transitState") && object["transitState"].is_string())
      {
        item->orderState_ = convertOrderStateFromStr(object["transitState"].get<std::string>());
      }
      if (object.contains("contexts") && object["contexts"].is_array())
      {
        item->orderContexts_ = object["contexts"];
      }
      if (object.contains("fallbackWaypointIdList") && object["fallbackWaypointIdList"].is_array())
      {
        for (auto fallbackWaypoint : object["fallbackWaypointIdList"])
        {
          item->fallbackWaypointId_.push_back(fallbackWaypoint);
        }
      }
      if (object.contains("startWaypointId") && object["startWaypointId"].is_string())
      {
        item->sourceStationId_ = object["startWaypointId"].get<std::string>();
      }
      if (object.contains("steps") && object["steps"].is_array())
      {
        for (auto stepCount = 0; stepCount < object["steps"].size(); stepCount++)
        {
          SterilizationStep step;
          step.stepId = object["steps"][stepCount]["id"].get<std::string>();
          step.waitingDuration = object["steps"][stepCount]["duration"].get<int>();
          step.waypointId = object["steps"][stepCount]["waypointId"].get<std::string>();
          step.waypointName = object["steps"][stepCount]["waypointName"].get<std::string>();
          if (object["steps"][stepCount].contains("sterilizationMode"))
          {
            step.sterilizationMode =  object["steps"][stepCount]["sterilizationMode"].get<std::string>();
          }
          item->sterilizationSteps_.push_back(std::move(step));
        }
      }
      if (object.contains("endWaypointId") && object["endWaypointId"].is_string())
      {
        item->targetStationId_ = object["endWaypointId"].get<std::string>();
      }
      if (object.contains("createdTime") && object["createdTime"].is_number_integer())
      {
        auto timeCount = object.at("createdTime").get<int64_t>();
        std::chrono::milliseconds dur(timeCount);
        std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> dt(dur);
        item->orderCreateSec_ = dt;
      }
      if (object.contains("orderScheduleType") && object["orderScheduleType"].is_string())
      {
        item->orderScheduleType_ = object["orderScheduleType"].get<std::string>();
      }
      return item;
    }

    ScheduleOrder::ScheduleOrder()
    {
      this->computeCommandHandler_.emplace(OrderState::FAILED,       std::bind(&ScheduleOrder::computeOrderCommandAtFailedState, this));
      this->computeCommandHandler_.emplace(OrderState::PENDING,      std::bind(&ScheduleOrder::computeOrderCommandAtPendingState, this));
      this->computeCommandHandler_.emplace(OrderState::EXPIRED,      std::bind(&ScheduleOrder::computeOrderCommandAtExpiredState, this));
      this->computeCommandHandler_.emplace(OrderState::WAITING,      std::bind(&ScheduleOrder::computeOrderCommandAtWaitingState, this));
      this->computeCommandHandler_.emplace(OrderState::MOUNTING,     std::bind(&ScheduleOrder::computeOrderCommandAtMountingState, this));
      this->computeCommandHandler_.emplace(OrderState::COMPLETED,    std::bind(&ScheduleOrder::computeOrderCommandAtCompletedState, this));
      this->computeCommandHandler_.emplace(OrderState::CANCELLED,    std::bind(&ScheduleOrder::computeOrderCommandAtCancelledState, this));
      this->computeCommandHandler_.emplace(OrderState::PICKING_UP,   std::bind(&ScheduleOrder::computeOrderCommandAtPickingUpState, this));
      this->computeCommandHandler_.emplace(OrderState::DISMOUNTING,  std::bind(&ScheduleOrder::computeOrderCommandAtDismountingState, this));
      this->computeCommandHandler_.emplace(OrderState::DROPPING_OFF, std::bind(&ScheduleOrder::computeOrderCommandAtDroppingOffState, this));

      this->updateProgressHandler_.emplace(OrderState::FAILED,       std::bind(&ScheduleOrder::updateOrderProgressAtFailedState, this, std::placeholders::_1));
      this->updateProgressHandler_.emplace(OrderState::PENDING,      std::bind(&ScheduleOrder::updateOrderProgressAtPendingState, this, std::placeholders::_1));
      this->updateProgressHandler_.emplace(OrderState::EXPIRED,      std::bind(&ScheduleOrder::updateOrderProgressAtExpiredState, this, std::placeholders::_1));
      this->updateProgressHandler_.emplace(OrderState::WAITING,      std::bind(&ScheduleOrder::updateOrderProgressAtWaitingState, this, std::placeholders::_1));
      this->updateProgressHandler_.emplace(OrderState::MOUNTING,     std::bind(&ScheduleOrder::updateOrderProgressAtMountingState, this, std::placeholders::_1));
      this->updateProgressHandler_.emplace(OrderState::COMPLETED,    std::bind(&ScheduleOrder::updateOrderProgressAtCompletedState, this, std::placeholders::_1));
      this->updateProgressHandler_.emplace(OrderState::CANCELLED,    std::bind(&ScheduleOrder::updateOrderProgressAtCancelledState, this, std::placeholders::_1));
      this->updateProgressHandler_.emplace(OrderState::PICKING_UP,   std::bind(&ScheduleOrder::updateOrderProgressAtPickingUpState, this, std::placeholders::_1));
      this->updateProgressHandler_.emplace(OrderState::DISMOUNTING,  std::bind(&ScheduleOrder::updateOrderProgressAtDismountingState, this, std::placeholders::_1));
      this->updateProgressHandler_.emplace(OrderState::DROPPING_OFF, std::bind(&ScheduleOrder::updateOrderProgressAtDroppingOffState, this, std::placeholders::_1));
    }

    void ScheduleOrder::setOrderTags(const std::vector<std::string>& tags)
    {
      std::unique_lock<std::shared_timed_mutex> lk(mutex_);
      orderTags_ = tags;
    }

    bool ScheduleOrder::setOrderState(const OrderState& state)
    {
      std::unique_lock<std::shared_timed_mutex> lk(mutex_);
      orderState_ = state;
      return true;
    }


    bool ScheduleOrder::setAuthenticDeliver(bool authentic)
    {
      std::unique_lock<std::shared_timed_mutex> lk(mutex_);
      authenticDeliver_ = authentic;
      return true;
    }

    nlohmann::json ScheduleOrder::toJson()
    {
      std::unique_lock<std::shared_timed_mutex> lk(mutex_);
      nlohmann::json orderJson;
      orderJson["id"] = id_;
      orderJson["hiveId"] = hiveId_;
      orderJson["hive"]["qr"] = hiveQr_;
      orderJson["priority"] = orderPriority_;
      orderJson["effectiveArrivalWait"] = effectiveArrivalWait_;
      orderJson["transitMode"] = toString(orderType_);
      orderJson["transitState"] = toString(orderState_);
      orderJson["startWaypointId"] = sourceStationId_;
      orderJson["endWaypointId"] = targetStationId_;
      orderJson["orderScheduleType"] = orderScheduleType_;
      if (!orderContexts_.empty())
      {
        orderJson["contexts"] = orderContexts_;
      }
      if (!fallbackWaypointId_.empty())
      {
        orderJson["fallbackWaypointIdList"] = nlohmann::json::array();
        for (auto fallbackWaypoint : fallbackWaypointId_)
        {
          orderJson["fallbackWaypointIdList"].push_back(fallbackWaypoint);
        }
      }
      if (!taskId_.empty())
      {
        orderJson["taksId"] = taskId_;
      }
      if (!shouldPickHive_)
      {
        orderJson["shouldPickHive"] = shouldPickHive_;
      }
      if (paused_)
      {
        orderJson["paused"] = paused_;
      }
      return orderJson;
    }

    void ScheduleOrder::sterilizeWaitCallback(const boost::system::error_code& ec)
    {
      if (ec == boost::system::errc::success)
      {
        if (orderState_ == OrderState::DROPPING_OFF)
        {
          nlohmann::json infraryJson;
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          sterilizationSteps_[stepCount_].finished = true;
          infraryJson["isHiveCanSterilize"] = false;
          isSterilizeOrderReachedStation_ = false;
          deviceUtility->useInfraryToSendMsg(infraryJson.dump());
          SPDLOG_INFO("ScheduleOrder timeup ,send: {}",infraryJson.dump());
          stepCount_++;
        }
      }
    }

    void ScheduleOrder::updateSterilizationOrderProgress(bool isFinished)
    {
      std::unique_lock<std::shared_timed_mutex> lk(mutex_);
      nlohmann::json infraryJson;
      if (sterilizeWaitTimer_)
      {
        boost::system::error_code ec;
        sterilizeWaitTimer_->cancel();
        sterilizeWaitTimer_.reset();
        // sterilizationSteps_[stepCount_].waitingDuration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - \
        //   sterilizationSteps_[stepCount_].waitingStartSec).count();
      }

      if (isFinished && orderState_ == OrderState::DROPPING_OFF)
      {
        sterilizationSteps_[stepCount_].finished = true;
        stepCount_++;
      }
      if (isSterilizeOrderReachedStation_)
      {
        infraryJson["isHiveCanSterilize"] = false;
        isSterilizeOrderReachedStation_ = false;
        if (auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>())
        {
          deviceUtility->useInfraryToSendMsg(infraryJson.dump());
        }
        SPDLOG_INFO("ScheduleOrder cancel timer [{}] finish ,send: {}", isFinished, infraryJson.dump());
      }
      if (stepCount_ < sterilizationSteps_.size())
      {
        sterilizationSteps_[stepCount_].reached = false;
      }
      else
      {
        orderState_ = OrderState::COMPLETED;
        if (auto missionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>())
        {
          missionCenter->reportOrderProgress(id_, toString(orderState_));
        }
        lastUpdateSec_ = std::chrono::system_clock::now();
        SPDLOG_INFO("ScheduleOrder {} change state to completed at {}", id_, __func__);
      }
    }

    void ScheduleOrder::waitingStateCallback(const boost::system::error_code& ec)
    {
      if (ec == boost::system::errc::success)
      {
        if (orderState_ == OrderState::WAITING)
        {
          this->orderState_ = OrderState::COMPLETED;
          if (auto missionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>())
          {
            missionCenter->reportOrderProgress(id_, toString(orderState_));
          }
          lastUpdateSec_ = std::chrono::system_clock::now();
          SPDLOG_INFO("ScheduleOrder {} change state to completed at {}", id_, __func__);
        }
      }
    }

    void ScheduleOrder::cancelWaitingPollTimer()
    {
      std::unique_lock<std::shared_timed_mutex> lk(mutex_);
      if (orderState_ == OrderState::WAITING)
      {
        this->orderState_ = OrderState::COMPLETED;
        SPDLOG_INFO("ScheduleOrder {} change state to completed at {}", id_, __func__);
        if (waitingPollTimer_)
        {
          boost::system::error_code ec;
          waitingPollTimer_->cancel();
          waitingPollTimer_.reset();
        }
        lastUpdateSec_ = std::chrono::system_clock::now();
      }
      else
      {
        SPDLOG_INFO("ScheduleOrder {} order state ({}) != WAITING", id_, this->orderState_);
      }
    }

    bool ScheduleOrder::isOrderShouldPickHive()
    {
      return orderType_ == OrderType::HIVE_COLLECT || orderType_ == OrderType::HIVE_DISPATCH ||
             orderType_ == OrderType::HIVE_STERILIZATION || orderType_ == OrderType::HIVE_AVOID_DOCK ||
             orderType_ == OrderType::HIVE_FORCE_CHARGE || orderType_ == OrderType::HIVE_DECOUPLING ||
             orderType_ == OrderType::HIVE_CHARGE || orderType_ == OrderType::HIVE_RETURN;
    }

    bool ScheduleOrder::isOrderShouldDropHive()
    {
      return orderType_ == OrderType::HIVE_AVOID_DOCK || orderType_ == OrderType::HIVE_FORCE_CHARGE ||
             orderType_ == OrderType::HIVE_DECOUPLING || orderType_ == OrderType::HIVE_CHARGE ||
             orderType_ == OrderType::HIVE_RETURN;
    }

    void ScheduleOrder::updateOrderCommandSequence(const std::shared_ptr<CommandVertex>& command)
    {
      if (command->orderState().empty() || command->orderId().empty() || command->orderId() != id_)
      {
        return;
      }
      // if (command->stationId() != targetStationId_ && HIVE_STERILIZATION == orderType_ && orderState_ == OrderState::DROPPING_OFF)
      // {
      //   return;
      // }
      auto orderState = convertOrderStateFromStr(command->orderState());
      SPDLOG_INFO("ScheduleOrder {} state update command {}", command->orderState(), command->toJson().dump());
      commandSequence_[orderState] = command;
    }

    void ScheduleOrder::updateOrderProgress(const nlohmann::json& progressJson)
    {
      std::unique_lock<std::shared_timed_mutex> lk(mutex_);
      std::shared_ptr<CommandVertex> command = CommandVertex::parseFromJson(progressJson);
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      hivePicked_ = deviceUtility->isHiveAttachedOnRobot();
      OrderState tempState = orderState_;
      if (HIVE_STERILIZATION == orderType_ && orderState_ == OrderState::DROPPING_OFF)
      {
        nlohmann::json infraryJson;
        if ("PAUSE" == command->commandType() && CommandState::COMMAND_COMPLETED == command->commandState())
        {
          if (stepCount_ < sterilizationSteps_.size() && sterilizationSteps_[stepCount_].reached && !sterilizationSteps_[stepCount_].finished)
          {
            int leftWaitingDuration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - \
                sterilizationSteps_[stepCount_].waitingStartSec).count();
            if (leftWaitingDuration < sterilizationSteps_[stepCount_].waitingDuration)
            {
              sterilizationSteps_[stepCount_].waitingDuration = leftWaitingDuration;
              boost::system::error_code ec;
              sterilizeWaitTimer_->cancel(ec);
              sterilizeWaitTimer_.reset();
              if (isSterilizeOrderReachedStation_)
              {
                infraryJson["isHiveCanSterilize"] = false;
                isSterilizeOrderReachedStation_ = false;
                SPDLOG_INFO("ScheduleOrder sterilize pause ,send: {}",infraryJson.dump());
              }
            }
          }
        }
        else if ("RESUME" == command->commandType() && CommandState::COMMAND_COMPLETED == command->commandState())
        {
          if (sterilizationSteps_[stepCount_].waitingDuration)
          {
            auto ioService = cti::missionSchedule::common::getContainer()->resolveOrNull<boost::asio::io_service>();
            sterilizeWaitTimer_ = std::make_shared<boost::asio::deadline_timer>(*ioService);
            sterilizeWaitTimer_->expires_from_now(boost::posix_time::seconds(sterilizationSteps_[stepCount_].waitingDuration));
            sterilizeWaitTimer_->async_wait([self = shared_from_this()](const auto& ec) { self->sterilizeWaitCallback(ec); });
            sterilizationSteps_[stepCount_].waitingStartSec = std::chrono::system_clock::now();
            infraryJson["isHiveCanSterilize"] = true;
            isSterilizeOrderReachedStation_ = true;
            SPDLOG_INFO("ScheduleOrder sterilize resume,send {}",infraryJson.dump());
          }
        }
        if (!infraryJson.empty())
        {
          deviceUtility->useInfraryToSendMsg(infraryJson.dump());
        }
      }
      if ((progressJson.contains("orderId") && progressJson["orderId"] == id_)
          || (!hivePicked_ && progressJson.contains("hiveOrderSetId") && progressJson["hiveOrderSetId"] == hiveOrderSetId_))
      {
        updateOrderCommandSequence(command);
        if (CommandState::COMMAND_DIRECT_FAILED == command->commandState() && HIVE_STERILIZATION != orderType_)
        {
          robotChargingContinue_ = false;
          orderState_ = OrderState::FAILED;
        }
        else if (CommandState::COMMAND_FAILED == command->commandState())
        {
          if (command->reason() == "无法停止现有的导航任务")
          {
            pauseCommandExpireSec_ = std::chrono::system_clock::now() + std::chrono::seconds(10);
          }
          else if (HIVE_STERILIZATION == orderType_ && orderState_ == OrderState::DROPPING_OFF)
          {
            failedTimesCount_ = 0;
          }
          else if (OrderState::DROPPING_OFF == orderState_ && !fallbackWaypointId_.empty())
          {
            targetStationId_ = fallbackWaypointId_[0];
            fallbackWaypointId_.erase(fallbackWaypointId_.begin());
          }
          else
          {
            failedTimesCount_++;
            if (failedTimesCount_ >= retryThreshold_)
            {
              robotChargingContinue_ = false;
              orderState_ = OrderState::FAILED;
            }
          }
        }
      }
      else if (orderScheduleType_ != "PLATFORM" && !isCommandStateOnFinished(command->commandState()))
      {
        orderState_ = OrderState::CANCELLED;
        SPDLOG_INFO("ScheduleOrder update special state to {} from progressJson {}", toString(orderState_), progressJson.dump());
      }
      updateProgressHandler_[orderState_](command);
      if (tempState != orderState_)
      {
        failedTimesCount_ = 0;
        predecessorState_ = tempState;
      }
      if ((tempState != orderState_ || command->commandState() == CommandState::COMMAND_QUEUEING) && isCommandTypeInOrderConsideration(command->commandType()))
      {
        SPDLOG_INFO("ScheduleOrder update state to {} from progressJson {}", toString(orderState_), progressJson.dump());
        if (command->orderId() == id_)
        {
          authenticDeliver_ = true;
        }
        else
        {
          authenticDeliver_ = false;
        }
        if (auto missionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>())
        {
          if (orderState_ == OrderState::DROPPING_OFF && command->orderId() == id_)
          {
            missionCenter->reportOrderProgress(id_, toString(orderState_), "", true);
          }
          else if (progressJson.contains("reason") && progressJson.at("reason").is_string())
          {
            missionCenter->reportOrderProgress(id_, toString(orderState_), progressJson.at("reason").get<std::string>());
          }
          else
          {
            missionCenter->reportOrderProgress(id_, toString(orderState_));
          }
        }
      }
      lastUpdateSec_ = std::chrono::system_clock::now();
    }

    void ScheduleOrder::synchronizePlatformUpdate(const nlohmann::json& platformJson)
    {
      std::unique_lock<std::shared_timed_mutex> lk(mutex_);
      OrderState tempState = orderState_;
      nlohmann::json orderJson;
      SPDLOG_INFO("ScheduleOrder synchronize platform update {}", platformJson.dump());
      if (platformJson.is_array() && platformJson.size() &&
          platformJson[0].contains("transitRequests") &&
          platformJson[0]["transitRequests"].is_array() &&
          platformJson[0]["transitRequests"].size() &&
          platformJson[0]["transitRequests"][0].contains("id") &&
          platformJson[0]["transitRequests"][0].contains("transitState") &&
          platformJson[0]["transitRequests"][0]["id"] == id_)
      {
        orderJson = platformJson[0]["transitRequests"][0];
      }
      else if (platformJson.contains("id") &&
               platformJson.contains("transitState") &&
               platformJson["id"] == id_)
      {
        orderJson = platformJson;
      }
      if (orderJson.empty())
      {
        return;
      }

      auto platformState = convertOrderStateFromStr(orderJson["transitState"].get<std::string>());
      if (platformState < orderState_)
      {
        if (platformState == OrderState::DROPPING_OFF && orderState_ == OrderState::WAITING)
        {
          if (auto missionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>())
          {
            missionCenter->reportOrderProgress(id_, toString(orderState_));
          }
        }
        return ;
      }
      SPDLOG_INFO("ScheduleOrder synchronize platform update {}", id_);
      orderState_ = platformState;
      if (orderJson.contains("hive") && orderJson["hive"].contains("robotId"))
      {
        auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
        auto robotId = deviceUtility->getRobotCloudId();
        if (!orderJson["hive"]["robotId"].is_null() && !orderJson["hive"]["robotId"].get<std::string>().empty() &&
            orderJson["hive"]["robotId"].get<std::string>() != robotId && !isOrderStateOnFinished(tempState))
        {
          orderState_ = OrderState::CANCELLED;
          if (auto missionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>())
          {
            missionCenter->reportOrderProgress(id_, toString(orderState_), "柜体已被其他机器人装载");
          }
        }
      }
      if (authenticDeliver_ &&
        (orderState_ == OrderState::CANCELLED || orderState_ == OrderState::FAILED) && !isOrderStateOnFinished(tempState))
      {
        auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
        if ((commandSequence_.find(tempState) == commandSequence_.end() ||
            !isCommandStateOnFinished(commandSequence_[tempState]->commandState()))
            && deviceUtility->isDeviceStatusOk())
        {
          auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
          missionPlanner->publishAbortCommand(id_);
        }
      }
      if (isOrderStateOnFinished(orderState_) && orderType_ == OrderType::HIVE_STERILIZATION)
      {
        if (sterilizeWaitTimer_)
        {
          boost::system::error_code ec;
          sterilizeWaitTimer_->cancel();
          sterilizeWaitTimer_.reset();
          // sterilizationSteps_[stepCount_].waitingDuration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - \
          // sterilizationSteps_[stepCount_].waitingStartSec).count();
          SPDLOG_INFO("ScheduleOrder cancel timer ");
        }
        nlohmann::json infraryJson;
        if (isSterilizeOrderReachedStation_)
        {
          infraryJson["isHiveCanSterilize"] = false;
          isSterilizeOrderReachedStation_ = false;
          if (auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>())
          {
            deviceUtility->useInfraryToSendMsg(infraryJson.dump());
          }
          SPDLOG_INFO("ScheduleOrder send: {}", infraryJson.dump());
        }
      }
      return ;
    }

    void ScheduleOrder::updateOrderProgressAtFailedState(const std::shared_ptr<CommandVertex>& progressInfo)
    {
      return;
    }

    void ScheduleOrder::updateOrderProgressAtExpiredState(const std::shared_ptr<CommandVertex>& progressInfo)
    {
      return;
    }

    void ScheduleOrder::updateOrderProgressAtPendingState(const std::shared_ptr<CommandVertex>& progressInfo)
    {
      if (progressInfo->hiveOrderSetId().empty() && progressInfo->orderId().empty())
      {
        return;
      }
      if (progressInfo->switchFloor())
      {
        return;
      }

      if ("PICK_UP" == progressInfo->commandType() && CommandState::COMMAND_COMPLETED == progressInfo->commandState()
          && progressInfo->stationId() == sourceStationId_)
      {
        orderState_ = OrderState::DROPPING_OFF;
      }
      else if ("MOVE" == progressInfo->commandType() && CommandState::COMMAND_COMPLETED == progressInfo->commandState()
          && progressInfo->stationId() == sourceStationId_)
      {
        sourceArrived_ = true;
        if (isOrderShouldPickHive() || orderType_ == OrderType::ROBOT_CHARGE)
        {
          orderState_ = OrderState::MOUNTING;
        }
        else if (orderType_ == OrderType::ROBOT_RETURN)
        {
          orderState_ = OrderState::COMPLETED;
        }
      }
      else if (CommandState::COMMAND_STARTUP == progressInfo->commandState())
      {
        if (progressInfo->stationId() == sourceStationId_ && !hivePicked_)
        {
          orderState_ = OrderState::PICKING_UP;
        }
        else if (progressInfo->stationId() == targetStationId_ || hivePicked_)
        {
          orderState_ = OrderState::DROPPING_OFF;
        }
      }
      return;
    }

    void ScheduleOrder::updateOrderProgressAtWaitingState(const std::shared_ptr<CommandVertex>& progressInfo)
    {
      if (!progressInfo->stationId().empty() && progressInfo->stationId() != targetStationId_)
      {
        orderState_ = OrderState::CANCELLED;
        if (waitingPollTimer_)
        {
          boost::system::error_code ec;
          waitingPollTimer_->cancel(ec);
          waitingPollTimer_.reset();
        }
        SPDLOG_INFO("ScheduleOrder update Order at {}", __func__);
      }
      return;
    }

    void ScheduleOrder::updateOrderProgressAtCancelledState(const std::shared_ptr<CommandVertex>& progressInfo)
    {
      return;
    }

    void ScheduleOrder::updateOrderProgressAtCompletedState(const std::shared_ptr<CommandVertex>& progressInfo)
    {
      return;
    }

    void ScheduleOrder::updateOrderProgressAtPickingUpState(const std::shared_ptr<CommandVertex>& progressInfo)
    {
      if (progressInfo->hiveOrderSetId().empty() && progressInfo->orderId().empty())
      {
        return;
      }
      if (progressInfo->switchFloor())
      {
        return;
      }
      if ("MOVE" == progressInfo->commandType() && CommandState::COMMAND_COMPLETED == progressInfo->commandState()
          && progressInfo->stationId() == sourceStationId_)
      {
        sourceArrived_ = true;
        if (isOrderShouldPickHive() || orderType_ == OrderType::ROBOT_CHARGE)
        {
          orderState_ = OrderState::MOUNTING;
        }
        else if (orderType_ == OrderType::ROBOT_RETURN)
        {
          orderState_ = OrderState::COMPLETED;
        }
      }
    }

    void ScheduleOrder::updateOrderProgressAtMountingState(const std::shared_ptr<CommandVertex>& progressInfo)
    {
      if (progressInfo->hiveOrderSetId().empty() && progressInfo->orderId().empty())
      {
        return;
      }
      if ("PICK_UP" == progressInfo->commandType() && CommandState::COMMAND_COMPLETED == progressInfo->commandState()
          && progressInfo->stationId() == sourceStationId_)
      {
        orderState_ = OrderState::DROPPING_OFF;
      }
    }

    void ScheduleOrder::updateOrderProgressAtDroppingOffState(const std::shared_ptr<CommandVertex>& progressInfo)
    {
      if (progressInfo->hiveOrderSetId().empty() && progressInfo->orderId().empty() && progressInfo->orderId() != id_)
      {
        return ;
      }
      if (progressInfo->switchFloor())
      {
        return;
      }
      SPDLOG_INFO("{} - [{}]", progressInfo->toJson().dump(),targetStationId_);
      nlohmann::json infraryJson;
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      if ("MOVE" == progressInfo->commandType() && CommandState::COMMAND_COMPLETED == progressInfo->commandState()
          && HIVE_STERILIZATION == orderType_ && progressInfo->stationId() == sterilizationSteps_[stepCount_].waypointId)
      {
        sterilizationSteps_[stepCount_].reached = true;
        if (sterilizationSteps_[stepCount_].waitingDuration)
        {
          auto ioService = cti::missionSchedule::common::getContainer()->resolveOrNull<boost::asio::io_service>();
          sterilizeWaitTimer_ = std::make_shared<boost::asio::deadline_timer>(*ioService);
          sterilizeWaitTimer_->expires_from_now(boost::posix_time::seconds(sterilizationSteps_[stepCount_].waitingDuration));
          sterilizeWaitTimer_->async_wait([self = shared_from_this()](const auto& ec)
                                          { self->sterilizeWaitCallback(ec); });
          sterilizationSteps_[stepCount_].waitingStartSec = std::chrono::system_clock::now();
          infraryJson["isHiveCanSterilize"] = true;
          isSterilizeOrderReachedStation_ = true;
          SPDLOG_INFO("ScheduleOrder reached station and waiting > 0,send: {}",infraryJson.dump());
        }
        else
        {
          sterilizationSteps_[stepCount_].reached = true;
          sterilizationSteps_[stepCount_].finished = true;
          if (isSterilizeOrderReachedStation_)
          {
            isSterilizeOrderReachedStation_ = false;
            infraryJson["isHiveCanSterilize"] = false;
            SPDLOG_INFO("ScheduleOrder reached station but waiting == 0,send: {}",infraryJson.dump());
          }
          stepCount_++;
        }
        if (!infraryJson.empty())
        {
          deviceUtility->useInfraryToSendMsg(infraryJson.dump());
        }
      }
      else if ("MOVE" == progressInfo->commandType() && (CommandState::COMMAND_FAILED == progressInfo->commandState() || CommandState::COMMAND_DIRECT_FAILED == progressInfo->commandState()) && HIVE_STERILIZATION == orderType_ && progressInfo->stationId() == sterilizationSteps_[stepCount_].waypointId)
      {
        sterilizationSteps_[stepCount_].reached = false;
        sterilizationSteps_[stepCount_].finished = false;
        if (isSterilizeOrderReachedStation_)
        {
          isSterilizeOrderReachedStation_ = false;
          infraryJson["isHiveCanSterilize"] = false;
        }
        if (!infraryJson.empty())
        {
          SPDLOG_INFO("ScheduleOrder move to station failed send: {}",infraryJson.dump());
          deviceUtility->useInfraryToSendMsg(infraryJson.dump());
        }
        SPDLOG_WARN("ScheduleOrder move fault steps[{}]", stepCount_);
        stepCount_++;
      }
      else if ("MOVE" == progressInfo->commandType() && CommandState::COMMAND_COMPLETED == progressInfo->commandState()
          && progressInfo->stationId() == targetStationId_)
      {
        targetArrived_ = true;
        auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
        deviceUtility->setRobotState(RobotState::IDLE);
        if (isOrderShouldDropHive() || OrderType::ROBOT_CHARGE == orderType_)
        {
          orderState_ = OrderState::DISMOUNTING;
        }
        else if (effectiveArrivalWait_ > 0 && (OrderType::HIVE_COLLECT == orderType_ || OrderType::HIVE_DISPATCH == orderType_))
        {
          orderState_ = OrderState::WAITING;
          if (auto ioService = cti::missionSchedule::common::getContainer()->resolveOrNull<boost::asio::io_service>())
          {
            waitingPollTimer_ = std::make_shared<boost::asio::deadline_timer>(*ioService);
            waitingPollTimer_->expires_from_now(boost::posix_time::milliseconds(effectiveArrivalWait_));
            waitingPollTimer_->async_wait([self = shared_from_this()](const auto& ec) { self->waitingStateCallback(ec); });
          }
        }
        else
        {
          orderState_ = OrderState::COMPLETED;
        }
        infraryJson["type"] = "arrivedStation";
        infraryJson["waypointId"] = targetStationId_;
        SPDLOG_INFO("ScheduleOrder arrived station send: {}", infraryJson.dump());
        deviceUtility->useInfraryToSendMsg(infraryJson.dump());
      }
      else if ("PEEK" == progressInfo->commandType() && CommandState::COMMAND_COMPLETED == progressInfo->commandState()
          && progressInfo->stationId() == targetStationId_ && !progressInfo->qr().empty())
      {
        auto responseJson = progressInfo->data();
        if (responseJson.contains("qrs"))
        {
          peekedQr_ = responseJson.at("qrs").get<std::vector<std::string>>();
          std::sort(peekedQr_.begin(), peekedQr_.end());
          peekedQr_.erase(std::unique(peekedQr_.begin(), peekedQr_.end()), peekedQr_.end());
          peekedQr_.erase(std::remove_if(peekedQr_.begin(), peekedQr_.end(),
            [](const std::string& qr)
            {
              int qrCode = std::stoi(qr);
              return !(qrCode >= 1000 && qrCode <= 10000);
            }), peekedQr_.end());
          for (auto qrIter = peekedQr_.begin(); qrIter != peekedQr_.end(); qrIter++)
          {
            SPDLOG_INFO("ScheduleOrder Peek command detect qr {}", *qrIter);
          }
          auto stationQr = std::find(peekedQr_.begin(), peekedQr_.end(), progressInfo->qr());
          if (stationQr != peekedQr_.end())
          {
            if (peekedQr_.size() >= 3 || peekedQr_.size() == 0 || (peekedQr_.size() == 2 && hivePicked_))
            {
              peekQrCount_++;
              if (peekQrCount_ >= 3)
              {
                orderState_ = OrderState::FAILED;
              }
            }
            else
            {
              orderState_ = OrderState::DISMOUNTING;
            }
          }
          else
          {
            peekQrCount_++;
            if (peekQrCount_ >= 3)
            {
              orderState_ = OrderState::FAILED;
            }
          }
        }
      }
    }

    void ScheduleOrder::updateOrderProgressAtDismountingState(const std::shared_ptr<CommandVertex>& progressInfo)
    {
      if (progressInfo->hiveOrderSetId().empty() && progressInfo->orderId().empty() && progressInfo->orderId() != id_)
      {
        return ;
      }
      if ("DROP_OFF" == progressInfo->commandType() && CommandState::COMMAND_COMPLETED == progressInfo->commandState()
          && progressInfo->stationId() == targetStationId_)
      {
        orderState_ = OrderState::COMPLETED;
      }
      else if ("CHARGE" == progressInfo->commandType() && CommandState::COMMAND_COMPLETED == progressInfo->commandState()
          && progressInfo->stationId() == targetStationId_ && (OrderType::HIVE_CHARGE == orderType_ || OrderType::HIVE_FORCE_CHARGE == orderType_))
      {
        orderState_ = OrderState::COMPLETED;
      }
      else if ("PICK_UP_CHARGE" == progressInfo->commandType() && CommandState::COMMAND_COMPLETED == progressInfo->commandState()
          && progressInfo->stationId() == targetStationId_ && OrderType::ROBOT_CHARGE == orderType_)
      {
        orderState_ = OrderState::COMPLETED;
      }
      else if ("CHARGE" == progressInfo->commandType() && CommandState::COMMAND_COMPLETED == progressInfo->commandState()
          && progressInfo->stationId() == targetStationId_ && OrderType::ROBOT_CHARGE == orderType_)
      {
        orderState_ = OrderState::COMPLETED;
        auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
        deviceUtility->setRobotState(RobotState::CHARGING);
      }
    }

    std::string ScheduleOrder::calculateBestTargetStationId()
    {
      std::string returnWaypointId;
      StationModel destinationWaypoint;
      std::vector<StationModel> fallbackWaypoints;
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      auto robotInfo = deviceUtility->getRobotInfo();
      bool destinationDodge = false;
      returnWaypointId = targetStationId_;
      for (auto fallbackWaypointId : fallbackWaypointId_)
      {
        StationModel fallbackWaypoint;
        if (deviceUtility->getWaypointInfo(fallbackWaypointId, fallbackWaypoint))
        {
          fallbackWaypoints.push_back(fallbackWaypoint);
        }
      }
      if (deviceUtility->getWaypointInfo(targetStationId_, destinationWaypoint))
      {
        deviceUtility->foreachSystemRobot([fallbackWaypoints, destinationWaypoint, robotInfo,
                                            &destinationDodge](std::shared_ptr<RobotUtility> robot)
        {
          if (robot->robotId() == robotInfo->robotId())
          {
            return false;
          }
          // The other robot is in the destination area and fallback points is not empty
          if (!fallbackWaypoints.empty() && destinationWaypoint.floor() == robot->currentFloor() &&
             destinationWaypoint.isInsideOccupyArea(robot->localDestination()))
          {
            destinationDodge = true;
            return true;
          }
          if (!fallbackWaypoints.empty() && destinationWaypoint.floor() == robot->currentFloor() &&
             destinationWaypoint.isInsideOccupyArea(robot->platformDestination()))
          {
            destinationDodge = true;
            return true;
          }
          if (!fallbackWaypoints.empty() && destinationWaypoint.floor() == robot->currentFloor() &&
             destinationWaypoint.isInsideOccupyArea(robot->location()))
          {
            destinationDodge = true;
            return true;
          }
          return false;
        });
      }
      if (!destinationDodge)
      {
        return returnWaypointId;
      }
      // Destination dodge and fallback waypoint is not empty, returnWaypoint is fallbackWaypoint.
      for (auto fallbackWaypoint : fallbackWaypoints)
      {
        bool fallbackWaypointDodge = false;
        deviceUtility->foreachSystemRobot([robotInfo, fallbackWaypoint, &fallbackWaypointDodge](std::shared_ptr<RobotUtility> robot)
        {
          if (robot->robotId() == robotInfo->robotId())
          {
            return false;
          }
          if (fallbackWaypoint.floor() == robot->currentFloor() && fallbackWaypoint.isInsideOccupyArea(robot->localDestination()))
          {
            fallbackWaypointDodge = true;
            return true;
          }
          if (fallbackWaypoint.floor() == robot->currentFloor() && fallbackWaypoint.isInsideOccupyArea(robot->platformDestination()))
          {
            fallbackWaypointDodge = true;
            return true;
          }
          if (fallbackWaypoint.floor() == robot->currentFloor() && fallbackWaypoint.isInsideOccupyArea(robot->location()))
          {
            fallbackWaypointDodge = true;
            return true;
          }
          return false;
        });
        if (!fallbackWaypointDodge)
        {
          returnWaypointId = fallbackWaypoint.id();
          break;
        }
      }
      return returnWaypointId;
    }

    nlohmann::json ScheduleOrder::computeOrderExecuteCommand()
    {
      SPDLOG_INFO("ScheduleOrder {} compute command at {}", id_, __func__);
      std::unique_lock<std::shared_timed_mutex> lk(mutex_);
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      hivePicked_ = deviceUtility->isHiveAttachedOnRobot();
      if (orderState_ == OrderState::PENDING)
      {
        if ((!hivePicked_ || (hivePicked_ && hiveId_ != deviceUtility->getRobotInfo()->hiveAttachedOnRobot()))
            && orderScheduleType_ == "PLATFORM" && shouldPickHive_)
        {
          orderState_ = OrderState::PICKING_UP;
          // auto robotInfo = deviceUtility->getRobotInfo();
          // if (robotInfo->robotState() == RobotState::CHARGING && !robotInfo->currentStation().expired())
          // {
          //   auto station = robotInfo->currentStation().lock();
          //   if (station->id() == sourceStationId_)
          //   {
          //     orderState_ = OrderState::MOUNTING;
          //   }
          // }
        }
        else
        {
          orderState_ = OrderState::DROPPING_OFF;
        }
      }

      if (std::chrono::system_clock::now() < pauseCommandExpireSec_)
      {
        return nlohmann::json::object();
      }

      if (!commandSequence_.empty())
      {
        if (commandSequence_.find(orderState_) != commandSequence_.end())
        {
          // if (HIVE_STERILIZATION == orderType_ && orderState_ == OrderState::DROPPING_OFF && ((stepCount_ + 1) < sterilizationSteps_.size()))
          // {
          //   StationModel targetStation;
          //   auto robotInfo = deviceUtility->getRobotInfo();
          //   if (deviceUtility->getWaypointInfo(targetStationId_, targetStation) &&
          //       RobotState::BUSY == robotInfo->robotState())
          //   {
          //     if (targetStation.floor() == robotInfo->currentFloor() &&
          //         robotInfo->location().distanceOf(targetStation.coordinate()) < 1.0f)
          //     {
          //       stepCount_++;
          //       nlohmann::json command = computeCommandHandler_[orderState_]();
          //       command["skipSterilizationMovement"] = true;
          //       return command;
          //     }
          //   }
          // }
          if (!isCommandStateOnFinished(commandSequence_[orderState_]->commandState()))
          {
            if (orderState_ == OrderState::DROPPING_OFF)
            {
              if (calculateBestTargetStationId() != targetStationId_)
              {
                nlohmann::json command = computeCommandHandler_[orderState_]();
                if (!orderTags_.empty() && !command.empty())
                {
                  command["orderTags"] = nlohmann::json::array();
                  for (auto tagIter = orderTags_.begin(); tagIter != orderTags_.end(); tagIter++)
                  {
                    command["orderTags"].push_back(*tagIter);
                  }
                }
                return command;
              }
            }
            return nlohmann::json::object();
          }
        }
      }

      nlohmann::json command = computeCommandHandler_[orderState_]();
      if (!orderTags_.empty() && !command.empty())
      {
        command["orderTags"] = nlohmann::json::array();
        for (auto tagIter = orderTags_.begin(); tagIter != orderTags_.end(); tagIter++)
        {
          command["orderTags"].push_back(*tagIter);
        }
      }
      return command;
    }

    nlohmann::json ScheduleOrder::computeOrderCommandAtFailedState()
    {
      SPDLOG_INFO("ScheduleOrder {} compute command at {}", id_, __func__);
      return nlohmann::json::object();
    }

    nlohmann::json ScheduleOrder::computeOrderCommandAtExpiredState()
    {
      SPDLOG_INFO("ScheduleOrder {} compute command at {}", id_, __func__);
      return nlohmann::json::object();
    }

    nlohmann::json ScheduleOrder::computeOrderCommandAtPendingState()
    {
      bool pick_up = false;
      SPDLOG_INFO("ScheduleOrder {} compute command at {}", id_, __func__);
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      nlohmann::json command;
      command["orderId"] = id_;
      command["orderState"] = toString(orderState_);
      command["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
      command["scheduleType"] = orderScheduleType_;
      if (!orderContexts_.empty())
      {
        command["contexts"] = orderContexts_;
      }
      if (!hivePicked_ && orderScheduleType_ == "PLATFORM")
      {
        pick_up = true;
        command["waypointId"] = sourceStationId_;
      }
      else
      {
        command["waypointId"] = targetStationId_;
      }
      if (!hiveOrderSetId_.empty())
      {
        command["hiveOrderSetId"] = hiveOrderSetId_;
      }
      StationModel sourceStation;
      deviceUtility->getWaypointInfo(command["waypointId"], sourceStation);
      // if (OrderType::ROBOT_CHARGE == orderType_)
      // {
      //   command["commandType"] = "PEEK";
      // }
      // if (OrderType::ROBOT_CHARGE == orderType_)
      // {
      //   command["commandType"] = "PEEK";
      // }
      if (pick_up)
      {
        if (isOrderShouldPickHive() || OrderType::ROBOT_CHARGE == orderType_)
        {
          if (sourceStation.type() != "DOCK_NORMAL")
          {
            command["horizontalOffset"] = 0.65;
          }
          else
          {
            command["horizontalOffset"] = 0.9;
          }
        }
      }
      else
      {
        if ((isOrderShouldDropHive() || OrderType::ROBOT_CHARGE == orderType_) && sourceStation.type() != "DOCK_NORMAL")
        {
          command["horizontalOffset"] = 0.65;
        }
      }
      return command;
    }

    nlohmann::json ScheduleOrder::computeOrderCommandAtWaitingState()
    {
      SPDLOG_INFO("ScheduleOrder {} compute command at {}", id_, __func__);
      return nlohmann::json::object();
    }

    nlohmann::json ScheduleOrder::computeOrderCommandAtCancelledState()
    {
      SPDLOG_INFO("ScheduleOrder {} compute command at {}", id_, __func__);
      // nlohmann::json command;
      // command["orderId"] = id_;
      // command["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
      // command["commandType"] = "ABORT";
      // command["orderState"] = toString(orderState_);
      return nlohmann::json::object();
    }

    nlohmann::json ScheduleOrder::computeOrderCommandAtCompletedState()
    {
      SPDLOG_INFO("ScheduleOrder {} compute command at {}", id_, __func__);
      return nlohmann::json::object();
    }

    nlohmann::json ScheduleOrder::computeOrderCommandAtPickingUpState()
    {
      SPDLOG_INFO("ScheduleOrder {} compute command at {}", id_, __func__);
      nlohmann::json command;
      command["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
      command["orderId"] = id_;
      command["orderState"] = toString(orderState_);
      command["waypointId"] = sourceStationId_;
      if (!orderContexts_.empty())
      {
        command["contexts"] = orderContexts_;
      }
      if (!hiveOrderSetId_.empty())
      {
        command["hiveOrderSetId"] = hiveOrderSetId_;
      }
      command["scheduleType"] = orderScheduleType_;
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      auto robotInfo = deviceUtility->getRobotInfo();
      if (robotInfo->robotState() == RobotState::CHARGING && hivePicked_ && hiveId_ != deviceUtility->getRobotInfo()->hiveAttachedOnRobot() )
      {
        command["dropInPlace"] = true;
        command["commandType"] = "DROP_OFF";
        command["waypointId"] = robotInfo->currentStation().lock()->id();
        return command;
      }
      StationModel sourceStation;
      deviceUtility->getWaypointInfo(sourceStationId_, sourceStation);
      command["commandType"] = "MOVE";
      // if (OrderType::ROBOT_CHARGE == orderType_)
      // {
      //   command["commandType"] = "PEEK";
      // }
      if (isOrderShouldPickHive())
      {
        if (sourceStation.type() != "DOCK_NORMAL")
        {
          command["horizontalOffset"] = 0.65;
        }
        else
        {
          command["horizontalOffset"] = 0.9;
        }
      }
      return command;
    }

    nlohmann::json ScheduleOrder::computeOrderCommandAtMountingState()
    {
      SPDLOG_INFO("ScheduleOrder {} compute command at {}", id_, __func__);
      nlohmann::json command;
      command["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
      command["orderId"] = id_;
      command["orderState"] = toString(orderState_);
      command["hiveId"] = hiveId_;
      command["waypointId"] = sourceStationId_;
      if (!orderContexts_.empty())
      {
        command["contexts"] = orderContexts_;
      }

      if (!hiveOrderSetId_.empty())
      {
        command["hiveOrderSetId"] = hiveOrderSetId_;
      }
      command["scheduleType"] = orderScheduleType_;
      command["qr"] = hiveQr_;
      command["commandType"] = "PICK_UP";
      // auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      // auto robotInfo = deviceUtility->getRobotInfo();
      // if (robotInfo->robotState() == RobotState::CHARGING && !robotInfo->currentStation().expired())
      // {
      //   command["commandType"] = "ABORT";
      // }
      return command;
    }

    nlohmann::json ScheduleOrder::computeOrderCommandAtDroppingOffState()
    {
      SPDLOG_INFO("ScheduleOrder {} compute command at {}", id_, __func__);
      nlohmann::json command;
      if (orderScheduleType_ == "PLATFORM" && shouldPickHive_ && !hivePicked_)
      {
        orderState_ = OrderState::CANCELLED;
        if (auto missionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>())
        {
          missionCenter->reportOrderProgress(id_, toString(orderState_));
        }
        SPDLOG_INFO("should pick Hive && hive did not pick");
        return command;
      }
      command["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
      command["orderId"] = id_;
      if (!orderContexts_.empty())
      {
        command["contexts"] = orderContexts_;
      }
      if (HIVE_STERILIZATION == orderType_)
      {
        SPDLOG_INFO("steps :{}({}) ", stepCount_, sterilizationSteps_.size());
        if (stepCount_ < sterilizationSteps_.size() && !sterilizationSteps_[stepCount_].reached && !sterilizationSteps_[stepCount_].finished)
        {
          command["taskId"] = taskId_;
          command["stepId"] = sterilizationSteps_[stepCount_].stepId;
          command["waypointId"] = sterilizationSteps_[stepCount_].waypointId;
          targetStationId_ = sterilizationSteps_[stepCount_].waypointId;
        }
        else
        {
          command.clear();
          if (stepCount_ < sterilizationSteps_.size())
          {
            SPDLOG_INFO("reached:{} finished:{}", sterilizationSteps_[stepCount_].reached, sterilizationSteps_[stepCount_].finished);
          }
          else
          {
            orderState_ = OrderState::COMPLETED;
            if (auto missionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>())
            {
              missionCenter->reportOrderProgress(id_, toString(orderState_));
            }
            SPDLOG_INFO("ScheduleOrder Sterilization Order completed all the steps");
          }
          return nlohmann::json();
        }
      }
      else
      {
        std::string oldWaypointId, newWaypointId;
        oldWaypointId = targetStationId_;
        newWaypointId  = calculateBestTargetStationId();
        SPDLOG_INFO("ScheduleOrder compute oldWaypointId: {}, newWaypointId: {}", oldWaypointId, newWaypointId);
        if (oldWaypointId != newWaypointId)
        {
          for (auto itor = fallbackWaypointId_.begin(); itor != fallbackWaypointId_.end();)
          {
            if ((*itor) == newWaypointId)
            {
              fallbackWaypointId_.erase(itor);
              break;
            }
            itor++;
          }
          targetStationId_ = newWaypointId;
          fallbackWaypointId_.push_back(oldWaypointId);
          SPDLOG_INFO("ScheduleOrder change waypointId: {}", newWaypointId);
        }
        command["waypointId"] = newWaypointId;
      }
      command["orderState"] = toString(orderState_);
      if (!hiveOrderSetId_.empty())
      {
        command["hiveOrderSetId"] = hiveOrderSetId_;
      }
      command["scheduleType"] = orderScheduleType_;
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      auto robotInfo = deviceUtility->getRobotInfo();

      if (orderScheduleType_ != "PLATFORM" && robotInfo->robotState() == RobotState::CHARGING && hivePicked_)
      {
        command["dropInPlace"] = true;
        command["commandType"] = "DROP_OFF";
        command["waypointId"] = robotInfo->currentStation().lock()->id();
        SPDLOG_INFO("schedule type ({})!= platform && charging && picked", orderScheduleType_);
        return command;
      }
      StationModel targetStation;
      deviceUtility->getWaypointInfo(targetStationId_, targetStation);
      command["commandType"] = "MOVE";
      // if (OrderType::ROBOT_CHARGE == orderType_)
      // {
      //   command["commandType"] = "PEEK";
      // }
      if ((isOrderShouldDropHive() || OrderType::ROBOT_CHARGE == orderType_) && targetStation.type() != "DOCK_NORMAL")
      {
        command["horizontalOffset"] = 0.65;
      }
      return command;
    }

    nlohmann::json ScheduleOrder::computeOrderCommandAtDismountingState()
    {
      SPDLOG_INFO("ScheduleOrder {} compute command at {}", id_, __func__);
      nlohmann::json command;
      if (shouldPickHive_ && !hivePicked_)
      {
        orderState_ = OrderState::COMPLETED;
        if (auto missionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>())
        {
          missionCenter->reportOrderProgress(id_, toString(orderState_));
        }
        return command;
      }

      command["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
      command["orderId"] = id_;
      command["orderState"] = toString(orderState_);
      if (!orderContexts_.empty())
      {
        command["contexts"] = orderContexts_;
      }
      if (!hiveId_.empty())
      {
        command["hiveId"] = hiveId_;
      }
      command["waypointId"] = targetStationId_;
      if (!hiveOrderSetId_.empty())
      {
        command["hiveOrderSetId"] = hiveOrderSetId_;
      }
      command["chargingContinueFlag"] = false;
      command["scheduleType"] = orderScheduleType_;
      if (OrderType::ROBOT_CHARGE == orderType_)
      {
        auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
        auto station = deviceUtility->getStationInfo(targetStationId_);
        // auto stationQrIter = std::remove(peekedQr_.begin(), peekedQr_.end(), station->qr());
        // peekedQr_.erase(std::remove(std::begin(peekedQr_), std::end(peekedQr_), station->qr()), std::end(peekedQr_));
        // if (peekedQr_.empty())
        // else if (peekedQr_.size() == 1 && !hivePicked_)
        // std::string hiveId = deviceUtility->getHiveIdByHiveQr(peekedQr_[0]);

        if (station->hiveOccupyThisStation().empty())
        {
          command["commandType"] = "CHARGE";
        }
        else if (!station->hiveOccupyThisStation().empty() && !hivePicked_)
        {
          HiveUtility hiveInfo;
          std::string hiveId = station->hiveOccupyThisStation();
          deviceUtility->getHiveInfo(hiveId, hiveInfo);
          if (!hiveId.empty())
          {
            command["hiveId"] = hiveId;
          }
          SPDLOG_WARN("ScheduleOrder compute pick_up command hiveQr {}, hiveId {}", hiveInfo.qr, hiveId);
          command["qr"] = hiveInfo.qr;
          command["chargingContinueFlag"] = true;
          command["commandType"] = "PICK_UP";
          robotChargingContinue_ = true;
        }
        else
        {
          command["commandType"] = "CHARGE";
          command["ignoreHiveChargingStatus"] = true;
        }
      }
      else if (OrderType::HIVE_CHARGE == orderType_ || OrderType::HIVE_FORCE_CHARGE == orderType_)
      {
        command["commandType"] = "CHARGE";
      }
      else
      {
        command["commandType"] = "DROP_OFF";
      }
      return command;
    }
  }
}
