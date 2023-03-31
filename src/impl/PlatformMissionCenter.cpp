#include "deviceMiscellaneous/PlatformMissionCenter.h"
#include <vector>

namespace cti
{
  namespace missionSchedule
  {
    std::shared_ptr<CommandVertex> CommandVertex::parseFromJson(const nlohmann::json& json)
    {
      if (!json.is_object())
      {
        return nullptr;
      }
      auto item = std::make_shared<CommandVertex>();
      if (json.contains("id") && json["id"].is_string())
      {
        item->id_ = json["id"].get<std::string>();
      }
      if (json.contains("qr") && json["qr"].is_string())
      {
        item->qr_ = json["qr"].get<std::string>();
      }
      if (json.contains("hiveQr") && json["hiveQr"].is_string())
      {
        item->hiveQr_ = json["hiveQr"].get<std::string>();
      }
      if (json.contains("horizontalOffset") && json["horizontalOffset"].is_number_float())
      {
        item->horizontalOffset_ = json["horizontalOffset"].get<double>();
      }
      if (json.contains("coordinates") && json["coordinates"].is_array())
      {
        item->coordinate_.parseFromJson(json.at("coordinates"));
      }
      if (json.contains("hiveOrderSetId") && json["hiveOrderSetId"].is_string())
      {
        item->hiveOrderSetId_ = json["hiveOrderSetId"].get<std::string>();
      }
      if (json.contains("orderId") && json["orderId"].is_string())
      {
        item->orderId_ = json["orderId"].get<std::string>();
      }
      if (json.contains("reason") && json["reason"].is_string())
      {
        item->reason_ = json["reason"].get<std::string>();
      }
      if (json.contains("contexts") && json["contexts"].is_array())
      {
        item->contexts_ = json["contexts"];
      }
      // if (json.contains("withBox") && json["withBox"].is_boolean())
      // {
      //   item->withBox_ = json["withBox"].get<bool>();
      // }

      if (json.contains("dropInPlace") && json["dropInPlace"].is_boolean())
      {
        item->dropInPlace_ = json["dropInPlace"].get<bool>();
      }
      if (json.contains("ignoreHiveChargingStatus") && json["ignoreHiveChargingStatus"].is_boolean())
      {
        item->ignoreHiveChargingStatus_ = json["ignoreHiveChargingStatus"].get<bool>();
      }
      if (json.contains("chargingContinueFlag") && json["chargingContinueFlag"].is_boolean())
      {
        item->chargingContinueFlag_ = json["chargingContinueFlag"].get<bool>();
      }
      if (json.contains("waypointId") && json["waypointId"].is_string())
      {
        item->stationId_ = json["waypointId"].get<std::string>();
      }
      if (json.contains("commandType") && json["commandType"].is_string())
      {
        item->commandType_ = json["commandType"].get<std::string>();
      }
      if (json.contains("scheduleType") && json["scheduleType"].is_string())
      {
        item->scheduleType_ = json["scheduleType"].get<std::string>();
      }
      if (json.contains("switchFloor") && json["switchFloor"].is_boolean())
      {
        item->switchFloor_ = json["switchFloor"].get<bool>();
      }
      if (json.contains("commandState") && json["commandState"].is_string())
      {
        auto state = json["commandState"].get<std::string>();
        if ("QUEUEING" == state)
        {
          item->commandState_ = CommandState::COMMAND_QUEUEING;
        }
        else if ("STARTUP" == state)
        {
          item->commandState_ = CommandState::COMMAND_STARTUP;
        }
        else if ("COMPLETED" == state)
        {
          item->commandState_ = CommandState::COMMAND_COMPLETED;
        }
        else if ("CANCELLED" == state)
        {
          item->commandState_ = CommandState::COMMAND_CANCELLED;
        }
        else if ("FAILED" == state)
        {
          item->commandState_ = CommandState::COMMAND_FAILED;
        }
        else if ("DIRECT_FAILED" == state)
        {
          item->commandState_ = CommandState::COMMAND_DIRECT_FAILED;
        }
      }
      if (json.contains("orderState") && json["orderState"].is_string())
      {
        item->orderState_ = json["orderState"].get<std::string>();
      }

      item->data_ = json;
      return item;
    }

    nlohmann::json HiveOrderSet::toJson()
    {
      std::shared_lock<std::shared_timed_mutex> lk(mutex_);
      nlohmann::json orderSetJson;
      orderSetJson["hiveId"] = hiveId_;
      orderSetJson["transitRequests"] = nlohmann::json::array();
      for (auto orderIter = deliverOrders_.begin(); orderIter != deliverOrders_.end(); orderIter++)
      {
        orderSetJson["transitRequests"].push_back(orderIter->second->toJson());
      }
      return orderSetJson;
    }

    bool HiveOrderSet::isAllOrderFinished()
    {
      std::shared_lock<std::shared_timed_mutex> lk(mutex_);
      for (auto orderIter = deliverOrders_.begin(); orderIter != deliverOrders_.end(); orderIter++)
      {
        if (!isOrderStateOnFinished(orderIter->second->orderState()))
        {
          return false;
        }
      }
      return true;
    }

    size_t HiveOrderSet::getHiveScheduleOrderSize()
    {
      std::shared_lock<std::shared_timed_mutex> lk(mutex_);
      return deliverOrders_.size();
    }

    void HiveOrderSet::addOrderFirstObserveTime(std::string order_id)
    {
      if (orderFirstObserveTimeMap_.find(order_id) != orderFirstObserveTimeMap_.end())
      {
        SPDLOG_INFO("HiveOrderSet : key({}) of orderFirstObserveTimeMap_ conflict!",order_id);
        return ;
      }
      SPDLOG_INFO("HiveOrderSet: orderFirstObserveTimeMap_ add {}", order_id);
      orderFirstObserveTimeMap_.insert(std::pair<std::string, std::chrono::system_clock::time_point>(order_id, std::chrono::system_clock::now()));
    }

    void HiveOrderSet::delOrderFirestObserveTime(std::string order_id)
    {
      if (orderFirstObserveTimeMap_.find(order_id) != orderFirstObserveTimeMap_.end())
      {
        SPDLOG_INFO("HiveOrderSet: orderFirstObserveTimeMap_ delete {}", order_id);
        orderFirstObserveTimeMap_.erase(order_id);
      }
    }

    int HiveOrderSet::getDurationOfCurrentTimeAndFirstObserveTime(std::string order_id)
    {
      std::map<std::string, std::chrono::system_clock::time_point>::iterator iter;
      iter = orderFirstObserveTimeMap_.find(order_id);
      if (iter == orderFirstObserveTimeMap_.end())
      {
        SPDLOG_INFO("HiveOrderSet : key({}) of orderFirstObserveTimeMap_ is not existence!", order_id);
        return -1;
      }
      std::chrono::system_clock::time_point currentSec = std::chrono::system_clock::now();
      return std::abs(std::chrono::duration_cast<std::chrono::seconds>(currentSec - iter->second).count());
    }

    std::shared_ptr<ScheduleOrder> HiveOrderSet::pickBetterOrder(std::shared_ptr<ScheduleOrder> first,
                                                                 std::shared_ptr<ScheduleOrder> second)
    {
      // Judge according to whether there is an end
      if (isOrderStateOnFinished(first->orderState()) || isOrderStateOnFinished(second->orderState()))
      {
        if (isOrderStateOnFinished(first->orderState()))
        {
          delOrderFirestObserveTime(first->id());
          return second;
        }
        else
        {
          delOrderFirestObserveTime(second->id());
          return first;
        }
      }

      //-------test log
      SPDLOG_INFO("HiveOrderSet : First id {}, Second id {}", first->id(), second->id());
      //-------

      // Judge according to whether the robot occupies the destination
      bool firstDestinationDodge = false;
      auto firstDestinationId = first->targetStationId();
      StationModel firstDestinationWaypoint, secondDestinationWaypoint;;
      bool secondDestinationDodge = false;
      auto secondDestinationId = second->targetStationId();
      auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      auto robotPtr = deviceUtility->getRobotInfo();   //get this robot info
      bool firstWithWaypoint = deviceUtility->getWaypointInfo(firstDestinationId, firstDestinationWaypoint);
      bool secondWithWaypoint = deviceUtility->getWaypointInfo(secondDestinationId, secondDestinationWaypoint);
      std::vector<std::string> firstFallbackWaypointIds = first->getFallbackWaypointId();
      std::vector<std::string> secondFallbackWaypointIds = second->getFallbackWaypointId();

      if (firstWithWaypoint && secondWithWaypoint)
      {
        deviceUtility->foreachSystemRobot([this, robotPtr,
                                          firstDestinationWaypoint, first, &firstDestinationDodge,
                                          secondDestinationWaypoint, second, &secondDestinationDodge](std::shared_ptr<RobotUtility> robot)
        {
          if (robot->robotId() == robotPtr->robotId())
          {
            return false;
          }
          if (firstDestinationWaypoint.isInsideOccupyArea(robot->location()))  //The other robot is in the first point area
          {
            if (robot->robotState() != RobotState::IDLE)
            {
              firstDestinationDodge = false;
              return true;
            }
            //Record the starting time when the target location is occupied
            this->addOrderFirstObserveTime(first->id());
            firstDestinationDodge = true;
            SPDLOG_INFO("HiveOrderSet: [first] There is other robot( {} ) at the destination( {} ,{} )", robot->robotId(), firstDestinationWaypoint.id() ,firstDestinationWaypoint.name());
            return true;
          }
          if (secondDestinationWaypoint.isInsideOccupyArea(robot->location()))    //The other robot is in the second point area
          {
            if (robot->robotState() != RobotState::IDLE)
            {
              secondDestinationDodge = false;
              return true;
            }
            //Record the starting time when the target location is occupied
            this->addOrderFirstObserveTime(second->id());
            secondDestinationDodge = true;
            SPDLOG_INFO("HiveOrderSet : [second] There is other robot( {} ) at the destination( {} ,{} )", robot->robotId(), secondDestinationWaypoint.id() ,secondDestinationWaypoint.name());
            return true;
          }
          return false;
        });

        for (auto firstFallbackWaypointId : firstFallbackWaypointIds)
        {
          bool fallbackWaypointDodge = false;
          StationModel firstFallbackWaypoint;
          if (deviceUtility->getWaypointInfo(firstFallbackWaypointId, firstFallbackWaypoint))
          {
            deviceUtility->foreachSystemRobot([robotPtr, &fallbackWaypointDodge, &firstDestinationDodge, firstFallbackWaypoint](std::shared_ptr<RobotUtility> robot)
            {
              if (robotPtr->robotId() == robot->robotId())
              {
                return false;
              }
              if (firstDestinationDodge && firstFallbackWaypoint.floor() == robot->currentFloor() &&
                  firstFallbackWaypoint.isInsideOccupyArea(robot->location()))  // fallback waypoint of first is not idle and destination of first dodge
              {
                SPDLOG_INFO("HiveOrderSet : [first] Destination dodge and fallback waypoint ( {} ,{} ) is not idle.", firstFallbackWaypoint.id(), firstFallbackWaypoint.name());
                fallbackWaypointDodge = true;
                return true;
              }
              return false;
            });
          }
          if (!fallbackWaypointDodge)
          {
            firstDestinationDodge = false;
            break;
          }
        }
        for (auto secondFallbackWaypointId : secondFallbackWaypointIds)
        {
          bool fallbackWaypointDodge = false;
          StationModel secondFallbackWaypoint;
          if (deviceUtility->getWaypointInfo(secondFallbackWaypointId, secondFallbackWaypoint))
          {
            deviceUtility->foreachSystemRobot([robotPtr, &fallbackWaypointDodge, &secondDestinationDodge, secondFallbackWaypoint](std::shared_ptr<RobotUtility> robot)
            {
              if (robotPtr->robotId() == robot->robotId())
              {
                return false;
              }
              if (secondDestinationDodge && secondFallbackWaypoint.floor() == robot->currentFloor() &&
                  secondFallbackWaypoint.isInsideOccupyArea(robot->location()))  // fallback waypoint of first is not idle and destination of second dodge
              {
                SPDLOG_INFO("HiveOrderSet : [second] Destination dodge, and fallback waypoint ( {} ,{} ) is not idle.", secondFallbackWaypoint.id(), secondFallbackWaypoint.name());
                secondDestinationDodge = false;
                return true;
              }
              return false;
            });
          }
          if (!fallbackWaypointDodge)
          {
            secondDestinationDodge = false;
            break;
          }
        }

        if (firstDestinationDodge)
        {
          if (robotPtr->currentFloor() != firstDestinationWaypoint.floor())
          {
            return second;
          }
          else if (robotPtr->currentFloor() == firstDestinationWaypoint.floor() && getDurationOfCurrentTimeAndFirstObserveTime(first->id()) > 30)
          {
            return second;
          }
        }
        else if (secondDestinationDodge)
        {
          if (robotPtr->currentFloor() != secondDestinationWaypoint.floor())
          {
            return first;
          }
          else if (robotPtr->currentFloor() == secondDestinationWaypoint.floor() && getDurationOfCurrentTimeAndFirstObserveTime(second->id()) > 30)
          {
            return first;
          }
        }
      }

      if (first->orderState() == OrderState::WAITING)
      {
        return first;
      }
      else if (first->orderState() == OrderState::WAITING)
      {
        return second;
      }

      // Judge according to the order type
      if (orderTypeCompareTable[first->orderType()] < orderTypeCompareTable[second->orderType()])
      {
        return first;
      }
      else if (orderTypeCompareTable[first->orderType()] > orderTypeCompareTable[second->orderType()])
      {
        return second;
      }
      // Judge according to the priority
      if (first->orderPriority() > second->orderPriority())
      {
        return first;
      }
      else if (first->orderPriority() < second->orderPriority())
      {
        return second;
      }

      StationModel firstTarget, secondTarget;
      // auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
      // auto robotPtr = deviceUtility->getRobotInfo();    //this robot info
      if (deviceUtility->getWaypointInfo(first->targetStationId(), firstTarget) &&
          deviceUtility->getWaypointInfo(second->targetStationId(), secondTarget))
      {
        // Judge according to the floor, creat time of order,
        if (firstTarget.floor() == secondTarget.floor())
        {
          if (first->orderCreateSec() > second->orderCreateSec())
          {
            return second;
          }
          return first;
        }
        if (firstTarget.floor() == robotPtr->currentFloor())
        {
          return first;
        }
        else if (secondTarget.floor() == robotPtr->currentFloor())
        {
          return second;
        }
        int firstFloor = std::atoi(firstTarget.floor().data());
        int secondFloor = std::atoi(secondTarget.floor().data());
        if (firstFloor <= secondFloor)
        {
          return first;
        }
      }
      else if (deviceUtility->getWaypointInfo(first->targetStationId(), firstTarget))
      {
        return first;
      }
      return second;
    }

    bool HiveOrderSet::addScheduleOrder(const std::shared_ptr<ScheduleOrder>& order)
    {
      std::unique_lock<std::shared_timed_mutex> lk(mutex_);
      if (!order)
      {
        return false;
      }
      order->hiveOrderSetId() = hiveOrderSetId_;
      if (deliverOrders_.find(order->id()) != deliverOrders_.end())
      {
        return false;
      }
      order->setOrderTags(hiveTags_);
      auto emplaceResult = deliverOrders_.emplace(order->id(), order);
      if (emplaceResult.second)
      {
        insertOrderTable_.push_back(order->id());
      }
      return emplaceResult.second;
    }

    void HiveOrderSet::updateOrderSetProgress(const nlohmann::json& progressJson)
    {
      std::unique_lock<std::shared_timed_mutex> lk(mutex_);
      SPDLOG_INFO("HiveOrderSet update progress from progressJson {}", progressJson.dump());
      for (auto orderIter = deliverOrders_.begin(); orderIter != deliverOrders_.end();)
      {
        orderIter->second->updateOrderProgress(progressJson);
        if (isOrderStateOnFinished(orderIter->second->orderState()))
        {
          insertOrderTable_.erase(std::remove(insertOrderTable_.begin(), insertOrderTable_.end(), orderIter->second->id()), insertOrderTable_.end());
          deliverOrders_.erase(orderIter++);
        }
        else
        {
          orderIter++;
        }
      }
      lastUpdateSec_ = std::chrono::system_clock::now();
    }

    nlohmann::json HiveOrderSet::computeBestDeliverOrderCommand()
    {
      std::unique_lock<std::shared_timed_mutex> lk(mutex_);
      if (deliverOrders_.empty())
      {
        return nlohmann::json();
      }
      std::shared_ptr<ScheduleOrder> bestOrder;
      for (auto orderIdIter = insertOrderTable_.begin(); orderIdIter != insertOrderTable_.end();)
      {
        if (deliverOrders_.find(*orderIdIter) == deliverOrders_.end())
        {
          insertOrderTable_.erase(orderIdIter++);
        }
        else
        {
          if (!bestOrder)
          {
            bestOrder = deliverOrders_[*orderIdIter];
          }
          else
          {
            auto betterOrder = pickBetterOrder(bestOrder, deliverOrders_[*orderIdIter]);

            auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
            StationModel betterDestinationWaypoint;
            auto betterDestinationId = betterOrder->targetStationId();
            if(! deviceUtility->getWaypointInfo(betterDestinationId, betterDestinationWaypoint)) {}
            SPDLOG_INFO("HiveOrderSet : Better order is {} waypointid:({} , {})",betterOrder->id(), betterDestinationWaypoint.id() ,betterDestinationWaypoint.name());

            if (betterOrder != bestOrder)
            {
              bestOrder.reset();
              bestOrder = betterOrder;
            }
          }
          orderIdIter++;
        }
      }
      if (!bestOrder)
      {
        return nlohmann::json::object();
      }
      return bestOrder->computeOrderExecuteCommand();    // get best order command
    }

    std::shared_ptr<ScheduleOrder> HiveOrderSet::getScheduleOrder(const std::string& orderId)
    {
      std::shared_lock<std::shared_timed_mutex> lk(mutex_);
      if (deliverOrders_.find(orderId) == deliverOrders_.end())
      {
        return nullptr;
      }
      return deliverOrders_.at(orderId);
    }

    bool HiveOrderSet::eraseOrder(const std::string& orderId)
    {
      std::shared_lock<std::shared_timed_mutex> lk(mutex_);
      auto orderIter = deliverOrders_.find(orderId);
      if (orderIter == deliverOrders_.end())
      {
        SPDLOG_INFO("{} can find orderId {}", __func__, orderId);
        return false;
      }
      insertOrderTable_.erase(std::remove(insertOrderTable_.begin(), insertOrderTable_.end(), orderId), insertOrderTable_.end());
      orderIter->second.reset();
      deliverOrders_.erase(orderIter);
      return true;
    }

    void HiveOrderSet::foreachScheduleOrder(std::function<bool(std::shared_ptr<ScheduleOrder>)> handler)
    {
      std::unique_lock<std::shared_timed_mutex> lk(mutex_);
      if (!handler)
      {
        return ;
      }

      for (auto& order : deliverOrders_)
      {
        if (handler(order.second))
        {
          break;
        }
      }
    }

    class PlatformMissionCenter : public IPlatformMissionCenter, public std::enable_shared_from_this<PlatformMissionCenter>
    {
      private:
        bool runPlatformSchedule_{true};
        bool orderLocalStorageLoaded_{false};
        nlohmann::json orderSetStoredJson_;
        nlohmann::json lastOrderEvalueateResult_;
        std::string hiveConstraintedId_;
        std::string orderLocalStoragePath_;
        std::shared_timed_mutex mutex_;
        std::shared_ptr<HiveOrderSet> orderSet_;
        ros::Publisher transitUpdatePublisher_;
        ros::Subscriber platformMissionSubscriber_;
        ros::Subscriber hiveOrderConstraintSubscriber_;
        ros::Subscriber platformMissionUpdateSubscriber_;
        ros::ServiceClient platformOrderClient_;
        ros::ServiceClient cloudPlatformDispatchOrderClient_;
        std::shared_ptr<ScheduleOrder> manualOrder_;
        std::vector<std::shared_ptr<CommandVertex>> platformCommands_;
        std::chrono::system_clock::time_point lastDownloadHiveOrderSec_;

        bool isRobotAndHiveOperationModeMatched(const std::string& robotOperationMode, const std::string& hiveOperationMode)
        {
          if (robotOperationMode.empty() || hiveOperationMode.empty())
          {
            return true;
          }
          if (robotOperationMode == hiveOperationMode)
          {
            return true;
          }
          return false;
        }

        bool storePlatformOrdersToLocalStorage(const nlohmann::json& orderSetJson, bool force = false)
        {
          auto jsonDiff = cti::missionSchedule::contrastJson(orderSetStoredJson_, orderSetJson);
          if (!jsonDiff.empty() || force)
          {
            orderSetStoredJson_ = orderSetJson;
            auto content = orderSetJson.dump();
            std::ofstream fs(orderLocalStoragePath_, std::ios::out | std::ios::trunc);
            fs.write(content.data(), content.size());
            fs.flush();
            fs.close();
          }
          return true;
        }

        nlohmann::json readPlatformOrdersFromLocalStorage()
        {
          nlohmann::json orderSetJson;
          try
          {
            std::ifstream fs(orderLocalStoragePath_, std::ios::in);
            fs >> orderSetJson;
            return orderSetJson;
          }
          catch (std::exception& ex)
          {
            SPDLOG_WARN("read local order storage file {} exception - {}", orderLocalStoragePath_, ex.what());
          }
        }

      public:
        PlatformMissionCenter()
        {
        }

        ~PlatformMissionCenter() {}

        bool initialize()
        {
          if (auto nodeHandle = cti::missionSchedule::common::getContainer()->resolveOrNull<ros::NodeHandle>())
          {
            platformOrderClient_ = nodeHandle->serviceClient<cti_msgs::DispathRequest>("/dispath_interface");
            cloudPlatformDispatchOrderClient_ = nodeHandle->serviceClient<cti_msgs::DispathRequest>("/cloud_scheduling_node/dispatch_interface");
            transitUpdatePublisher_ = nodeHandle->advertise<std_msgs::String>("/mission_schedule/transit_update", 10);
            hiveOrderConstraintSubscriber_ = nodeHandle->subscribe("/mission_schedule/hive_constraint", 1, &PlatformMissionCenter::onHiveConstraint, this);
            platformMissionSubscriber_ = nodeHandle->subscribe("/cloud_scheduling_node/platform_mission", 1, &PlatformMissionCenter::onPlatformCommand, this);
            platformMissionUpdateSubscriber_ = nodeHandle->subscribe("/cloud_scheduling_node/transit_platform_update", 1, &PlatformMissionCenter::onPlatformTransitUpdate, this);
          }
          orderLocalStoragePath_ = std::string(::getenv("HOME")) + "/.ros/" PROJECT_NAME + "/orderLocalStorage.json";
        }

        void onPlatformTransitUpdate(const std_msgs::String& msg)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          nlohmann::json transitUpdate = nlohmann::json::parse(msg.data);
          if (!transitUpdate.contains("id") || !transitUpdate.contains("transitState"))
          {
            return;
          }
          SPDLOG_INFO("PlatformMission Center new transit update {}", transitUpdate.dump());
          std::string orderId = transitUpdate.at("id").get<std::string>();
          if (orderSet_)
          {
            auto order = orderSet_->getScheduleOrder(orderId);
            if (order)
            {
              SPDLOG_INFO("PlatformMission Center transit {} update {}", orderId, transitUpdate.dump());
              order->synchronizePlatformUpdate(transitUpdate);
            }
          }
          return;
        }

        void onHiveConstraint(const std_msgs::String& msg)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          hiveConstraintedId_ = msg.data;
          SPDLOG_INFO("PlatformMission receive hive constraint {}", hiveConstraintedId_);
        }

        void onPlatformCommand(const std_msgs::String& msg)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          nlohmann::json command = nlohmann::json::parse(msg.data);
          std::string commandId = command.at("id").get<std::string>();
          SPDLOG_INFO("PlatformMission Center new command {}", msg.data);
          if (command.contains("requestedBy") && "39136da7-56a5-49ff-85b3-967803f4e1ee" == command.at("requestedBy").get<std::string>())
          {
            return ;
          }
          platformCommands_.clear();
          if (command.contains("commandType") && command["commandType"] == "STILL")
          {
            int duration = 10;
            bool still = true;
            bool allowDirectCommand = false;
            if (command.contains("still"))
            {
              still = command.at("still").get<bool>();
            }
            if (command.contains("duration"))
            {
              duration = command.at("duration").get<int>();
            }
            if (command.contains("allowDirectCommand"))
            {
              allowDirectCommand = command.at("allowDirectCommand").get<bool>();
            }
            auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
            deviceUtility->setStillMode(still, allowDirectCommand, duration);
            return ;
          }
          platformCommands_.push_back(CommandVertex::parseFromJson(command));
        }

        bool uploadLocalScheduleOrder(const nlohmann::json& orderJson, std::string& responseBody, std::string& errorMessage) override
        {
          if (!orderJson.contains("transitMode"))
          {
            SPDLOG_INFO("PlatformMission uploadLocalScheduleOrder doesn't contain transitMode parameter!");
            return false;
          }
          cti_msgs::DispathRequest dispatchService;
          dispatchService.request.params = orderJson.dump();
          if (orderJson["transitMode"] == "ROBOT_CHARGE")
          {
            dispatchService.request.request = cti_msgs::DispathRequestRequest::REQUEST_ROBOT_CHARGE;
          }
          else if (orderJson["transitMode"] == "ROBOT_RETURN")
          {
            dispatchService.request.request = cti_msgs::DispathRequestRequest::REQUEST_ROBOT_RETURN;
          }
          else
          {
            SPDLOG_INFO("PlatformMission uploadLocalScheduleOrder contains incorrect transitMode parameter!");
            return false;
          }
          if (platformOrderClient_.call(dispatchService) && !dispatchService.response.success)
          {
            SPDLOG_INFO("PlatformMission uploadLocalScheduleOrder report order failed {}", dispatchService.response.error_message);
          }
          errorMessage = dispatchService.response.error_message;
          responseBody = dispatchService.response.response_body;
          return dispatchService.response.success;
        }

        std::string downloadAvailableResourceFromPlatform(const std::string& hiveId)
        {
          std::string orderStr;
          nlohmann::json orderJson;
          // orderJson["hiveId"] = hiveId;
          orderJson["resourceType"] = "DOCK_CHARGE,DOCK_NORMAL,DOCK_WALL";
          cti_msgs::DispathRequest dispatchService;
          dispatchService.request.params = orderJson.dump();
          dispatchService.request.task_id= hiveId;
          dispatchService.request.request = cti_msgs::DispathRequestRequest::REQUEST_GET_AVAILABLE_RESOURCE_LIST;
          if (!platformOrderClient_.call(dispatchService) || !dispatchService.response.success)
          {
            SPDLOG_INFO("PlatformMission download available resource failed {} with parameter {}", dispatchService.response.error_message, orderJson.dump());
            return orderStr;
          }
          SPDLOG_INFO("PlatformMission download available resource {} with parameter {}", dispatchService.response.response_body, orderJson.dump());
          return dispatchService.response.response_body;
        }

        std::string downloadManualOrdersOfRobotFromPlatform(const std::string& robotId)
        {
          std::string ordersStr;
          nlohmann::json orderJson;
          orderJson["robotId"] = robotId;
          orderJson["sourceType"] = "MANUAL";
          orderJson["transitState"] = "PENDING";
          cti_msgs::DispathRequest dispatchService;
          dispatchService.request.params = orderJson.dump();
          dispatchService.request.request = cti_msgs::DispathRequestRequest::REQUEST_ROBOT_REQUEST_LIST;
          if (!platformOrderClient_.call(dispatchService) || !dispatchService.response.success)
          {
            SPDLOG_INFO("PlatformMission download manual orders failed {} with parameter {}", dispatchService.response.error_message, orderJson.dump());
            return ordersStr;
          }
          SPDLOG_INFO("PlatformMission download manual orders {} with parameter {}", dispatchService.response.response_body, orderJson.dump());
          return dispatchService.response.response_body;
        }

        std::string downloadOrderInformationFromPlatform(const std::string& orderId)
        {
          std::string ordersStr;
          nlohmann::json orderJson;
          orderJson["id"] = orderId;
          // orderJson["transitState"] = "PENDING";
          cti_msgs::DispathRequest dispatchService;
          dispatchService.request.params = orderJson.dump();
          dispatchService.request.request = cti_msgs::DispathRequestRequest::REQUEST_TASK_LIST;
          if (!platformOrderClient_.call(dispatchService) || !dispatchService.response.success)
          {
            SPDLOG_INFO("PlatformMission download hive order update failed {} with parameter {}", dispatchService.response.error_message, orderJson.dump());
            return ordersStr;
          }
          SPDLOG_INFO("PlatformMission download hive order update {} with parameter {}", dispatchService.response.response_body, orderJson.dump());
          return dispatchService.response.response_body;
        }

        std::string downloadHiveOrdersOfAttachedHiveFromPlatform(const std::string& hiveId)
        {
          std::string ordersStr;
          nlohmann::json orderJson;
          orderJson["hiveId"] = hiveId;
          nlohmann::json transitState = nlohmann::json::array();
          transitState.push_back("PENDING");
          orderJson["transitState"] = transitState;
          cti_msgs::DispathRequest dispatchService;
          dispatchService.request.params = orderJson.dump();
          dispatchService.request.request = cti_msgs::DispathRequestRequest::REQUEST_TASK_LIST;
          if (!cloudPlatformDispatchOrderClient_.call(dispatchService) || !dispatchService.response.success)
          {
            SPDLOG_INFO("PlatformMission download hive orders failed {} with parameter {}", dispatchService.response.error_message, orderJson.dump());
            return ordersStr;
          }
          SPDLOG_INFO("PlatformMission download hive orders {} with parameter {}", dispatchService.response.response_body, orderJson.dump());
          return dispatchService.response.response_body;
        }

        std::string downloadHiveOrdersFromPlatform()
        {
          std::string ordersStr;
          nlohmann::json orderJson;
          nlohmann::json transitState = nlohmann::json::array();
          transitState.push_back("PENDING");
          orderJson["transitState"] = transitState;
          cti_msgs::DispathRequest dispatchService;
          dispatchService.request.params = orderJson.dump();
          dispatchService.request.request = cti_msgs::DispathRequestRequest::REQUEST_TASK_LIST;
          if (!cloudPlatformDispatchOrderClient_.call(dispatchService) || !dispatchService.response.success)
          {
            SPDLOG_INFO("PlatformMission download hive orders failed {} with parameter {}", dispatchService.response.error_message, orderJson.dump());
            return ordersStr;
          }
          SPDLOG_INFO("PlatformMission download hive orders {} with parameter {}", dispatchService.response.response_body, orderJson.dump());
          return dispatchService.response.response_body;
        }

        bool confirmHiveOrderOwnershipWithPlatform(const std::string& hiveId)
        {
          cti_msgs::DispathRequest dispatchService;
          dispatchService.request.request = cti_msgs::DispathRequestRequest::REQUEST_OCCUPY_HIVE;
          dispatchService.request.hive_id = hiveId;
          if (!platformOrderClient_.call(dispatchService) || !dispatchService.response.success)
          {
            SPDLOG_INFO("PlatformMission confirm hive order {} failed", hiveId);
            return false;
          }
          SPDLOG_INFO("PlatformMission confirmed hive order {}", hiveId);
          return true;
        }

        bool reportOrderAllocationResult(const std::string& orderAllocationResult) override
        {
          std_msgs::String uploadMsg;
          uploadMsg.data = orderAllocationResult;
          transitUpdatePublisher_.publish(uploadMsg);
          SPDLOG_INFO("PlatformMission report order allocation result {}", uploadMsg.data);
          return true;
        }

        bool reportElevatorAllocationResult(const std::string& orderId,
                                            const std::string& elevatorId,
                                            double cost) override
        {
          std_msgs::String uploadMsg;
          nlohmann::json updateJson;
          updateJson["cost"] = cost;
          updateJson["requestId"] = orderId;
          updateJson["elevatorId"] = elevatorId;
          updateJson["eventType"] = "ELEVATOR_SELECTED";
          uploadMsg.data = updateJson.dump();
          transitUpdatePublisher_.publish(uploadMsg);
          SPDLOG_INFO("PlatformMission report elevator allocation result {}", uploadMsg.data);
          return true;
        }

        bool reportOrderSelectionProgress(const std::string& orderId,
                                          const std::string& waypointId,
                                          const std::string& dockId = "") override
        {
          std_msgs::String uploadMsg;
          nlohmann::json updateJson;
          updateJson["requestId"] = orderId;
          updateJson["eventType"] = "END_WAYPOINT_SELECTED";
          if (waypointId.empty())
          {
            return false;
          }
          updateJson["endWaypointId"] = waypointId;
          if (!dockId.empty())
          {
            updateJson["targetDockId"] = dockId;
          }
          uploadMsg.data = updateJson.dump();
          transitUpdatePublisher_.publish(uploadMsg);
          SPDLOG_INFO("PlatformMission report order progress {}", uploadMsg.data);
          return true;
        }

        bool confirmEndWaypoint(const std::string& orderId, const std::string& waypointId, std::string& responseBody, std::string& errorMessage)
        {
          cti_msgs::DispathRequest dispatchService;
          dispatchService.request.request = cti_msgs::DispathRequestRequest::REQUEST_DETER_DESTINATION;
          if (orderId.empty() || waypointId.empty())
          {
            SPDLOG_INFO("PlatformMission cinfirmEndwayPoint failed orderid:{} waypointId:{} is empty", orderId, waypointId);
            return false;
          }
          dispatchService.request.task_id = orderId;
          dispatchService.request.end_waypoint_id = waypointId;
          if (platformOrderClient_.call(dispatchService) && !dispatchService.response.success)
          {
            errorMessage = dispatchService.response.error_message;
            SPDLOG_INFO("PlatformMission cinfirmEndwayPoint failed {}", dispatchService.response.error_message);
            return false;
          }
          SPDLOG_INFO("PlatformMission confirmEndwayPoint ");
          responseBody = dispatchService.response.response_body;
          return dispatchService.response.success;
        }

        bool reportOrderProgress(const std::string& orderId,
                                 const std::string& progressState,
                                 const std::string& reason = "",
                                 const bool& authenticTarget = false) override
        {
          std_msgs::String uploadMsg;
          nlohmann::json updateJson;
          updateJson["requestId"] = orderId;
          updateJson["eventType"] = "STATE_UPDATED";
          if (authenticTarget)
          {
            updateJson["authenticTarget"] = authenticTarget;
          }
          if (!reason.empty())
          {
            updateJson["reason"] = reason;
          }
          updateJson["transitStateUpdated"] = "BEEN_" + progressState;
          uploadMsg.data = updateJson.dump();
          transitUpdatePublisher_.publish(uploadMsg);
          SPDLOG_INFO("PlatformMission report order progress {}", uploadMsg.data);
          return true;
        }

        std::shared_ptr<HiveOrderSet> parsePlatformOrders(const nlohmann::json& orderJson)
        {
          SPDLOG_INFO("platformMissionCenter create hiveOrderSet");
          if (!orderJson.is_object() || !orderJson.contains("hiveId") || !orderJson.contains("transitRequests"))
          {
            SPDLOG_INFO("platformMissionCenter create hiveOrderSet Failed");
            return nullptr;
          }
          std::vector<std::string> hiveTags;
          if (orderJson.contains("hive") && orderJson["hive"].contains("tags"))
          {
            for (auto tag : orderJson["hive"]["tags"])
            {
              hiveTags.push_back(tag);
            }
          }
          std::string hiveId = orderJson["hiveId"].get<std::string>();
          auto transitRequests = orderJson["transitRequests"];
          auto hiveOrderSetId = boost::uuids::to_string(boost::uuids::random_generator()());
          auto item = std::make_shared<HiveOrderSet>(hiveOrderSetId, hiveId);
          item->hiveTags() = hiveTags;
          for (int orderCount = 0; orderCount < transitRequests.size(); orderCount++)
          {
            auto transitRequest = transitRequests[orderCount];
            transitRequest["orderScheduleType"] = "PLATFORM";
            if (auto scheduleOrder = ScheduleOrder::parse(transitRequest))
            {
              if (item->addScheduleOrder(scheduleOrder))
              {
                SPDLOG_INFO("platformMissionCenter create hiveOrderSet add order {}", scheduleOrder->id());
              }
            }
          }
          item->lastPlatformUpdateSec() = std::chrono::system_clock::now();
          return item;
        }

        bool isPlatformMissionRunning() override
        {
          if (manualOrder_)
          {
            return true;
          }
          if (orderSet_ && orderSet_->isAllOrderFinished())
          {
            bool expireWaitingTime = true;
            std::chrono::system_clock::time_point currentSec = std::chrono::system_clock::now();
            orderSet_->foreachScheduleOrder([currentSec, &expireWaitingTime](std::shared_ptr<ScheduleOrder> order)
            {
              if (std::chrono::duration_cast<std::chrono::seconds>(currentSec- order->lastUpdateSec()).count() < 3)
              {
                expireWaitingTime = false;
                return true;
              }
              return false;
            });
            if (expireWaitingTime)
            {
              orderSet_.reset();
            }
            return !expireWaitingTime;
          }
          else if (orderSet_ || !platformCommands_.empty())
          {
            return true;
          }
          return false;
        }

        bool isCloudDirectCommandEmpty() override
        {
          return platformCommands_.empty();
        }

        nlohmann::json computeAllocationResult(const std::string& hiveId,
                                               const nlohmann::json& hiveOrdersJson,
                                               const std::map<std::string, double>& orderCostMap,
                                               const std::map<std::string, std::string> hiveAllocationReason,
                                               const std::map<std::string, std::string> orderAllocationReason)
        {
          nlohmann::json orderSeclectReportJson;
          orderSeclectReportJson["eventType"] = "ROBOT_SELECTED";
          orderSeclectReportJson["selectedRequestList"] = nlohmann::json::array();
          orderSeclectReportJson["qualifiedRequestList"] = nlohmann::json::array();
          orderSeclectReportJson["disqualifiedRequestList"] = nlohmann::json::array();

          for (auto count = 0; count < hiveOrdersJson.size(); count++)
          {
            if (!hiveOrdersJson[count].contains("hiveId") ||
                !hiveOrdersJson[count].contains("transitRequests") ||
                0 == hiveOrdersJson[count]["transitRequests"].size())
            {
              continue;
            }
            std::string orderHiveId = hiveOrdersJson[count].at("hiveId").get<std::string>();
            if (orderHiveId == hiveId && !hiveId.empty())
            {
              for (int orderCount = 0; orderCount < hiveOrdersJson[count]["transitRequests"].size(); orderCount++)
              {
                nlohmann::json chosenRequestJson;
                chosenRequestJson["requestId"] = hiveOrdersJson[count]["transitRequests"][orderCount]["id"];
                chosenRequestJson["cost"] = orderCostMap.at(hiveId);
                orderSeclectReportJson["selectedRequestList"].push_back(chosenRequestJson);
              }
              continue;
            }
            if (orderCostMap.find(orderHiveId) != orderCostMap.end())
            {
              for (int orderCount = 0; orderCount < hiveOrdersJson[count]["transitRequests"].size(); orderCount++)
              {
                nlohmann::json reportRequestJson;
                std::string orderId = hiveOrdersJson[count]["transitRequests"][orderCount]["id"];
                reportRequestJson["requestId"] = orderId;
                reportRequestJson["cost"] = orderCostMap.at(orderHiveId);
                if (orderCostMap.at(orderHiveId) >= 1000)
                {
                  if (orderAllocationReason.find(orderId) != orderAllocationReason.end())
                  {
                    reportRequestJson["reason"] = orderAllocationReason.at(orderId);
                  }
                  else if (hiveAllocationReason.find(hiveId) != hiveAllocationReason.end())
                  {
                    reportRequestJson["reason"] = hiveAllocationReason.at(orderId);
                  }
                  else
                  {
                    reportRequestJson["reason"] = "请求起点不可达";
                  }
                  orderSeclectReportJson["disqualifiedRequestList"].push_back(reportRequestJson);
                }
                else
                {
                  if (orderAllocationReason.find(orderId) != orderAllocationReason.end())
                  {
                    reportRequestJson["reason"] = orderAllocationReason.at(orderId);
                  }
                  else if (hiveAllocationReason.find(hiveId) != hiveAllocationReason.end())
                  {
                    reportRequestJson["reason"] = hiveAllocationReason.at(orderId);
                  }
                  else
                  {
                    reportRequestJson["reason"] = "竞争失败";
                  }
                  orderSeclectReportJson["qualifiedRequestList"].push_back(reportRequestJson);
                }
              }
            }
            else
            {
              for (int orderCount = 0; orderCount < hiveOrdersJson[count]["transitRequests"].size(); orderCount++)
              {
                nlohmann::json reportRequestJson;
                std::string orderId = hiveOrdersJson[count]["transitRequests"][orderCount]["id"];
                if (orderAllocationReason.find(orderId) != orderAllocationReason.end())
                {
                  reportRequestJson["reason"] = orderAllocationReason.at(orderId);
                }
                else if (hiveAllocationReason.find(hiveId) != hiveAllocationReason.end())
                {
                  reportRequestJson["reason"] = hiveAllocationReason.at(orderId);
                }
                else
                {
                  reportRequestJson["reason"] = "柜体请求缺失组件";
                }
                reportRequestJson["requestId"] = orderId;
                orderSeclectReportJson["disqualifiedRequestList"].push_back(reportRequestJson);
              }
            }
          }

          SPDLOG_INFO("PlatformMissionCenter upload origin json {}", orderSeclectReportJson.dump());
          SPDLOG_INFO("PlatformMissionCenter upload last json {}", lastOrderEvalueateResult_.dump());
          nlohmann::json reportJsonDiff = cti::missionSchedule::contrastJson(lastOrderEvalueateResult_, orderSeclectReportJson);
          if (!reportJsonDiff.empty())
          {
            lastOrderEvalueateResult_ = orderSeclectReportJson;
          }
          SPDLOG_INFO("PlatformMissionCenter upload contrast json {}", reportJsonDiff.dump());
          return reportJsonDiff;
        }

        bool loadOrderLocalStorage() override
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          if (orderLocalStorageLoaded_ || orderSet_)
          {
            orderLocalStorageLoaded_ = true;
            return false;
          }
          orderLocalStorageLoaded_ = !orderLocalStorageLoaded_;
          auto storedOrderJson = readPlatformOrdersFromLocalStorage();
          SPDLOG_INFO("PlatformMissionCenter read local storage {}", storedOrderJson.dump());
          if (storedOrderJson.empty())
          {
            return false;
          }
          auto orderSet = parsePlatformOrders(storedOrderJson);
          if (!orderSet)
          {
            return false;
          }
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          bool hiveAttached = deviceUtility->isHiveAttachedOnRobot();
          orderSet->foreachScheduleOrder([hiveAttached, self = shared_from_this()](std::shared_ptr<ScheduleOrder> order)
          {
            if (hiveAttached)
            {
              auto orderStr = self->downloadOrderInformationFromPlatform(order->id());
              if (orderStr.empty())
              {
                return false;
              }
              nlohmann::json orderJson = nlohmann::json::parse(orderStr);
              order->synchronizePlatformUpdate(orderJson);
            }
            else
            {
              self->reportOrderProgress(order->id(), "CANCELLED", "Robot rebooted");
            }
            return false;
          });
          if (!hiveAttached || orderSet->isAllOrderFinished())
          {
            storePlatformOrdersToLocalStorage(nlohmann::json::object(), true);
            return false;
          }
          orderSet_ = orderSet;
          return true;
        }

        bool allocateManualOrderToRobot() override
        {
          bool getNewManualOrder = false;
          nlohmann::json chosenManualOrder;
          std::map<std::string, std::string> orderRejectReason;
          std::chrono::system_clock::time_point latestOrderSec;
          std::chrono::system_clock::time_point currentSec = std::chrono::system_clock::now();
          std::string chargingStationId;
          if (manualOrder_)
          {
            latestOrderSec = manualOrder_->orderCreateSec();
            if ((std::chrono::duration_cast<std::chrono::seconds>(currentSec - latestOrderSec).count()) % 10)
            {
              return getNewManualOrder;
            }
            SPDLOG_INFO("PlatformMissionCenter during {}",std::chrono::duration_cast<std::chrono::seconds>(currentSec - latestOrderSec).count());
          }
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotInfo = deviceUtility->getRobotInfo();
          if (deviceUtility->getRobotCloudId().empty())
          {
            SPDLOG_INFO("PlatformMissionCenter cloud id is empty");
            return false;
          }
          std::string orderStr = downloadManualOrdersOfRobotFromPlatform(deviceUtility->getRobotCloudId());
          if (orderStr.empty())
          {
            return false;
          }
          nlohmann::json transitRequests  = nlohmann::json::parse(orderStr);
          if (!transitRequests.is_array() || transitRequests.size() == 0)
          {
            return false;
          }
          for (int orderCount = 0; orderCount < transitRequests.size(); orderCount++)
          {
            auto transitRequest = transitRequests[orderCount];
            transitRequest["orderScheduleType"] = "PLATFORM";
            if (auto scheduleOrder = ScheduleOrder::parse(transitRequest))
            {
              int orderLifeDuration = std::chrono::duration_cast<std::chrono::seconds>(currentSec - scheduleOrder->orderCreateSec()).count();
              if (orderLifeDuration > 60)
              {
                orderRejectReason[scheduleOrder->id()] = "请求创建时间已过期";
                continue;
              }
              if (scheduleOrder->orderCreateSec() < latestOrderSec)
              {
                orderRejectReason[scheduleOrder->id()] = "被创建时间更晚的请求覆盖";
                continue;
              }
              if (manualOrder_)
              {
                if (manualOrder_->orderType() == scheduleOrder->orderType())
                {
                  orderRejectReason[scheduleOrder->id()] = "正在执行相同类型请求";
                  continue;
                }
                if (manualOrder_->id() == scheduleOrder->id())
                {
                  orderRejectReason[scheduleOrder->id()] = "正在执行相同id的请求";
                  continue;
                }
              }
              if (OrderType::ROBOT_CHARGE == scheduleOrder->orderType())
              {
                if (RobotState::CHARGING == robotInfo->robotState())
                {
                  orderRejectReason[scheduleOrder->id()] = "正在充电";
                  continue;
                }
                if(!scheduleOrder->targetStationId().empty())
                {
                  //TODO : check whether the endwaypointId is valid
                  chosenManualOrder = transitRequest;
                  latestOrderSec = scheduleOrder->orderCreateSec();
                  getNewManualOrder = true;
                  continue;
                }
                if (robotInfo->hiveAttachedSet() && robotInfo->hiveAttached() && robotInfo->hiveAttachedOnRobot().empty() && scheduleOrder->targetStationId().empty())
                {
                  orderRejectReason[scheduleOrder->id()] = "机器已装载未知柜体,无法选择合适的充电桩";
                  continue;
                }
                if (robotInfo->hiveAttachedSet() && !robotInfo->hiveAttached())
                {
                  auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
                  chargingStationId = missionPlanner->allocateChargingStationToRobot(robotInfo->robotId());
                  if (chargingStationId.empty())
                  {
                    orderRejectReason[scheduleOrder->id()] = "未装载柜体,未找到合适充电桩";
                    continue;
                  }
                  transitRequest["transitState"] = "PENDING";
                  transitRequest["endWaypointId"] = chargingStationId;
                  SPDLOG_INFO("PlatformMissionCenter get manual order {}", transitRequest.dump());
                  chosenManualOrder = transitRequest;
                  latestOrderSec = scheduleOrder->orderCreateSec();
                  getNewManualOrder = true;
                  continue;
                }
                if (robotInfo->hiveAttached() && robotInfo->hiveAttachedOnRobotSet() && !robotInfo->hiveAttachedOnRobot().empty())
                {
                  auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
                  std::string resourceStr = downloadAvailableResourceFromPlatform(deviceUtility->getRobotInfo()->hiveAttachedOnRobot());
                  std::vector<std::string> chargingStationIds;
                  if (!resourceStr.empty())
                  {
                    nlohmann::json resourceJson = nlohmann::json::parse(resourceStr);
                    if (resourceJson.is_object() && resourceJson.contains("DOCK_CHARGE") && resourceJson["DOCK_CHARGE"].is_array() && resourceJson["DOCK_CHARGE"].size())
                    {
                      for (int stationCount = 0; stationCount < resourceJson["DOCK_CHARGE"].size(); stationCount++)
                      {
                        auto stationJson = resourceJson["DOCK_CHARGE"][stationCount];
                        if (stationJson.contains("waypointId") && stationJson["waypointId"].is_string())
                        {
                          SPDLOG_INFO("get stationId: {}", stationJson["waypointId"]);
                          chargingStationIds.push_back(stationJson["waypointId"]);
                        }
                      }
                    }
                    std::string esponseBody, errorMessage;
                    while (!(chargingStationId = missionPlanner->pickBetterChargingStation(robotInfo->robotId(), chargingStationIds)).empty())
                    {
                      bool confirmOk = confirmEndWaypoint(scheduleOrder->id(), chargingStationId, esponseBody, errorMessage);
                      if (!confirmOk)
                      {
                        auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
                        missionPlanner->addBanedStation(chargingStationId);
                        auto stationIter = find(chargingStationIds.begin(), chargingStationIds.end(), chargingStationId);
                        if (stationIter != chargingStationIds.end())
                        {
                          chargingStationIds.erase(stationIter);
                        }
                        reportOrderProgress(scheduleOrder->id(), "FAILED", "确认调度请求终点失败");
                        chargingStationId.clear();
                        continue;
                      }
                      break;
                    }
                  }
                  if (chargingStationId.empty() || deviceUtility->getStationInfo(chargingStationId)->type() != "DOCK_CHARGE")
                  {
                    orderRejectReason[scheduleOrder->id()] = "已装载柜体,未找到合适充电桩";
                    continue;
                  }
                  transitRequest["transitState"] = "PENDING";
                  transitRequest["transitMode"] = "HIVE_CHARGE";
                  transitRequest["endWaypointId"] = chargingStationId;
                  SPDLOG_INFO("PlatformMissionCenter get manual order {}", transitRequest.dump());
                  chosenManualOrder = transitRequest;
                  latestOrderSec = scheduleOrder->orderCreateSec();
                  getNewManualOrder = true;
                }
              }
              else
              {
                orderRejectReason[scheduleOrder->id()] = "暂不支持该类型请求";
                continue;
              }
            }
          }
          for (int orderCount = 0; orderCount < transitRequests.size(); orderCount++)
          {
            auto transitRequest = transitRequests[orderCount];
            auto orderId = transitRequest.at("id").get<std::string>();
            if (!chosenManualOrder.empty() && chosenManualOrder.contains("id") && orderId == chosenManualOrder["id"])
            {
              continue;
            }
            if (orderRejectReason.find(orderId) == orderRejectReason.end())
            {
              reportOrderProgress(orderId, "FAILED", "未知错误原因");
              continue;
            }
            reportOrderProgress(orderId, "FAILED", orderRejectReason.at(orderId));
          }

          if (getNewManualOrder)
          {
            if (manualOrder_ && chosenManualOrder.contains("id") && manualOrder_->id() != chosenManualOrder["id"])
            {
              reportOrderProgress(manualOrder_->id(), "FAILED", "被其他请求覆盖");
              manualOrder_.reset();
            }
            manualOrder_ = ScheduleOrder::parse(chosenManualOrder);
            StationModel waypoint;
            if(!deviceUtility->getWaypointInfo(manualOrder_->targetStationId(), waypoint))
            {
              reportOrderProgress(manualOrder_->id(), "FAILED", "未找到点位的具体信息");
              return false;
            }
            reportOrderSelectionProgress(manualOrder_->id(), waypoint.id(), waypoint.dockId());
          }
          return getNewManualOrder;
        }

        bool allocateHiveOrderToRobot() override
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          if (!runPlatformSchedule_)
          {
            return runPlatformSchedule_;
          }
          // if (allocateManualOrderToRobot() || manualOrder_)
          // {
          //   return true;
          // }
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          if (deviceUtility->isHiveAttachedOnRobot() && deviceUtility->getRobotInfo()->hiveAttachedOnRobot().empty())
          {
            return false;
          }
          if (deviceUtility->isHiveAttachedOnRobot() && deviceUtility->getRobotInfo()->robotState() != RobotState::CHARGING)
          {
            std::string hiveId = deviceUtility->getRobotInfo()->hiveAttachedOnRobot();
            std::string orderStr = downloadHiveOrdersOfAttachedHiveFromPlatform(hiveId);
            if (orderStr.empty())
            {
              return false;
            }
            nlohmann::json hiveOrdersJson = nlohmann::json::parse(orderStr);
            if (hiveOrdersJson.is_array() && hiveOrdersJson.size() && confirmHiveOrderOwnershipWithPlatform(hiveId))
            {
              if (auto orderSet = parsePlatformOrders(hiveOrdersJson[0]))
              {
                if (orderSet->hiveId() == hiveId)
                {
                  orderSet_.reset();
                  orderSet_ = orderSet;
                  return true;
                }
              }
            }
            return false;
          }
          if (deviceUtility->isDevicePowerBad())
          {
            return false;
          }
          std::string ordersStr = downloadHiveOrdersFromPlatform();
          if (ordersStr.empty())
          {
            return false;
          }
          nlohmann::json hiveOrdersJson = nlohmann::json::parse(ordersStr);
          std::vector<double> robotCostVec;
          std::vector<int> hiveOrderAssignment;
          std::map<int, int> chosenOrderIndexMap;
          std::map<int, std::string> orderHiveIdMap;
          std::map<std::string, double> orderCostMap;
          std::vector<std::vector<double>> orderCostMatrix;
          std::map<std::string, std::string> hiveAllocationReason;
          std::map<std::string, std::string> orderAllocationReason;
          std::map<std::string, std::chrono::system_clock::time_point> orderCreateTimeMap;
          if (hiveOrdersJson.is_array() && hiveOrdersJson.size())
          {
            std::string hiveId;
            int robotIndex = 0, chosenOrderIndex = 0;
            auto robotId = deviceUtility->getRobotInfo()->robotId();
            deviceUtility->foreachSystemRobot([&robotIndex, robotId](std::shared_ptr<RobotUtility> robot)
            {
              if (robot->robotId() == robotId)
              {
                return true;
              }
              robotIndex++;
              return false;
            });
            for (auto count = 0; count < hiveOrdersJson.size(); count++)
            {
              bool orderCompletence = false;
              std::string startWaypointId;
              std::string hiveOperationMode;
              std::vector<std::string> hiveTags;
              int topPriorityNumber = std::numeric_limits<int>::max();
              int orderExistTimeCost = 0;
              std::chrono::system_clock::time_point oldestOrderCreateSec = std::chrono::system_clock::now();
              if (!hiveOrdersJson[count].contains("transitRequests") || 0 == hiveOrdersJson[count]["transitRequests"].size())
              {
                continue;
              }
              if (hiveOrdersJson[count].contains("hive"))
              {
                if (hiveOrdersJson[count]["hive"].contains("operationMode"))
                {
                  hiveOperationMode = hiveOrdersJson[count]["hive"].at("operationMode").get<std::string>();
                }
                if (hiveOrdersJson[count]["hive"].contains("tags"))
                {
                  for (auto tag : hiveOrdersJson[count]["hive"]["tags"])
                  {
                    hiveTags.push_back(tag);
                  }
                }
              }
              SPDLOG_INFO("PlatformMissionCenter evaluate hive order {}", hiveOrdersJson[count].dump());
              std::string orderHiveId = hiveOrdersJson[count].at("hiveId").get<std::string>();
              if (!hiveConstraintedId_.empty() &&  orderHiveId != hiveConstraintedId_)
              {
                orderCompletence = false;
                break;
              }

              for (int orderCount = 0; orderCount < hiveOrdersJson[count]["transitRequests"].size(); orderCount++)
              {
                auto order = hiveOrdersJson[count]["transitRequests"][orderCount];

                std::string requestId = hiveOrdersJson[count]["transitRequests"][orderCount]["id"].get<std::string>();
                if (order.contains("startWaypointId") && !order["startWaypointId"].empty() &&
                    order.contains("endWaypointId") && !order["endWaypointId"].empty() && order.contains("hive") &&
                    (!order["hive"].contains("robotId") || order["hive"]["robotId"] == deviceUtility->getRobotCloudId()) &&
                    (!order.contains("robotId") || order["robotId"] == deviceUtility->getRobotCloudId()))
                {
                  orderCompletence = true;
                  if (!isOperationModeMatched(deviceUtility->getRobotInfo()->operationMode(), hiveOperationMode))
                  {
                    orderAllocationReason[requestId] = "机器人与柜体运行模式不一致";
                  }
                  else if (!isTagsMatched(deviceUtility->getRobotInfo()->tags(), hiveTags))
                  {
                    orderAllocationReason[requestId] = "机器人与柜体标签不一致";
                  }
                  if (order.contains("transitMode") && order["transitMode"] == "HIVE_STERILIZATION")
                  {
                    if (!order.contains("steps") || !order["steps"].is_array() || order["steps"].empty())
                    {
                      orderAllocationReason[requestId] = "消毒请求缺少步骤字段";
                      orderCompletence = false;
                      break;
                    }
                  }
                }
                else
                {
                  if (!order.contains("startWaypointId") || order["startWaypointId"].empty())
                  {
                    orderAllocationReason[requestId] = "请求缺少起点";
                  }
                  else if (!order.contains("endWaypointId") || order["endWaypointId"].empty())
                  {
                    orderAllocationReason[requestId] = "请求缺少终点";
                  }
                  else if (order.contains("hive") && order["hive"].contains("robotId"))
                  {
                    orderAllocationReason[requestId] = "请求所属柜体已绑定其他机器人";
                  }
                  else if (order.contains("robotId"))
                  {
                    orderAllocationReason[requestId] = "请求所属柜体已指定其他机器人";
                  }
                  else
                  {
                    orderAllocationReason[requestId] = "未知原因,请联系研发";
                  }
                  orderCompletence = false;
                  break;
                }
              }
              if (!orderCompletence)
              {
                hiveAllocationReason[orderHiveId] = "请求所属柜体包含不符合要求的请求";
                SPDLOG_INFO("Hive {} order check completence failed", orderHiveId);
                continue;
              }

              bool hiveAttached = false;
              if (hiveOrdersJson[count].contains("hive") && hiveOrdersJson[count]["hive"].contains("robotId"))
              {
                hiveAttached = true;
              }

              startWaypointId = hiveOrdersJson[count]["transitRequests"][0]["startWaypointId"].get<std::string>();
              StationModel startWaypoint;
              if (!deviceUtility->getWaypointInfo(startWaypointId, startWaypoint))
              {
                hiveAllocationReason[orderHiveId] = "未知柜体请求起点";
                SPDLOG_INFO("Hive {} order check startWaypoint {} failed", orderHiveId, startWaypointId);
                continue;
              }
              chosenOrderIndexMap[chosenOrderIndexMap.size()] = count;
              orderHiveIdMap[orderHiveIdMap.size()] = orderHiveId;

              orderCreateTimeMap[orderHiveId] = oldestOrderCreateSec;
              for (int orderCount = 0; orderCount < hiveOrdersJson[count]["transitRequests"].size(); orderCount++)
              {
                int priority = 0, orderTypePriority = 10, priorityNumber = std::numeric_limits<int>::max();
                std::chrono::system_clock::time_point orderCreateSec = std::chrono::system_clock::now();
                auto order = hiveOrdersJson[count]["transitRequests"][orderCount];
                if (!order.contains("transitMode"))
                {
                  continue;
                }
                if (order.contains("priority"))
                {
                  priority = order.at("priority").get<int>();
                }
                if (order.contains("createdTime"))
                {
                  auto timeCount = order.at("createdTime").get<int64_t>();
                  std::chrono::milliseconds dur(timeCount);
                  std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> dt(dur);
                  orderCreateSec = dt;
                }
                if (orderCreateSec < oldestOrderCreateSec)
                {
                  oldestOrderCreateSec = orderCreateSec;
                  orderCreateTimeMap[orderHiveId] = oldestOrderCreateSec;
                }
                orderTypePriority = orderTypeCompareTable[ScheduleOrder::convertTransitModeStrToOrderType(order.at("transitMode").get<std::string>())];
                priorityNumber = orderTypePriority * 10 + 10 - priority;
                if (priorityNumber < topPriorityNumber)
                {
                  topPriorityNumber = priorityNumber;
                }
              }
              int orderExistDuration = std::chrono::duration_cast<std::chrono::minutes>(std::chrono::system_clock::now() - oldestOrderCreateSec).count();
              orderExistTimeCost = 50 - orderExistDuration;
              SPDLOG_INFO("hive {} order exist duration cost {}", orderHiveId, orderExistTimeCost);
              topPriorityNumber += orderExistTimeCost;
              robotCostVec.clear();
              deviceUtility->foreachSystemRobot([&robotCostVec, &orderCostMap, orderHiveId, robotId, topPriorityNumber, hiveOperationMode, hiveTags, startWaypoint, deviceUtility, hiveAttached, self = shared_from_this()](std::shared_ptr<RobotUtility> robot)
              {
                double robotCost = topPriorityNumber;
                if (!robot->chassisPowerSet() || !robot->localizedSet() || !robot->localized() || robot->disabled() ||
                    deviceUtility->isDevicePowerBad(robot->robotId()) || (robot->hiveAttached() && robot->robotId() != robotId) ||
                    (robot->robotState() != RobotState::CHARGING && robot->robotState() != RobotState::IDLE &&
                    robot->robotId() != robotId))
                {
                  if (robot->robotId() == robotId)
                  {
                    orderCostMap[orderHiveId] = 10000;
                  }
                  robotCostVec.push_back(10000);
                  return false;
                }
                if (robot->robotId() != robotId && hiveAttached)
                {
                  robotCost += 1000;
                  if (robot->robotId() == robotId)
                  {
                    orderCostMap[orderHiveId] = robotCost;
                  }
                  robotCostVec.push_back(robotCost);
                  return false;
                }
                if (!isOperationModeMatched(robot->operationMode(), hiveOperationMode))
                {
                  robotCost += 1000;
                  if (robot->robotId() == robotId)
                  {
                    orderCostMap[orderHiveId] = robotCost;
                  }
                  robotCostVec.push_back(robotCost);
                  return false;
                }

                if (!isTagsMatched(robot->tags(), hiveTags))
                {
                  robotCost += 1000;
                  if (robot->robotId() == robotId)
                  {
                    orderCostMap[orderHiveId] = robotCost;
                  }
                  robotCostVec.push_back(robotCost);
                  return false;
                }

                robotCost += robot->chassisPower() * 0.1;

                Position startLocation = robot->location();
                std::string startFloor = robot->currentFloor();
                SPDLOG_INFO("hive order assign robot {} compute level from {} to {}", robot->robotId(), startFloor, startWaypoint.floor());
                auto elevatorPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IElevatorPlanner>();
                auto elevatorPlan = elevatorPlanner->calculateBestElevatorPlan(startLocation, startFloor, startWaypoint);
                robotCost += elevatorPlan.cost;

                auto pathPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IPathPlanner>();
                for (auto elevatorStep = elevatorPlan.planSteps.begin(); elevatorStep != elevatorPlan.planSteps.end(); elevatorStep++)
                {
                  if (startFloor == elevatorStep->elevatorFloor)
                  {
                    double pathCost = pathPlanner->calculatePathCost(startLocation, elevatorStep->elevatorLocation, startFloor);
                    robotCost += pathCost;
                  }
                  startLocation = elevatorStep->elevatorLocation;
                  startFloor = elevatorStep->elevatorFloor;
                }
                SPDLOG_INFO("hive order assign robot {} cost value {}", robot->robotId(), robotCost);
                SPDLOG_INFO("hive order assign robot {} compute level from {} to {}", robot->robotId(), startFloor, startWaypoint.floor());
                if (startFloor != startWaypoint.floor())
                {
                  robotCost += 1000;
                }
                else
                {
                  double pathCost = pathPlanner->calculatePathCost(startLocation, startWaypoint.coordinate(), startFloor);
                  robotCost += pathCost;
                }
                SPDLOG_INFO("hive order assign robot {} cost origin value {}", robot->robotId(), robotCost);
                if (robotCost < 0)
                {
                  robotCost = 0;
                }
                SPDLOG_INFO("hive order assign robot {} cost value {}", robot->robotId(), robotCost);
                if (robot->robotId() == robotId)
                {
                  orderCostMap[orderHiveId] = robotCost;
                }
                robotCostVec.push_back(robotCost);
                return false;
              });
              orderCostMatrix.push_back(robotCostVec);
            }
            if (orderCostMatrix.empty())
            {
              nlohmann::json orderSeclectReportJson = computeAllocationResult(hiveId, hiveOrdersJson, orderCostMap, hiveAllocationReason, orderAllocationReason);
              if (!orderSeclectReportJson.empty())
              {
                reportOrderAllocationResult(orderSeclectReportJson.dump());
              }
              return false;
            }
            std::priority_queue<OrderElement> orderCostQueue;
            HungarianAlgorithm::Solve(orderCostMatrix, hiveOrderAssignment);
            for (auto assignmentCount = 0; assignmentCount < hiveOrderAssignment.size(); assignmentCount++)
            {
              OrderElement order;
              std::string assignmentHiveId = orderHiveIdMap[assignmentCount];
              order.hiveId = assignmentHiveId;
              order.orderIndex = chosenOrderIndexMap[assignmentCount];
              order.orderCost = orderCostMap[assignmentHiveId];
              order.orderCreateTime = orderCreateTimeMap[assignmentHiveId];
              orderCostQueue.push(order);
              SPDLOG_INFO("Platform Order allocation result orderIndex {}, robotIndex {}, curentOrderIndex {}", assignmentCount,  hiveOrderAssignment[assignmentCount], robotIndex);
              if (hiveOrderAssignment[assignmentCount] == robotIndex && orderCostMatrix[assignmentCount][robotIndex] < 1000)
              {
                chosenOrderIndex = chosenOrderIndexMap[assignmentCount];
                hiveId = orderHiveIdMap[assignmentCount];
              }
            }
            SPDLOG_INFO("Platform Order allocation hiveId {}", hiveId);
            std::chrono::system_clock::time_point currentSec = std::chrono::system_clock::now();
            while (!orderCostQueue.empty())
            {
              OrderElement order = orderCostQueue.top();
              orderCostQueue.pop();
              if (order.hiveId == hiveId)
              {
                break;
              }
              int orderPendingDuration = std::chrono::duration_cast<std::chrono::seconds>(currentSec - order.orderCreateTime).count();
              if (orderPendingDuration > 60 && order.orderCost < 1000)
              {
                hiveId = order.hiveId;
                chosenOrderIndex = order.orderIndex;
                SPDLOG_INFO("Platform Order Seize hiveId {}", hiveId);
                break;
              }
            }
            SPDLOG_INFO("Platform Order decision hiveId {}", hiveId);
            if (hiveId.empty())
            {
              nlohmann::json orderSeclectReportJson = computeAllocationResult(hiveId, hiveOrdersJson, orderCostMap, hiveAllocationReason, orderAllocationReason);
              if (!orderSeclectReportJson.empty())
              {
                reportOrderAllocationResult(orderSeclectReportJson.dump());
              }
              return false;
            }
            if (confirmHiveOrderOwnershipWithPlatform(hiveId))
            {
              if (auto orderSet = parsePlatformOrders(hiveOrdersJson[chosenOrderIndex]))
              {
                orderSet_.reset();
                orderSet_ = orderSet;
                nlohmann::json orderSeclectReportJson = computeAllocationResult(hiveId, hiveOrdersJson, orderCostMap, hiveAllocationReason, orderAllocationReason);
                if (!orderSeclectReportJson.empty())
                {
                  reportOrderAllocationResult(orderSeclectReportJson.dump());
                }
                return true;
              }
            }
            else
            {
              hiveAllocationReason[hiveId] = "与平台确认请求抢占失败";
              hiveId.clear();
              nlohmann::json orderSeclectReportJson = computeAllocationResult(hiveId, hiveOrdersJson, orderCostMap, hiveAllocationReason, orderAllocationReason);
              if (!orderSeclectReportJson.empty())
              {
                reportOrderAllocationResult(orderSeclectReportJson.dump());
              }
            }
          }
          return false;
        }

        nlohmann::json computeCloudDirectCommand() override
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          if (!platformCommands_.empty())
          {
            for (auto& commandIter : platformCommands_)
            {
              if (commandIter->commandState() == CommandState::COMMAND_QUEUEING)
              {
                return commandIter->data();
              }
            }
          }
          return nlohmann::json::object();
        }

        nlohmann::json computeManualOrderCommand() override
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          allocateManualOrderToRobot();
          if (manualOrder_)
          {
            return manualOrder_->computeOrderExecuteCommand();
          }
          return nlohmann::json::object();
        }

        nlohmann::json computeOrderCommand() override
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          // allocateManualOrderToRobot();
          // if (manualOrder_)
          // {
            // return manualOrder_->computeOrderExecuteCommand();
          // }
          if (orderSet_ && platformCommands_.empty())
          {
            std::chrono::system_clock::time_point currentSec = std::chrono::system_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(currentSec - orderSet_->lastPlatformUpdateSec()).count() > 5 && orderSet_->getHiveScheduleOrderSize())
            {
              orderSet_->foreachScheduleOrder([self = shared_from_this()](std::shared_ptr<ScheduleOrder> order)
              {
                if (isOrderStateOnFinished(order->orderState()))
                {
                  return false;
                }
                auto orderStr = self->downloadOrderInformationFromPlatform(order->id());
                if (orderStr.empty())
                {
                  return false;
                }
                nlohmann::json orderJson = nlohmann::json::parse(orderStr);
                order->synchronizePlatformUpdate(orderJson);
                return false;
              });
              std::string orderStr = downloadHiveOrdersOfAttachedHiveFromPlatform(orderSet_->hiveId());
              if (!orderStr.empty())
              {
                nlohmann::json hiveOrdersJson = nlohmann::json::parse(orderStr);
                if (hiveOrdersJson.is_array() && hiveOrdersJson.size() &&
                    hiveOrdersJson[0].contains("transitRequests") &&
                    confirmHiveOrderOwnershipWithPlatform(orderSet_->hiveId()))
                {
                  auto transitRequests = hiveOrdersJson[0]["transitRequests"];
                  for (int orderCount = 0; orderCount < transitRequests.size(); orderCount++)
                  {
                    auto transitRequest = transitRequests[orderCount];
                    transitRequest["orderScheduleType"] = "PLATFORM";
                    if (auto scheduleOrder = ScheduleOrder::parse(transitRequest))
                    {
                      if (orderSet_->addScheduleOrder(scheduleOrder))
                      {
                        auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
                        if (!scheduleOrder->shouldPickHive() || deviceUtility->isHiveAttachedOnRobot())
                        {
                          scheduleOrder->setOrderState(OrderState::DROPPING_OFF);
                          reportOrderProgress(scheduleOrder->id(), ScheduleOrder::toString(OrderState::DROPPING_OFF));
                        }
                        else
                        {
                          scheduleOrder->setOrderState(OrderState::PICKING_UP);
                          reportOrderProgress(scheduleOrder->id(), ScheduleOrder::toString(OrderState::PICKING_UP));
                        }
                        SPDLOG_INFO("platformMissionCenter hiveOrderSet add order {}", scheduleOrder->id());
                      }
                    }
                  }
                }
                orderSet_->lastPlatformUpdateSec() = std::chrono::system_clock::now();
              }
            }
            if (orderSet_ && orderSet_->isAllOrderFinished())
            {
              return nlohmann::json::object();
            }
            return orderSet_->computeBestDeliverOrderCommand();    // get best command
          }
          else if (!platformCommands_.empty())
          {
            for (auto& commandIter : platformCommands_)
            {
              if (commandIter->commandState() == CommandState::COMMAND_QUEUEING)
              {
                return commandIter->data();
              }
            }
          }
          return nlohmann::json::object();
        }


        bool updatePlatformMissionState(const nlohmann::json& commandResult) override
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          std::string id = commandResult.at("id").get<std::string>();
          auto platformCommand = std::find_if(platformCommands_.begin(), platformCommands_.end(),
            [id](const std::shared_ptr<CommandVertex>& command)
            {
              return command->id() == id;
            });
          if (platformCommand != platformCommands_.end() && commandResult.contains("commandState"))
          {
            std::string commandState = commandResult.at("commandState").get<std::string>();
            if ("COMPLETED" == commandState || "FAILED" == commandState || "CANCELLED" == commandState)
            {
              (*platformCommand).reset();
              platformCommands_.erase(platformCommand);
            }
            else if ("STARTUP" == commandState)
            {
              (*platformCommand)->setCommandState(CommandState::COMMAND_STARTUP);
            }
          }
          if (manualOrder_)
          {
            manualOrder_->updateOrderProgress(commandResult);
            if (isOrderStateOnFinished(manualOrder_->orderState()))
            {
              manualOrder_.reset();
            }
          }
          if (orderSet_)
          {
            orderSet_->updateOrderSetProgress(commandResult);
            storePlatformOrdersToLocalStorage(orderSet_->toJson());
          }
          return true;
        }

        bool isManualOrderEmpty() override
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          if (manualOrder_)
          {
            return false;
          }
          return true;
        }

        bool isDeviceCharging() override
        {
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotInfo = deviceUtility->getRobotInfo();
          if (RobotState::CHARGING == robotInfo->robotState())
          {
            return true;
          }
          return false;
        }

        bool cancelAllPlatformOrders(const std::string& reason) override
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          std::set<std::string> cancelOrderIds;
          std::set<std::string> failureOrderIds;
          if (!orderSet_ || orderSet_->isAllOrderFinished())
          {
            return false;
          }
          orderSet_->foreachScheduleOrder([&cancelOrderIds, &failureOrderIds](std::shared_ptr<ScheduleOrder> order)
          {
            if (!isOrderStateOnFinished(order->orderState()))
            {
              auto orderId = order->id();
              if (order->orderType() == OrderType::HIVE_STERILIZATION && order->hivePicked())
              {
                failureOrderIds.insert(orderId);
              }
              else
              {
                cancelOrderIds.insert(orderId);
              }
            }
            return false;
          });
          for(auto orderId : cancelOrderIds)
          {
            reportOrderProgress(orderId, "CANCELLED", reason);
            orderSet_->eraseOrder(orderId);
            SPDLOG_INFO("platformMissionCenter cancel order {} reason {}",orderId, reason);
          }
          for(auto orderId : failureOrderIds)
          {
            reportOrderProgress(orderId, "FAILED", reason);
            orderSet_->eraseOrder(orderId);
            SPDLOG_INFO("platformMissionCenter cancel order {} reason {}",orderId, reason);
          }
          orderSet_.reset();
          return true;
        }

        bool cancelManualOrder() override
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          if (!manualOrder_)
          {
            return false;
          }
          if (isOrderStateOnFinished(manualOrder_->orderState()))
          {
            return false;
          }
          auto orderId = manualOrder_->id();
          reportOrderProgress(orderId, "CANCELLED", "被平台直接命令覆盖");
          manualOrder_.reset();
          return true;
        }

        bool isSterilizeOrderReachedStation() override
        {
          if (!orderSet_)
          {
            return false;
          }
          SPDLOG_INFO("PlatformMissionCenter orderSet size: {}",orderSet_->getHiveScheduleOrderSize());
          bool isSterilizeOrderReachedStation = false;
          orderSet_->foreachScheduleOrder([&isSterilizeOrderReachedStation](std::shared_ptr<ScheduleOrder> order)
          {
            if (order->orderType() == HIVE_STERILIZATION && order->orderState() == OrderState::DROPPING_OFF)
            {
              isSterilizeOrderReachedStation = order->isSterilizeOrderReachedStation();
              SPDLOG_INFO("PlatformMissionCenter foreach orders robot is {} in sterilize station", isSterilizeOrderReachedStation);
              return isSterilizeOrderReachedStation;
            }
            else
            {
              SPDLOG_INFO("PlatformMissionCenter foreach orders type is {}", order->orderType());
              return false;
            }
          });
          return isSterilizeOrderReachedStation;
        }

        bool cancelHiveSterilize() override
        {
          std::shared_ptr<ScheduleOrder> targetOrder;
          orderSet_->foreachScheduleOrder([&targetOrder](std::shared_ptr<ScheduleOrder> order)
          {
            if (order->orderType() == HIVE_STERILIZATION && order->isSterilizeOrderReachedStation())
            {
              SPDLOG_INFO("PlatformMissionCenter foreach orders pause sterilize and executeManualOrder or executeCloudDirectCommand");
              targetOrder = order;
              return true;
            }
            else
            {
              SPDLOG_INFO("PlatformMissionCenter foreach order type is {}", order->orderType());
            }
            return false;
          });
          if (targetOrder)
          {
            targetOrder->updateSterilizationOrderProgress(false);
          }
          return true;
        }

        bool changeOrderStateToCompleted()
        {
          std::shared_ptr<ScheduleOrder> targetOrder;
          orderSet_->foreachScheduleOrder([&targetOrder](std::shared_ptr<ScheduleOrder> order)
          {
            if ((order->orderType() == HIVE_COLLECT || order->orderType() == HIVE_DISPATCH) && order->orderState() == OrderState::WAITING)
            {
              SPDLOG_INFO("PlatformMissionCenter foreach order find correct order");
              targetOrder = order;
              return true;
            }
            else
            {
              SPDLOG_INFO("PlatformMissionCenter foreach order type is {}", order->orderType());
            }
            return false;
          });
          if (targetOrder)
          {
            targetOrder->cancelWaitingPollTimer();
            reportOrderProgress(targetOrder->id(), targetOrder->toString(OrderState::COMPLETED));
          }
          return true;
        }

        bool executeSterilizeNextStep() override
        {
          if (orderSet_->getHiveScheduleOrderSize() == 0)
          {
            return false;
          }
          std::shared_ptr<ScheduleOrder> targetOrder;
          orderSet_->foreachScheduleOrder([&targetOrder](std::shared_ptr<ScheduleOrder> order)
          {
            if (order->orderType() == HIVE_STERILIZATION && order->isSterilizeOrderReachedStation())
            {
              SPDLOG_INFO("PlatformMissionCenter foreach orders execute sterilize next step");
              targetOrder = order;
              return true;
            }
            else
            {
              SPDLOG_INFO("PlatformMissionCenter foreach order type is {}", order->orderType());
            }
            return false;
          });
          if (targetOrder)
          {
            targetOrder->updateSterilizationOrderProgress(true);
          }
          return true;
        }

        void showAllPlatformOrders() override
        {
          int num = 0;
          if (orderSet_)
          {
            orderSet_->foreachScheduleOrder([&num](std::shared_ptr<ScheduleOrder> order)
            {
              SPDLOG_INFO("Orders:{}:orderId:[{}] ,orderState:[{}] ,orderType:[{}] ,targetStationId:[{}]", num++, order->id(), ScheduleOrder::toString(order->orderState()), ScheduleOrder::toString(order->orderType()), order->targetStationId());
              return false;
            });
          }
        }
    };

    std::shared_ptr<IPlatformMissionCenter> createPlatformMissionCenter()
    {
      auto missionCenter = std::make_shared<PlatformMissionCenter>();
      missionCenter->initialize();
      return std::dynamic_pointer_cast<IPlatformMissionCenter>(missionCenter);
    }
  }
}
