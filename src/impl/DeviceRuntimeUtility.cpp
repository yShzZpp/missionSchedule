#include <fstream>
#include "deviceMiscellaneous/DeviceRuntimeUtility.h"

namespace cti
{
  namespace missionSchedule
  {
    class DeviceRuntimeUtility : public IDeviceRuntimeUtility, public std::enable_shared_from_this<DeviceRuntimeUtility>
    {
      private:
        bool stillMode_{false};
        bool manualMode_{false};
        bool zigbeeModuleFault_{false};
        bool navigationNormalMode_{true};
        bool isDirectedCommandAllowedInStillMode_{false};
        bool pairingState_{false};
        int robotState_;
        int floorHashValue_;
        int buildingHashValue_;
        int robotFaultCount_{0};
        std::string hubId_;
        std::string robotId_;
        std::string buildingId_;
        std::string robotCloudId_;
        Position currentLocation_;
        std::string currentFloor_;
        std::string workSpacePath_;
        std::string hiveAttachedOnRobotFromInfrary_;
        std::string hiveAttachedOnRobotFromCloud_;
        std::string hiveAttachedOnRobotFromLocal_;
        double badPowerThreshold_{10};
        double chargingToBasicPowerThreshold_{30};
        double goodPowerThreshold_{60};
        double nicePowerThreshold_{100};

        std::shared_timed_mutex mutex_;
        std::unordered_map<std::string, HiveUtility> hiveInfo_;
        std::unordered_map<std::string, std::string> hiveQrInfo_;
        std::unordered_map<std::string, StationModel> systemWaypoints_;
        std::map<std::string, std::string> chargingDockWaypointIdMap_;
        std::map<std::string, std::shared_ptr<RobotUtility>> systemInfo_;
        std::unordered_map<std::string, StationDistributionInfo> parkingDistribution_;
        std::unordered_map<std::string, StationDistributionInfo> chargingDistribution_;
        std::unordered_map<std::string, std::shared_ptr<StationModel>> elevatorDodgeStations_;
        std::unordered_map<std::string, std::shared_ptr<StationModel>> parkingStations_;
        std::unordered_map<std::string, std::shared_ptr<StationModel>> chargingStations_;

        std::vector<std::vector<Position>> narrowAreas_;
        std::vector<std::vector<Position>> gateNarrowAreas_;

        ros::Publisher infraryPublisher_;

        ros::Subscriber robotPowerThresholdSubscriber_;
        ros::Subscriber narrowAreaSubscriber_;
        ros::Subscriber robotSensorSubscriber_;
        ros::Subscriber robotStatusSubscriber_;
        ros::Subscriber robotStateSubscriber_;
        ros::Subscriber cloudBayInfoSubscriber_;
        ros::Subscriber chassisHealthSubscriber_;
        ros::Subscriber cloudRobotInfoSubscriber_;
        ros::Subscriber chassisBatterySubscriber_;
        ros::Subscriber backGardenInfoSubscriber_;
        ros::Subscriber cloudHivesInfoSubscriber_;
        ros::Subscriber cloudRobotStateUpdateSubscriber_;
        ros::Subscriber cloudWaypointStateUpdateSubscriber_;
        ros::Subscriber cloudDevicesInfoSubscriber_;
        ros::Subscriber cloudBuildingInfoSubscriber_;
        ros::Subscriber localCommunicationSubscriber_;
        ros::Subscriber robotConfigurationSubscriber_;
        ros::Subscriber infraryMsgSubscriber_;
        ros::Subscriber infraryStateSubscriber_;

        ros::ServiceClient updateCloudBindingClient_;

        std::chrono::system_clock::time_point robotLastEnableSec_;
        std::chrono::system_clock::time_point lastZigbeeUpdateSec_;
        std::chrono::system_clock::time_point stillModeExpireTime_;
        std::chrono::system_clock::time_point robotLastNotFaultSec_;
        std::chrono::system_clock::time_point robotStateLastUpdateSec_;

        std::chrono::system_clock::time_point lastUpdateCloudBindingSec_;
        std::chrono::system_clock::time_point lastUpdateLocalBindingSec_;

        std::shared_ptr<boost::asio::deadline_timer> generalPollTimer_;

        const char* toString(uint32_t waypointType)
        {
          switch (waypointType)
          {
            // case cti_msgs::CloudWaypointInfo::WAYPOINT_TYPE_BAY :
            //   return "BAY";
            // case cti_msgs::CloudWaypointInfo::WAYPOINT_TYPE_DOCK :
            //   return "DOCK";
            // case cti_msgs::CloudWaypointInfo::WAYPOINT_TYPE_PARKING :
            //   return "PARKING";
            case cti_msgs::CloudWaypointInfo::WAYPOINT_TYPE_NORMAL :
              return "NORMAL";
            case cti_msgs::CloudWaypointInfo::WAYPOINT_TYPE_TRANSIT :
              return "TRANSIT";
            case cti_msgs::CloudWaypointInfo::WAYPOINT_TYPE_BAY :
              return "BAY";
            case cti_msgs::CloudWaypointInfo::WAYPOINT_TYPE_PARKING :
              return "PARKING";
            case cti_msgs::CloudWaypointInfo::WAYPOINT_TYPE_DOCK_CHARGE :
              return "DOCK_CHARGE";
            case cti_msgs::CloudWaypointInfo::WAYPOINT_TYPE_DOCK_NORMAL :
              return "DOCK_NORMAL";
            case cti_msgs::CloudWaypointInfo::WAYPOINT_TYPE_DOCK_WALL :
              return "DOCK_WALL";
            case cti_msgs::CloudWaypointInfo::WAYPOINT_TYPE_LIFT :
              return "LIFT";
            case cti_msgs::CloudWaypointInfo::WAYPOINT_TYPE_HOLD :
              return "HOLD";
            case cti_msgs::CloudWaypointInfo::WAYPOINT_TYPE_AVOID :
              return "AVOID";
            case cti_msgs::CloudWaypointInfo::WAYPOINT_TYPE_GATE :
              return "GATE";
            case cti_msgs::CloudWaypointInfo::WAYPOINT_TYPE_HOME :
              return "HOME";
            default:
              return "UNDEFINE";
          }
        }

        const char* toString(RobotState state)
        {
          switch (state)
          {
            case RobotState::IDLE :
              return "IDLE";
            case RobotState::CHARGING :
              return "CHARGING";
            case RobotState::DOCKING :
              return "DOCKING";
            case RobotState::LIFTING :
              return "LIFTING";
            case RobotState::BUSY :
              return "BUSY";
            case RobotState::UNSTOPABLE :
              return "UNSTOPABLE";
            case RobotState::WAITINGLIFT :
              return "WAITINGLIFT";
            case RobotState::FAULT :
              return "FAULT";
            default:
              return "UNDEFINE";
          }
        }

        const char* toString(ChassisState state)
        {
          switch (state)
          {
            case ChassisState::ENABLED:
              return "ENABLED";
            case ChassisState::DISABLED:
              return "DISABLED";
            default:
              return "UNDEFINE";
          }
        }

        RobotState convertRobotState(int navState)
        {
          if (cti_msgs::BuildingRobotState::STATE_WAITTING_TASK == navState)
          {
            return RobotState::IDLE;
          }
          else if (cti_msgs::BuildingRobotState::STATE_CHARGING == navState)
          {
            return RobotState::CHARGING;
          }
          else if (cti_msgs::BuildingRobotState::STATE_CROSS_GATE == navState  ||
                   cti_msgs::BuildingRobotState::STATE_RECOVERY_CROSS_GATE_FAIL == navState ||
                   cti_msgs::BuildingRobotState::STATE_ENTERING_LIFE        == navState  ||
                   cti_msgs::BuildingRobotState::STATE_NAVIGATIONING_LIFT_PAUSE == navState  ||
                   cti_msgs::BuildingRobotState::STATE_NAVIGATIONING_PLACMENT_PAUSE == navState  ||
                   cti_msgs::BuildingRobotState::STATE_NAVIGATIONING_GATE_PAUSE == navState  ||
                   cti_msgs::BuildingRobotState::STATE_FINDING_ALL_QR == navState  ||
                   cti_msgs::BuildingRobotState::STATE_DYNAMIC_DODGING == navState  ||
                   cti_msgs::BuildingRobotState::STATE_STATIC_DODGING == navState  ||
                   cti_msgs::BuildingRobotState::STATE_WASHFLOOR_PAUSED == navState ||
                   (navState >= 300 && navState <= 400))
          {
            return RobotState::UNSTOPABLE;
          }
          else if (cti_msgs::BuildingRobotState::STATE_WAITTING_LIFE_INSIDE == navState  ||
                   cti_msgs::BuildingRobotState::STATE_WAITTING_LIFE_OUTSIDE == navState  ||
                   cti_msgs::BuildingRobotState::STATE_ENTERING_LIFE        == navState  ||
                   cti_msgs::BuildingRobotState::STATE_LEAVEING_LIFE        == navState  ||
                   cti_msgs::BuildingRobotState::STATE_NAVIGATIONING_LIFT   == navState  ||
                   cti_msgs::BuildingRobotState::STATE_RECOVERY_ENTERING_LIFE_FAIL == navState  ||
                   cti_msgs::BuildingRobotState::STATE_RECOVERY_CALL_LIFT_FAIL == navState  ||
                   cti_msgs::BuildingRobotState::STATE_RECOVERY_OPEN_LIFT_FAIL == navState  ||
                   cti_msgs::BuildingRobotState::STATE_NAVIGATION_APPLYING_ELEVATOR_AGAIN == navState  ||
                   cti_msgs::BuildingRobotState::STATE_ALLINLIFT_AND_STILLMOVINGIN == navState)
          {
            return RobotState::LIFTING;
          }
          else if (cti_msgs::BuildingRobotState::STATE_MOUNT_BOX        == navState  ||
                   cti_msgs::BuildingRobotState::STATE_CLOSETO_DUCK     == navState  ||
                   cti_msgs::BuildingRobotState::STATE_UNMOUNT_BOX      == navState  ||
                   cti_msgs::BuildingRobotState::STATE_RECOVERY_CHANGE_ERROR == navState  ||
                   cti_msgs::BuildingRobotState::STATE_CHARGE_ENTER     == navState  ||
                   cti_msgs::BuildingRobotState::STATE_CHARGE_LEAVE     == navState)
          {
            return RobotState::DOCKING;
          }
          else if ( cti_msgs::BuildingRobotState::STATE_NAVIGATIONING_PLACMENT == navState ||
                    cti_msgs::BuildingRobotState::STATE_NAVIGATIONING_GATE == navState ||
                    cti_msgs::BuildingRobotState::STATE_WAITTING_GATE_OPEN == navState ||
                    cti_msgs::BuildingRobotState::STATE_CROSS_GATE == navState ||
                    cti_msgs::BuildingRobotState::STATE_RECOVERY_CROSS_GATE_FAIL == navState)
          {
            return RobotState::BUSY;
          }
          else if (navState < 100)
          {
            return RobotState::IDLE;
          }
          else
          {
            return RobotState::FAULT;
          }
        }

        RobotState convertRobotStateFromNavigationState(int navState)
        {
          switch(navState)
          {
            case cti::common::AgentScheduleMessage::ERROR :
            case cti::common::AgentScheduleMessage::STOPPED :
            {
              return cti::missionSchedule::RobotState::FAULT;
            }
            case cti::common::AgentScheduleMessage::MOVING :
            {
              return cti::missionSchedule::RobotState::BUSY;
            }
            case cti::common::AgentScheduleMessage::PAUSE :
            case cti::common::AgentScheduleMessage::UNSTOPABLE :
            {
              return cti::missionSchedule::RobotState::UNSTOPABLE;
            }
            case cti::common::AgentScheduleMessage::DOCKING :
            case cti::common::AgentScheduleMessage::DOCKING_BLOCKED :
            {
              return cti::missionSchedule::RobotState::DOCKING;
            }
            case cti::common::AgentScheduleMessage::OCCUPY_LIFT :
            case cti::common::AgentScheduleMessage::LIFTING :
            {
              return cti::missionSchedule::RobotState::LIFTING;
            }
            case cti::common::AgentScheduleMessage::CHARGING :
            {
              return cti::missionSchedule::RobotState::CHARGING;
            }
            default :
            {
              return cti::missionSchedule::RobotState::IDLE;
            }
          }
          return cti::missionSchedule::RobotState::IDLE;
        }

        void updateSystemInfo(const RobotUtility& updateInfo)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          if (systemInfo_.find(updateInfo.robotId()) == systemInfo_.end())
          {
            systemInfo_[updateInfo.robotId()] = std::make_shared<RobotUtility>();
          }

          auto robotPtr = systemInfo_[updateInfo.robotId()];
          if (updateInfo.localizedSet())
          {
            robotPtr->setLocalized(updateInfo.localized());
          }
          if (updateInfo.targetFloorSet())
          {
            robotPtr->setTargetFloor(updateInfo.targetFloor());
          }
          if (updateInfo.chassisPowerSet())
          {
            robotPtr->setChassisPower(updateInfo.chassisPower());
          }
          if (updateInfo.desiredDestinationSet())
          {
            robotPtr->setDesiredDestination(updateInfo.desiredDestination());
          }
          if (updateInfo.currentElevatorIdSet())
          {
            robotPtr->setCurrentElevatorId(updateInfo.currentElevatorId());
          }
          if (updateInfo.currentElevatorDeviceIdSet())
          {
            robotPtr->setCurrentElevatorDeviceId(updateInfo.currentElevatorDeviceId());
          }
          if (updateInfo.robotStateSet())
          {
            robotPtr->setRobotState(updateInfo.robotState());
          }
          if (updateInfo.hiveAttachedOnRobotSet() && updateInfo.robotId() != robotId_)
          {
            robotPtr->setHiveAttachedOnRobot(updateInfo.hiveAttachedOnRobot());
          }
          if (updateInfo.robotStateStrSet())
          {
            robotPtr->setRobotStateStr(updateInfo.robotStateStr());
          }
          if (updateInfo.hiveAttachedSet())
          {
            robotPtr->setHiveAttached(updateInfo.hiveAttached());
          }
          if (updateInfo.disabledSet())
          {
            robotPtr->setDisabled(updateInfo.disabled());
          }
          if (updateInfo.operationModeSet())
          {
            robotPtr->setOperationMode(updateInfo.operationMode());
          }
          if (updateInfo.tagsSet())
          {
            robotPtr->setTags(updateInfo.tags());
          }

          if (updateInfo.lastPlatformUpdateSecSet())
          {
            if (updateInfo.lastPlatformUpdateSec() > robotPtr->lastPlatformUpdateSec())
            {
              robotPtr->setLastPlatformUpdateSec(updateInfo.lastPlatformUpdateSec());
              if (updateInfo.platformDestinationSet())
              {
                robotPtr->setPlatformDestination(updateInfo.platformDestination());
              }
            }
            if (updateInfo.lastPlatformUpdateSec() < robotPtr->lastLocalUpdateSec())
            {
              return ;
            }
          }
          else if (updateInfo.lastLocalUpdateSecSet())
          {
            robotPtr->setLastLocalUpdateSec(updateInfo.lastLocalUpdateSec());
            if (updateInfo.lastLocalUpdateSec() < robotPtr->lastPlatformUpdateSec())
            {
              return ;
            }
          }
          else
          {
            return ;
          }

          if (updateInfo.robotSignalSet())
          {
            robotPtr->setRobotSignal(updateInfo.robotSignal());
          }
          if (updateInfo.robotIdSet())
          {
            robotPtr->setRobotId(updateInfo.robotId());
          }
          if (updateInfo.localDestinationSet())
          {
            robotPtr->setLocalDestination(updateInfo.localDestination());
          }
          if (updateInfo.parkNameSet())
          {
            robotPtr->setParkName(updateInfo.parkName());
          }
          if (updateInfo.buildingNameSet())
          {
            robotPtr->setBuildingName(updateInfo.buildingName());
          }
          if (updateInfo.currentFloorSet())
          {
            robotPtr->setCurrentFloor(updateInfo.currentFloor());
          }
          if (updateInfo.robotStateStrSet())
          {
            robotPtr->setRobotStateStr(updateInfo.robotStateStr());
          }
          if (updateInfo.chassisStateStrSet())
          {
            robotPtr->setChassisStateStr(updateInfo.chassisStateStr());
          }
          if (updateInfo.lastPlatformUpdateSecSet())
          {
            robotPtr->setLastPlatformUpdateSec(updateInfo.lastPlatformUpdateSec());
          }
          if (updateInfo.lastLocalUpdateSecSet())
          {
            robotPtr->setLastLocalUpdateSec(updateInfo.lastLocalUpdateSec());
          }

          if (updateInfo.chassisStateSet())
          {
            robotPtr->setChassisState(updateInfo.chassisState());
            if (ChassisState::DISABLED == updateInfo.chassisState())
            {
              if (!robotPtr->currentStation().expired())
              {
                auto station = robotPtr->currentStation().lock();
                station->robotOccupyThisStation().reset();
              }
              robotPtr->currentStation().reset();
              return ;
            }
          }

          if (updateInfo.locationSet() || updateInfo.robotStateSet())
          {
            if (updateInfo.locationSet())
            {
              robotPtr->setLocation(updateInfo.location());
            }
            if (!robotPtr->currentStation().expired())
            {
              auto station = robotPtr->currentStation().lock();
              station->robotOccupyThisStation().reset();
            }

            robotPtr->currentStation().reset();
            if (RobotState::IDLE == robotPtr->robotState())
            {
              std::string stationId;
              double minDistance = std::numeric_limits<double>::max();
              for (auto& station : parkingStations_)
              {
                auto parking = station.second;
                auto stationDistance = robotPtr->location().distanceOf(parking->coordinate());
                if (robotPtr->currentFloor() == parking->floor() && stationDistance < minDistance &&
                    parking->isInsideOccupyArea(robotPtr->location()))
                {
                  minDistance = stationDistance;
                  stationId = parking->id();
                }
              }
              if (parkingStations_.find(stationId) != parkingStations_.end())
              {
                robotPtr->currentStation() = parkingStations_.at(stationId);
                parkingStations_.at(stationId)->robotOccupyThisStation() = robotPtr;
              }
            }
            else if (RobotState::CHARGING == robotPtr->robotState())
            {
              std::string stationId;
              double minDistance = std::numeric_limits<double>::max();
              for (auto& station : chargingStations_)
              {
                auto charging = station.second;
                auto stationDistance = robotPtr->location().distanceOf(charging->coordinate());
                if (robotPtr->currentFloor() == charging->floor() && stationDistance < minDistance &&
                    charging->isInsideOccupyArea(robotPtr->location()))
                {
                  minDistance = stationDistance;
                  stationId = charging->id();
                }
              }
              if (chargingStations_.find(stationId) != chargingStations_.end())
              {
                robotPtr->currentStation() = chargingStations_.at(stationId);
                chargingStations_.at(stationId)->robotOccupyThisStation() = robotPtr;
              }
            }
          }
        }

        Position convertIndexToPosition(int xIndex, int yIndex, double resolution)
        {
          Position returnValue;
          returnValue.slam.position.x = resolution * xIndex;
          returnValue.slam.position.y = resolution * yIndex;
          returnValue.slam.orientation.x = 0.0;
          returnValue.slam.orientation.y = 0.0;
          returnValue.slam.orientation.z = 0.0;
          returnValue.slam.orientation.w = 1.0;
          return returnValue;
        }

        static int sfold(const std::string& s)
        {
          int M = 60;
          int intLength = s.size() / 4;
          long sum = 0;
          for (int j = 0; j < intLength; j++)
          {
            std::string subString = s.substr(j * 4, (j * 4) + 4);
            char c[subString.size()];
            std::strcpy(c, subString.c_str());
            long int mult = 1;
            for (int k = 0; k < strlen(c); k++)
            {
              sum += c[k] * mult;
              mult *= 256;
            }
          }
          std::string subString = s.substr(intLength * 4);
          char c[subString.size()];
          std::strcpy(c, subString.c_str());
          long int mult = 1;
          for (int k = 0; k < strlen(c); k++)
          {
            sum += c[k] * mult;
            mult *= 256;
          }
          return (std::abs(sum) % M);
        }

      public:
        DeviceRuntimeUtility() {}

        ~DeviceRuntimeUtility() {}

        std::string getHubId()
        {
          return hubId_;
        }

        std::string getRobotCloudId()
        {
          return robotCloudId_;
        }

        std::string getBuildingId()
        {
          return buildingId_;
        }

        std::string getCurrentFloor()
        {
          return currentFloor_;
        }

        bool initialize()
        {
          lastZigbeeUpdateSec_ = std::chrono::system_clock::now();
          workSpacePath_ = std::string(::getenv("HOME")) + "/.ros/" PROJECT_NAME;
          if (auto ioService = cti::missionSchedule::common::getContainer()->resolveOrNull<boost::asio::io_service>())
          {
            generalPollTimer_ = std::make_shared<boost::asio::deadline_timer>(*ioService);
            generalPollTimer_->expires_from_now(boost::posix_time::milliseconds(500));
            generalPollTimer_->async_wait([self = shared_from_this()](const auto& ec) { self->pollTimerCallback(ec); });
          }

          if (auto nodeHandle = cti::missionSchedule::common::getContainer()->resolveOrNull<ros::NodeHandle>())
          {
            SPDLOG_INFO("DeviceUtility subscribe ros topic");
            infraryPublisher_ = nodeHandle->advertise<std_msgs::String>("/infrary_communication/infrary_send", 10);
            robotPowerThresholdSubscriber_ = nodeHandle->subscribe("/robotBatteryThreshold", 1, &DeviceRuntimeUtility::onModifyPowerThreshold, this);
            narrowAreaSubscriber_ = nodeHandle->subscribe("/zones", 1, &DeviceRuntimeUtility::onNarrowArea, this);
            robotStatusSubscriber_ = nodeHandle->subscribe("/robot_state", 1, &DeviceRuntimeUtility::onRobotStatus, this);
            chassisHealthSubscriber_ = nodeHandle->subscribe("/robot_base/health_state", 1, &DeviceRuntimeUtility::onChassisHealth, this);
            chassisBatterySubscriber_ = nodeHandle->subscribe("/robot_base/battery", 1, &DeviceRuntimeUtility::onChassisBattery, this);
            robotSensorSubscriber_ = nodeHandle->subscribe("/robot_base/sensors", 1, &DeviceRuntimeUtility::onRobotSensor, this);
            robotStateSubscriber_ = nodeHandle->subscribe("runningStatus/notify", 3, &DeviceRuntimeUtility::onRobotStateNotify, this);
            // backGardenInfoSubscriber_ = nodeHandle->subscribe("/back_garden/all_devices_info", 1, &DeviceRuntimeUtility::onBackGardenInfo, this);
            cloudRobotStateUpdateSubscriber_ = nodeHandle->subscribe("/cloud_scheduling_node/robot_platform_update", 10, &DeviceRuntimeUtility::onCloudRobotStateUpdate, this);
            cloudWaypointStateUpdateSubscriber_ = nodeHandle->subscribe("/cloud_scheduling_node/waypoint_platform_update", 10, &DeviceRuntimeUtility::onCloudWaypointStateUpdate, this);
            cloudHivesInfoSubscriber_ = nodeHandle->subscribe("/cloud_scheduling_node/building_hives", 10, &DeviceRuntimeUtility::onCloudBuildingHivesInfo, this);
            cloudDevicesInfoSubscriber_ = nodeHandle->subscribe("/cloud_scheduling_node/building_devices", 10, &DeviceRuntimeUtility::onCloudBuildingDevicesInfo, this);
            cloudRobotInfoSubscriber_ = nodeHandle->subscribe("/cloud_scheduling_node/cache/robot_info", 1, &DeviceRuntimeUtility::onCloudRobotInfo, this);
            cloudBuildingInfoSubscriber_ = nodeHandle->subscribe("/cloud_scheduling_node/cache/building_info", 1, &DeviceRuntimeUtility::onCloudBuildingMapInfo, this);
            cloudBayInfoSubscriber_ = nodeHandle->subscribe("/cloud_scheduling_node/cache/bay_info", 1, &DeviceRuntimeUtility::onCloudBayInfo, this);
            localCommunicationSubscriber_ = nodeHandle->subscribe("/local_communication_received_message", 1, &DeviceRuntimeUtility::onLocalCommunicationInfo, this);
            robotConfigurationSubscriber_ = nodeHandle->subscribe("/cloud_scheduling_node/cache/configuration", 1, &DeviceRuntimeUtility::onRobotConfiguration, this);
            infraryMsgSubscriber_ = nodeHandle->subscribe("/infrary_communication/infrary_receive", 10, &DeviceRuntimeUtility::onMissionMsgInfrary, this);
            infraryStateSubscriber_ = nodeHandle->subscribe("/infrary_communication/infrary_state", 10, &DeviceRuntimeUtility::onMissionStateInfrary, this);
            updateCloudBindingClient_ = nodeHandle->serviceClient<cti_msgs::CtiCommonService>("/cloud_scheduling_node/update_binding");
          }
          try
          {
            auto configFileName = workSpacePath_ + "/config.json";
            std::ifstream fs(configFileName, std::ios::in);
            nlohmann::json configJson;
            fs >> configJson;
            if (configJson.contains("mode") && configJson["mode"] == "MANUAL")
            {
              manualMode_ = true;
            }
            else
            {
              manualMode_ = false;
            }
          }
          catch(const std::exception& e)
          {
            manualMode_ = false;
            SPDLOG_INFO("read local configuration file {} exception - {}, will use the default config.", workSpacePath_, e.what());
          }
        }

        bool updateStationOccupyRelationship()
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          SPDLOG_INFO("run updateStationOccupyRelationship");
          for (auto& station : parkingStations_)
          {
            auto parking = station.second;
            parking->occupiedRobotId().clear();
          }
          for (auto& station : chargingStations_)
          {
            auto charging = station.second;
            charging->occupiedRobotId().clear();
          }
          for (auto& station : elevatorDodgeStations_)
          {
            auto dodge = station.second;
            dodge->occupiedRobotId().clear();
          }

          for (auto robotIter = systemInfo_.begin(); robotIter != systemInfo_.end(); robotIter++)
          {
            for (auto& station : parkingStations_)
            {
              auto parking = station.second;
              if (robotIter->second->currentFloor() == parking->floor() &&
                  robotIter->second->robotState() != RobotState::BUSY &&
                  ChassisState::ENABLED == robotIter->second->chassisState() &&
                  parking->isInsideOccupyArea(robotIter->second->location()))
              {
                parking->occupiedRobotId().push_back(robotIter->second->robotId());
                SPDLOG_INFO("updateStationOccupyRelationship robot {} occupy parking station {}", robotIter->second->robotId(), parking->name());
              }
            }
            for (auto& station : chargingStations_)
            {
              auto charging = station.second;
              if (robotIter->second->currentFloor() == charging->floor() &&
                  robotIter->second->robotState() != RobotState::BUSY &&
                  ChassisState::ENABLED == robotIter->second->chassisState() &&
                  charging->isInsideOccupyArea(robotIter->second->location()))
              {
                charging->occupiedRobotId().push_back(robotIter->second->robotId());
                SPDLOG_INFO("updateStationOccupyRelationship robot {} occupy charging station {}", robotIter->second->robotId(), charging->name());
              }
            }
            for (auto& station : elevatorDodgeStations_)
            {
              auto dodge = station.second;
              if (robotIter->second->currentFloor() == dodge->floor() &&
                  robotIter->second->robotState() != RobotState::BUSY &&
                  ChassisState::ENABLED == robotIter->second->chassisState() &&
                  dodge->isInsideOccupyArea(robotIter->second->location()))
              {
                dodge->occupiedRobotId().push_back(robotIter->second->robotId());
                SPDLOG_INFO("updateStationOccupyRelationship robot {} occupy dodge station {}", robotIter->second->robotId(), dodge->name());
              }
            }
          }
          SPDLOG_INFO("run updateStationOccupyRelationship end");
          return true;
        }

        bool isStillMode() override
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          if (stillMode_ && std::chrono::system_clock::now() > stillModeExpireTime_)
          {
            stillMode_ = !stillMode_;
            isDirectedCommandAllowedInStillMode_ = false;
          }
          return stillMode_;
        }

        bool isDirectedCommandAllowedInStillMode() override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          return isDirectedCommandAllowedInStillMode_;
        }

        void setStillMode(bool stillMode, bool allowDirectCommand, int duration) override
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          stillMode_ = stillMode;
          stillModeExpireTime_ = std::chrono::system_clock::now() + std::chrono::milliseconds(duration);
          isDirectedCommandAllowedInStillMode_ = allowDirectCommand;
        }

        void pollTimerCallback(const boost::system::error_code& ec)
        {
          if (ec == boost::system::errc::success)
          {
            std::unique_lock<std::shared_timed_mutex> lk(mutex_);
            std::chrono::system_clock::time_point currentSec = std::chrono::system_clock::now();

            for (auto robotIter = systemInfo_.begin(); robotIter != systemInfo_.end();)
            {
              SPDLOG_INFO("GeneralTimer sytem robots : {}", robotIter->second->toString());
              std::chrono::system_clock::time_point localUpdateSec = robotIter->second->lastLocalUpdateSec();
              std::chrono::system_clock::time_point platformUpdateSec = robotIter->second->lastPlatformUpdateSec();
              int localUpdateDuration = std::chrono::duration_cast<std::chrono::seconds>(currentSec - localUpdateSec).count();
              int platformUpdateDuration = std::chrono::duration_cast<std::chrono::seconds>(currentSec - platformUpdateSec).count();
              if (localUpdateDuration > 10)
              {
                Position invalidPosition;
                robotIter->second->setLocalDestination(invalidPosition);
                robotIter->second->setDesiredDestination(invalidPosition);
              }
              if (platformUpdateDuration > 10)
              {
                Position invalidPosition;
                robotIter->second->setPlatformDestination(invalidPosition);
              }
              if (robotIter->second->robotState() == RobotState::CHARGING || robotIter->second->robotState() == RobotState::IDLE)
              {
                if ((localUpdateDuration > 600 && platformUpdateDuration > 600) || !robotIter->second->localized())
                {
                  if (!robotIter->second->currentStation().expired())
                  {
                    auto station = robotIter->second->currentStation().lock();
                    station->robotOccupyThisStation().reset();
                    robotIter->second->currentStation().reset();
                  }
                  robotIter->second.reset();
                  systemInfo_.erase(robotIter++);
                  continue;
                }
              }
              else if ((localUpdateDuration > 30 && platformUpdateDuration > 30) || !robotIter->second->localized())
              {
                if (!robotIter->second->currentStation().expired())
                {
                  auto station = robotIter->second->currentStation().lock();
                  station->robotOccupyThisStation().reset();
                  robotIter->second->currentStation().reset();
                }
                robotIter->second.reset();
                systemInfo_.erase(robotIter++);
                continue;
              }
              robotIter++;
            }

            int zigbeeUpdateDuration = std::chrono::duration_cast<std::chrono::seconds>(currentSec - lastZigbeeUpdateSec_).count();
            if (zigbeeUpdateDuration > 20)
            {
              if (currentLocation_.valid() && systemInfo_[robotId_]->robotState() != LIFTING)
              {
                bool shouldZigbeeReceiveMessage = false;
                for (auto robotIter = systemInfo_.begin(); robotIter != systemInfo_.end(); robotIter++)
                {
                  if (robotIter->second->robotId() != robotId_ &&
                      robotIter->second->currentFloor() == currentFloor_ &&
                      robotIter->second->robotState() != RobotState::LIFTING &&
                      robotIter->second->chassisState() != ChassisState::DISABLED &&
                      robotIter->second->location().distanceOf(currentLocation_) < 15.0f)
                  {
                    shouldZigbeeReceiveMessage = true;
                    break;
                  }
                }
                if (shouldZigbeeReceiveMessage)
                {
                  zigbeeModuleFault_ = true;
                }
              }
            }
            else
            {
              zigbeeModuleFault_ = false;
            }

            auto robotInfo = systemInfo_.at(robotId_);
            bool hiveAttached = false;
            RobotState robotState = RobotState::DOCKING;
            SPDLOG_INFO("DeviceUtility cloud hive [{}], local hive [{}], infrary hive [{}], actual hive [{}]", hiveAttachedOnRobotFromCloud_,hiveAttachedOnRobotFromLocal_,hiveAttachedOnRobotFromInfrary_,robotInfo->hiveAttachedOnRobot());
            if (systemInfo_.find(robotId_) != systemInfo_.end())
            {
              auto robotInfo = systemInfo_.at(robotId_);
              hiveAttached = robotInfo->hiveAttached();
              robotState = robotInfo->robotState();
              if (robotState != RobotState::FAULT)
              {
                robotLastNotFaultSec_ = currentSec;
              }
              if (robotInfo->hiveAttachedSet() && (robotInfo->hiveAttached() || robotState == RobotState::CHARGING || robotState == RobotState::DOCKING) )
              {
                if (robotInfo->hiveAttachedOnRobot() != hiveAttachedOnRobotFromInfrary_ && !hiveAttachedOnRobotFromInfrary_.empty())
                {
                  robotInfo->setHiveAttachedOnRobot(hiveAttachedOnRobotFromInfrary_);
                  SPDLOG_INFO("DeviceUtility set actual hive id is [{}] by infrary", hiveAttachedOnRobotFromInfrary_);
                }
                else if (robotInfo->hiveAttachedOnRobot() != hiveAttachedOnRobotFromLocal_ && !hiveAttachedOnRobotFromLocal_.empty())
                {
                  robotInfo->setHiveAttachedOnRobot(hiveAttachedOnRobotFromLocal_);
                  SPDLOG_INFO("DeviceUtility set actual hive id is [{}] by local", hiveAttachedOnRobotFromLocal_);
                }
                else if (!robotInfo->hiveAttachedOnRobot().empty() && hiveAttachedOnRobotFromLocal_.empty() && hiveAttachedOnRobotFromInfrary_.empty())
                {
                  robotInfo->setHiveAttachedOnRobot("");
                  SPDLOG_INFO("DeviceUtility set actual hive id is null");
                }
              }
              else if (robotInfo->hiveAttachedSet() && !robotInfo->hiveAttached() && !robotInfo->hiveAttachedOnRobot().empty())
              {
                robotInfo->setHiveAttachedOnRobot("");
                SPDLOG_INFO("DeviceUtility set actual hive id is null");
              }
            }

            if (hiveAttachedOnRobotFromInfrary_ != hiveAttachedOnRobotFromLocal_)
            {
              auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IMissionPlanner>();
              bool shouldBindHive = false;
              if (hiveAttachedOnRobotFromInfrary_.empty())
              {
                SPDLOG_INFO("DeviceUtility infrary hive is empty local hive [{}] ", hiveAttachedOnRobotFromLocal_);
                shouldBindHive = false;
              }
              else
              {
                SPDLOG_INFO("DeviceUtility local is unbinding, bind local hive [{}]", hiveAttachedOnRobotFromInfrary_);
                shouldBindHive = true;
              }
              if ((RobotState::DOCKING == robotState || RobotState::CHARGING == robotState))
              {
                shouldBindHive = false;
              }
              if (std::chrono::duration_cast<std::chrono::seconds>(currentSec - lastUpdateLocalBindingSec_).count() >= 30 && shouldBindHive && hiveAttached)
              {
                missionPlanner->bindLocalHive(shouldBindHive, hiveAttachedOnRobotFromInfrary_);
                lastUpdateLocalBindingSec_ = currentSec;
              }
            }
            else
            {
              lastUpdateLocalBindingSec_ = currentSec - std::chrono::seconds(30);
            }

            if (hiveAttachedOnRobotFromInfrary_ != hiveAttachedOnRobotFromCloud_)
            {
              nlohmann::json bindingJson;
              cti_msgs::CtiCommonService updateBindingService;
              bool shouldBindHive = false;
              if (hiveAttachedOnRobotFromInfrary_.empty())
              {
                SPDLOG_INFO("DeviceUtility infrary hive is empty cloud hive [{}] ", hiveAttachedOnRobotFromCloud_);
                shouldBindHive = false;
              }
              else
              {
                SPDLOG_INFO("DeviceUtility local is unbinding, bind cloud hive [{}]", hiveAttachedOnRobotFromInfrary_);
                bindingJson["hiveId"] = hiveAttachedOnRobotFromInfrary_;
                bindingJson["bindingType"] = "binding";
                shouldBindHive = true;
              }
              if ((RobotState::DOCKING == robotState))
              {
                shouldBindHive = false;
              }
              if (std::chrono::duration_cast<std::chrono::seconds>(currentSec - lastUpdateCloudBindingSec_).count() >= 30 && shouldBindHive && hiveAttached)
              {
                updateBindingService.request.str = bindingJson.dump();
                SPDLOG_INFO("DeviceUtility call service update binding :[{}]", bindingJson.dump());
                if (updateCloudBindingClient_.call(updateBindingService) && !updateBindingService.response.result)
                {
                  SPDLOG_INFO("DeviceUtility call service error:{}", updateBindingService.response.error_message);
                }
                lastUpdateCloudBindingSec_ = currentSec;
              }
            }
            else
            {
              lastUpdateCloudBindingSec_ = currentSec - std::chrono::seconds(30);
            }

            if (generalPollTimer_)
            {
              generalPollTimer_->expires_from_now(boost::posix_time::milliseconds(1000));
              generalPollTimer_->async_wait([self = shared_from_this()](const auto& ec) { self->pollTimerCallback(ec); });
            }
          }
        }

        bool setRobotState(const RobotState& state)
        {
          if (robotId_.empty())
          {
            return false;
          }
          RobotUtility tempInfo;
          tempInfo.setRobotId(robotId_).setRobotState(state).setRobotStateStr(toString(state)).setLastLocalUpdateSec(std::chrono::system_clock::now());
          this->updateSystemInfo(std::move(tempInfo));
          SPDLOG_INFO("DeviceUtility set robot state : {}", systemInfo_[robotId_]->toString());
          return true;
        }

        void onNarrowArea(const cti_msgs::ZoneArray::ConstPtr &msg)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          narrowAreas_.clear();
          for (auto zoneIter = msg->zones.begin(); zoneIter != msg->zones.end(); zoneIter++)
          {
            if (zoneIter->property != cti_msgs::Zone::PROPERTY_NARROW)
            {
              continue;
            }
            std::vector<Position> zoneAnchors;
            for (auto anchorIter = zoneIter->anchors.begin(); anchorIter != zoneIter->anchors.end(); anchorIter++)
            {
              Position anchorPosition;
              anchorPosition.transferFromPose(*anchorIter);
              zoneAnchors.push_back(anchorPosition);
            }
            narrowAreas_.push_back(zoneAnchors);
          }
        }

        bool isPositionInsideNarrowArea(const Position& position)
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          double distance = std::numeric_limits<double>::max();
          for (auto narrowAreaIter = narrowAreas_.begin(); narrowAreaIter != narrowAreas_.end(); narrowAreaIter++)
          {
            double polygonDistance = calculatePositionDistanceToPolygon(position, *narrowAreaIter);
            if (polygonDistance < distance)
            {
              distance = polygonDistance;
            }
            if (distance > 0.0f)
            {
              return false;
            }
          }
          return true;
        }

        bool isPositionNearNarrowArea(const Position& position)
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          double distance = std::numeric_limits<double>::max();
          for (auto narrowAreaIter = narrowAreas_.begin(); narrowAreaIter != narrowAreas_.end(); narrowAreaIter++)
          {
            double polygonDistance = calculatePositionDistanceToPolygon(position, *narrowAreaIter);
            if (polygonDistance < distance)
            {
              distance = polygonDistance;
            }
            if (distance < 0.1f)
            {
              return true;
            }
          }
          return false;
        }

        void onRobotSensor(const cti_msgs::BaseSensors& message)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          if (robotId_.empty())
          {
            return;
          }
          if (message.state_motor_clutch == cti_msgs::BaseControl::MOTOR_ENABLE &&
              message.state_emergency_off == cti_msgs::BaseControl::MOTOR_NULL)
          {
            systemInfo_[robotId_]->setChassisState(ChassisState::ENABLED);
            systemInfo_[robotId_]->setChassisStateStr(toString(ChassisState::ENABLED));

          }
          else
          {
            systemInfo_[robotId_]->setChassisState(ChassisState::DISABLED);
            systemInfo_[robotId_]->setChassisStateStr(toString(ChassisState::DISABLED));
          }
        }

        void onCloudRobotStateUpdate(const std_msgs::String& msg)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          nlohmann::json robotState = nlohmann::json::parse(msg.data);
          SPDLOG_INFO("DeviceUtility get cloud robot state {} ,cloudId{}", msg.data,robotCloudId_);
          if (!robotState.is_object())
          {
            return;
          }
          if (!robotState.contains("id") || !robotState.at("id").is_string() || robotState.at("id").get<std::string>() != robotCloudId_)
          {
            return;
          }

          if (systemInfo_.find(robotId_) != systemInfo_.end() && robotState.contains("disabled") && robotState.at("disabled").is_boolean())
          {
            auto robotInfo = systemInfo_.at(robotId_);
            robotInfo->setDisabled(robotState.at("disabled").get<bool>());
            SPDLOG_INFO("DeviceUtility update self robot {} disabled state {} through cloud interface",robotId_,robotState.at("disabled").get<bool>());
          }
        }

        void onCloudWaypointStateUpdate(const std_msgs::String& msg)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          nlohmann::json waypointState = nlohmann::json::parse(msg.data);
          SPDLOG_INFO("DeviceUtility get cloud waypoint state {}", msg.data);
          if (waypointState.contains("id") && waypointState.at("id").is_string() && waypointState.contains("disabled") && waypointState.at("disabled").is_boolean())
          {
            std::string waypointId = waypointState.at("id").get<std::string>();
            if (waypointState.at("disabled").get<bool>())
            {
              if (systemWaypoints_.find(waypointId) != systemWaypoints_.end())
              {
              }
            }
            else
            {
            }
          }
        }

        void onRobotStateNotify(const cti_msgs::RobotNotifyState& msg)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          if (robotId_.empty())
          {
            return;
          }
          int robotState = msg.robotstate;
          if (robotState != robotState_)
          {
            robotState_ = robotState;
            robotStateLastUpdateSec_ = std::chrono::system_clock::now();
          }
          if (msg.run_mode == cti_msgs::RobotNotifyState::ROBOT_MODE_NAVIGATION)
          {
            navigationNormalMode_ = true;
          }
          else
          {
            navigationNormalMode_ = false;
          }
        }

        bool isStationValid(const std::string& stationId)
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (stationId.empty())
          {
            return false;
          }
          if (chargingStations_.find(stationId) != chargingStations_.end())
          {
            return true;
          }
          else if (parkingStations_.find(stationId) != parkingStations_.end())
          {
            return true;
          }
          return false;
        }

        bool isStationOccupied(const std::string& stationId)
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (stationId.empty())
          {
            return false;
          }
          if (chargingStations_.find(stationId) != chargingStations_.end())
          {
            if (!chargingStations_[stationId]->robotOccupyThisStation().expired())
            {
              return true;
            }
          }
          else if (parkingStations_.find(stationId) != parkingStations_.end())
          {
            if (!parkingStations_[stationId]->robotOccupyThisStation().expired())
            {
              return true;
            }
          }
          return false;
        }

        void useInfraryToSendMsg(const std::string& msg) override
        {
          std_msgs::String infraryMsg;
          infraryMsg.data = msg;
          infraryPublisher_.publish(infraryMsg);
        }

        std::string getStationOccupiedRobotId(const std::string& stationId)
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (stationId.empty())
          {
            return std::string();
          }
          if (chargingStations_.find(stationId) != chargingStations_.end())
          {
            if (!chargingStations_[stationId]->robotOccupyThisStation().expired())
            {
              auto robotPtr = chargingStations_[stationId]->robotOccupyThisStation().lock();
              return robotPtr->robotId();
            }
          }
          else if (parkingStations_.find(stationId) != parkingStations_.end())
          {
            if (!parkingStations_[stationId]->robotOccupyThisStation().expired())
            {
              auto robotPtr = parkingStations_[stationId]->robotOccupyThisStation().lock();
              return robotPtr->robotId();
            }
          }
          return std::string();
        }

        std::string getStationDockId(const std::string& stationId)
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (stationId.empty())
          {
            return std::string();
          }
          if (chargingStations_.find(stationId) != chargingStations_.end())
          {
            return chargingStations_[stationId]->dockId();
          }
          else if (parkingStations_.find(stationId) != parkingStations_.end())
          {
            return parkingStations_[stationId]->dockId();
          }
          return std::string();
        }

        void onRobotStatus(const cti_msgs::BuildingRobotState& msg)
        {
          RobotUtility tempInfo;
          robotId_ = msg.robotid;
          currentFloor_ = msg.current_floor;
          floorHashValue_ = sfold(msg.current_floor);
          buildingHashValue_ = sfold(msg.buildingname);
          Position location;
          location.transferFromPose(msg.pose);
          currentLocation_ = location;
          RobotState state = this->convertRobotState(msg.state);
          if (state == RobotState::FAULT)
          {
            robotFaultCount_++;
            if (robotFaultCount_ >= 3)
            {
              tempInfo.setRobotState(state).setRobotStateStr(toString(state));
            }
          }
          else
          {
            robotFaultCount_ = 0;
            tempInfo.setRobotState(state).setRobotStateStr(toString(state));
          }
          tempInfo.setRobotId(msg.robotid).setBuildingName(msg.buildingname).setParkName(msg.park_name).setCurrentFloor(msg.current_floor) \
            .setLocation(location).setHiveAttachedOnRobot(msg.box_uuid).setLocalized(msg.is_location) \
            .setCurrentElevatorId(msg.elevatorid).setLastLocalUpdateSec(std::chrono::system_clock::now());
          if (msg.payloadState == cti_msgs::RobotNotifyState::PAYLOADSTATE_LOADED)
          {
            tempInfo.setHiveAttached(true);
          }
          else
          {
            tempInfo.setHiveAttached(false);
          }
          if (msg.box_uuid != hiveAttachedOnRobotFromLocal_)
          {
            SPDLOG_INFO("DeviceUtility update hive id [{}] from local", msg.box_uuid);
          }
          if (!msg.box_uuid.empty())
          {
            hiveAttachedOnRobotFromLocal_ = msg.box_uuid;
          }
          else
          {
            hiveAttachedOnRobotFromLocal_.clear();
          }
          // if (msg.run_mode == cti_msgs::RobotNotifyState::ROBOT_MODE_NAVIGATION)
          // {
          //   navigationNormalMode_ = true;
          // }
          // else
          // {
          //   navigationNormalMode_ = false;
          // }
          SPDLOG_INFO("DeviceUtility update self robot {} state", tempInfo.robotId());
          this->updateSystemInfo(std::move(tempInfo));
        }

        std::string getHiveIdByHiveQr(const std::string& hiveQr)
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (hiveQr.empty())
          {
            return hiveQr;
          }
          if (hiveQrInfo_.find(hiveQr) != hiveQrInfo_.end())
          {
            return hiveQrInfo_[hiveQr];
          }
          return std::string();
        }

        void onCloudBuildingHivesInfo(const std_msgs::String& msg)
        {
          nlohmann::json hivesInfo = nlohmann::json::parse(msg.data);
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          std::set<std::string> hiveInfoSet;
          if (!hivesInfo.is_array())
          {
            return;
          }
          for (auto hive : hivesInfo)
          {
            if (!hive.contains("id") || !hive.contains("qr"))
            {
              return ;
            }
            std::string hiveQr = hive.at("qr").get<std::string>();
            std::string hiveId = hive.at("id").get<std::string>();
            hiveQrInfo_[hiveQr] = hiveId;
            hiveInfoSet.insert(hiveId);
            SPDLOG_INFO("DeviceUtility update hive {} information", hiveId);
            if (hiveInfo_.find(hiveId) != hiveInfo_.end())
            {
              hiveInfo_[hiveId].id = hiveId;
              hiveInfo_[hiveId].qr = hive.at("qr").get<std::string>();
              if (!hiveInfo_[hiveId].robotId.empty())
              {
                std::string robotId = hiveInfo_[hiveId].robotId;
                systemInfo_[robotId]->setHiveAttached(false);
                systemInfo_[robotId]->setHiveAttachedOnRobot("");
              }

              if (!hiveInfo_[hiveId].dockId.empty())
              {
                if (hive.contains("dockId") && hive["dockId"] == hiveInfo_[hiveId].dockId)
                {
                  continue;
                }
                std::string waypointId = hiveInfo_[hiveId].waypointId;
                hiveInfo_[hiveId].dockId.clear();
                hiveInfo_[hiveId].waypointId.clear();
                if (!chargingStations_[waypointId]->hiveOccupyThisStation().empty() &&
                    chargingStations_[waypointId]->hiveOccupyThisStation() == hiveId)
                {
                  chargingStations_[waypointId]->hiveOccupyThisStation().clear();
                }
              }

              if (hive.contains("dockId"))
              {
                std::string dockId = hive.at("dockId").get<std::string>();
                if (chargingDockWaypointIdMap_.find(dockId) != chargingDockWaypointIdMap_.end())
                {
                  std::string waypointId = chargingDockWaypointIdMap_[dockId];
                  if (chargingStations_.find(waypointId) != chargingStations_.end())
                  {
                    hiveInfo_[hiveId].dockId = dockId;
                    hiveInfo_[hiveId].waypointId = waypointId;
                    chargingStations_[waypointId]->hiveOccupyThisStation() = hiveId;
                  }
                }
              }

              // if (hive.contains("robotId"))
              // {
              //   std::string robotId = hive.at("robotId").get<std::string>();
              //   if (systemInfo_.find(robotId) != systemInfo_.end())
              //   {
              //     hiveInfo_[hiveId].robotId = robotId;
              //     systemInfo_[robotId]->setHiveAttached(true);
              //     systemInfo_[robotId]->setHiveAttachedOnRobot(hiveId);
              //   }
              // }
            }
            else
            {
              HiveUtility tempHive;
              tempHive.id = hiveId;
              tempHive.qr = hive.at("qr").get<std::string>();
              if (hive.contains("dockId"))
              {
                std::string dockId = hive.at("dockId").get<std::string>();
                if (chargingDockWaypointIdMap_.find(dockId) != chargingDockWaypointIdMap_.end())
                {
                  std::string waypointId = chargingDockWaypointIdMap_[dockId];
                  if (chargingStations_.find(waypointId) != chargingStations_.end())
                  {
                    tempHive.dockId = dockId;
                    tempHive.waypointId = waypointId;
                    chargingStations_[waypointId]->hiveOccupyThisStation() = hiveId;
                  }
                }
              }
              // if (hive.contains("robotId"))
              // {
              //   std::string robotId = hive.at("robotId").get<std::string>();
              //   if (systemInfo_.find(robotId) != systemInfo_.end())
              //   {
              //     tempHive.robotId = robotId;
              //     systemInfo_[robotId]->setHiveAttachedOnRobot(hiveId);
              //   }
              // }
              hiveInfo_[hiveId] = tempHive;
            }
          }
          for (auto hiveIter = hiveInfo_.begin(); hiveIter != hiveInfo_.end();)
          {
            if (hiveInfoSet.find(hiveIter->first) == hiveInfoSet.end())
            {
              if (!hiveIter->second.dockId.empty())
              {
                std::string waypointId = hiveIter->second.waypointId;
                if (!chargingStations_[waypointId]->hiveOccupyThisStation().empty() &&
                    chargingStations_[waypointId]->hiveOccupyThisStation() == hiveIter->first)
                {
                  chargingStations_[waypointId]->hiveOccupyThisStation().clear();
                }
              }
              hiveInfo_.erase(hiveIter++);
            }
            else
            {
              hiveIter++;
            }
          }
        }

        void onModifyPowerThreshold(const mission_schedule::batteryMsg::ConstPtr& msg)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          SPDLOG_INFO("get msg: bad: {} ,basic: {}, good: {}", msg->badBattery, msg->basicBattery, msg->goodBattery);
          SPDLOG_INFO("before power :{} ,badThreshold: {},basicThreshold: {},goodThreshold: {} ", systemInfo_.at(robotId_)->chassisPower(), badPowerThreshold_, chargingToBasicPowerThreshold_, goodPowerThreshold_);
          if (msg->badBattery != -1000.0)
          {
            badPowerThreshold_ = msg->badBattery;
          }
          if (msg->basicBattery != -1000.0)
          {
            chargingToBasicPowerThreshold_ = msg->basicBattery;
          }
          if (msg->goodBattery != -1000.0)
          {
            goodPowerThreshold_ = msg->goodBattery;
          }
          SPDLOG_INFO("aftre power :{} ,badThreshold: {},basicThreshold: {},goodThreshold: {} ", systemInfo_.at(robotId_)->chassisPower(), badPowerThreshold_, chargingToBasicPowerThreshold_, goodPowerThreshold_);
        }

        void onCloudBuildingDevicesInfo(const std_msgs::String& msg)
        {
          nlohmann::json devicesInfo = nlohmann::json::parse(msg.data);
          if (!devicesInfo.is_array())
          {
            return ;
          }
          for (auto device : devicesInfo)
          {
            RobotUtility tempInfo;
            if (!device.contains("deviceId"))
            {
              continue ;
            }
            tempInfo.setRobotId(device.at("deviceId").get<std::string>());
            if (device.contains("operationMode"))
            {
              tempInfo.setOperationMode(device.at("operationMode").get<std::string>());
            }
            else
            {
              tempInfo.setOperationMode(std::string());
            }
            std::vector<std::string> robotTags;
            if (device.contains("tags"))
            {
              for (auto tag : device["tags"])
              {
                robotTags.push_back(tag);
              }
            }
            tempInfo.setTags(robotTags);

            if (device.at("deviceId").get<std::string>() == robotId_)
            {
              if (device.contains("disabled"))
              {
                tempInfo.setDisabled(device.at("disabled").get<bool>());
              }
              if (device.contains("lastModifiedTime"))
              {
                auto timeCount = device.at("lastModifiedTime").get<int64_t>();
                std::chrono::milliseconds dur(timeCount);
                std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> dt(dur);
                tempInfo.setLastPlatformUpdateSec(dt);
                int platformUpdateDuration = std::chrono::duration_cast<std::chrono::minutes>(std::chrono::system_clock::now() - dt).count();
                if (std::abs(platformUpdateDuration) > 10)
                {
                  return;
                }
              }
              if (device.contains("hiveId"))
              {
                if (hiveAttachedOnRobotFromCloud_ != device.at("hiveId").get<std::string>())
                {
                  SPDLOG_INFO("DeviceUtility update hive id [{}] from cloud", device.at("hiveId").get<std::string>());
                }
                hiveAttachedOnRobotFromCloud_ = device.at("hiveId").get<std::string>();
              }
              else
              {
                if (!hiveAttachedOnRobotFromCloud_.empty())
                {
                  SPDLOG_INFO("DeviceUtility update no hive from cloud");
                }
                hiveAttachedOnRobotFromCloud_.clear();
              }
              this->updateSystemInfo(std::move(tempInfo));
              SPDLOG_INFO("DeviceUtility update self robot {} state {} through cloud interface", tempInfo.robotId(), tempInfo.toString());
              continue ;
            }

            if (device.contains("localized"))
            {
              tempInfo.setLocalized(device.at("localized").get<bool>());
            }
            if (device.contains("coordinates"))
            {
              Position location;
              location.parseFromJson(device.at("coordinates"));
              if (!location.valid())
              {
                return ;
              }
              tempInfo.setLocation(location);
            }
            if (device.contains("floorName"))
            {
              tempInfo.setCurrentFloor(device.at("floorName").get<std::string>());
            }
            if (device.contains("disabled"))
            {
              tempInfo.setDisabled(device.at("disabled").get<bool>());
            }
            if (device.contains("battery") && device["battery"].contains("percentage"))
            {
              int percentage = device["battery"]["percentage"].get<int>();
              tempInfo.setChassisPower(percentage);
            }
            if (device.contains("loadingState") && device["loadingState"] == "LOADED")
            {
              tempInfo.setHiveAttached(true);
            }
            else
            {
              tempInfo.setHiveAttached(false);
            }
            if (device.contains("targetWaypointId"))
            {
              std::string waypointId = device["targetWaypointId"].get<std::string>();
              if (!waypointId.empty() && systemWaypoints_.find(waypointId) != systemWaypoints_.end())
              {
                tempInfo.setTargetFloor(systemWaypoints_[waypointId].floor());
                tempInfo.setPlatformDestination(systemWaypoints_[waypointId].coordinate());
              }
            }
            if (device.contains("elevatorId"))
            {
              tempInfo.setCurrentElevatorId(device.at("elevatorId").get<std::string>());
            }
            else
            {
              tempInfo.setCurrentElevatorId("");
            }

            if (device.contains("hiveId"))
            {
              tempInfo.setHiveAttachedOnRobot(device.at("hiveId").get<std::string>());
            }
            else
            {
              tempInfo.setHiveAttachedOnRobot("");
            }
            if (device.contains("navigationStateCode"))
            {
              RobotState state = this->convertRobotState(device.at("navigationStateCode").get<int>());
              tempInfo.setRobotState(state);
              tempInfo.setRobotStateStr(toString(state));
            }
            if (device.contains("emergencyStopEngaged"))
            {
              bool emergencyStop = device.at("emergencyStopEngaged").get<bool>();
              ChassisState state = ChassisState::ENABLED;
              if (emergencyStop)
              {
                state = ChassisState::DISABLED;
              }
              tempInfo.setChassisState(state);
              tempInfo.setChassisStateStr(toString(state));
            }
            if (device.contains("lastModifiedTime"))
            {
              auto timeCount = device.at("lastModifiedTime").get<int64_t>();
              std::chrono::milliseconds dur(timeCount);
              std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> dt(dur);
              tempInfo.setLastPlatformUpdateSec(dt);
              int platformUpdateDuration = std::chrono::duration_cast<std::chrono::minutes>(std::chrono::system_clock::now() - dt).count();
              if (std::abs(platformUpdateDuration) > 10)
              {
                SPDLOG_INFO("DeviceUtility update robot {} drop update {} cause message too old through cloud interface", tempInfo.robotId(), tempInfo.toString());
                return;
              }
            }
            // tempInfo.setLastUpdateSec(std::chrono::system_clock::now());
            SPDLOG_INFO("DeviceUtility update robot {} state {} through cloud interface", tempInfo.robotId(), tempInfo.toString());
            this->updateSystemInfo(std::move(tempInfo));
          }
        }

        void onBackGardenInfo(const cti_msgs::BackgardenDownloadInfo& msg)
        {
          for (auto robotIter = msg.robots.begin(); robotIter!=msg.robots.end(); robotIter++)
          {
            if (robotIter->virtual_machine)
            {
              continue;
            }
            RobotUtility tempInfo;
            Position location, destination;
            location.transferFromPose(robotIter->current_pose);
            destination.transferFromPose(robotIter->goal_pose);
            RobotState state = this->convertRobotState(robotIter->state);
            tempInfo.setRobotId(std::to_string(robotIter->robot_id)).setTargetFloor(robotIter->target_floor).setBuildingName(robotIter->current_building)     \
              .setCurrentFloor(robotIter->current_floor).setLocation(location).setLocalDestination(destination).setRobotState(state).setRobotStateStr(toString(state)).setLastLocalUpdateSec(std::chrono::system_clock::now());
            // SPDLOG_INFO("DeviceUtility update robot {} state through back garden", tempInfo.robotId());
            this->updateSystemInfo(std::move(tempInfo));
          }
        }

        void onChassisHealth(const cti_msgs::HealthState& msg)
        {
          // SPDLOG_INFO("DeviceUtility update self {} chassis state", robotId_);
          if (robotId_.empty())
          {
            return;
          }
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          if(msg.base_vaild != 0)
          {
            SPDLOG_INFO("Agent {} Chassis Disabled!", robotId_);
            systemInfo_[robotId_]->setChassisState(ChassisState::DISABLED);
            systemInfo_[robotId_]->setChassisStateStr(toString(ChassisState::DISABLED));
          }
          else
          {
            systemInfo_[robotId_]->setChassisState(ChassisState::ENABLED);
            systemInfo_[robotId_]->setChassisStateStr(toString(ChassisState::ENABLED));
          }
        }

        void onChassisBattery(const cti_msgs::BaseBatteryState& msg)
        {
          // SPDLOG_INFO("DeviceUtility update self {} chassis power", robotId_);
          if (robotId_.empty())
          {
            return;
          }
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          if (msg.RSOC > 0)
          {
            systemInfo_[robotId_]->setChassisPower(msg.RSOC);
          }
          else
          {
            systemInfo_[robotId_]->setChassisPower(1.0);
          }
        }

        void onCloudRobotInfo(const cti_msgs::CloudRobotInfo& msg)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          robotCloudId_ = msg.robot_id;
          // manualMode_ = msg.disableMissionSchedule;
        }

        void onCloudBuildingMapInfo(const cti_msgs::CloudBuildingInfo& msg)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          systemWaypoints_.clear();
          parkingDistribution_.clear();
          chargingDistribution_.clear();
          chargingDockWaypointIdMap_.clear();
          std::set<std::string> parkingStationSet;
          std::set<std::string> chargingStationSet;
          std::set<std::string> elevatorDodgeStationSet;
          std::map<std::string, std::vector<Position>> parkingStationFloorMap;
          std::map<std::string, std::vector<Position>> chargingStationFloorMap;
          hubId_ = msg.hubId;
          buildingId_ = msg.id;
          for (auto floorIter : msg.floors)
          {
            for (auto stationIter : floorIter.waypoints)
            {
              Position stationPosition;
              stationPosition.transferFromPose(stationIter.coordinate);
              if (stationPosition.distanceOf() < 0.001 || !stationPosition.valid())
              {
                continue;
              }
              StationModel station;
              station.id() = stationIter.id;
              station.floorId() = floorIter.id;
              station.name() = stationIter.name;
              station.floor() = floorIter.floor;
              station.coordinate() = stationPosition;
              station.type() = toString(stationIter.waypointType);
              systemWaypoints_[stationIter.id] = station;
              // SPDLOG_INFO("MissionSchedule receive station {}", stationIter.name);
              if (station.type() == "BAY")
              {
                auto parkingStation = std::make_shared<StationModel>(station);
                if (parkingStations_.find(stationIter.id) != parkingStations_.end())
                {
                  auto oldParkingStation = parkingStations_[stationIter.id];
                  if (!oldParkingStation->robotOccupyThisStation().expired() &&
                      parkingStation->coordinate().distanceOf(oldParkingStation->coordinate()) > 0.1f)
                  {
                    auto robotPtr = oldParkingStation->robotOccupyThisStation().lock();
                    robotPtr->currentStation().reset();
                    oldParkingStation->robotOccupyThisStation().reset();
                  }
                  if (parkingStation->coordinate().distanceOf(oldParkingStation->coordinate()) > 0.1f)
                  {
                    oldParkingStation.reset();
                    parkingStations_.erase(stationIter.id);
                    parkingStations_[stationIter.id] = parkingStation;
                  }
                }
                else
                {
                  parkingStations_[stationIter.id] = parkingStation;
                }
                parkingStationSet.insert(stationIter.id);
                parkingStationFloorMap[parkingStation->floor()].push_back(parkingStation->coordinate());
                SPDLOG_INFO("MissionSchedule receive parking station {}", parkingStation->name());
              }
              else if (station.type() == "AVOID")
              {
                auto elevatorDodgeStation = std::make_shared<StationModel>(station);
                if (elevatorDodgeStations_.find(stationIter.id) != elevatorDodgeStations_.end())
                {
                  auto oldDodgeStation = elevatorDodgeStations_[stationIter.id];
                  if (elevatorDodgeStation->coordinate().distanceOf(oldDodgeStation->coordinate()) > 0.1f)
                  {
                    oldDodgeStation.reset();
                    elevatorDodgeStations_.erase(stationIter.id);
                    elevatorDodgeStations_[stationIter.id] = elevatorDodgeStation;
                  }
                }
                else
                {
                  elevatorDodgeStations_[stationIter.id] = elevatorDodgeStation;
                }

                SPDLOG_INFO("MissionSchedule receive elevator dodge station {}", elevatorDodgeStation->name());
                elevatorDodgeStationSet.insert(stationIter.id);
              }
            }
            for (auto dockIter : floorIter.docks)
            {
              if (systemWaypoints_.find(dockIter.waypointId) == systemWaypoints_.end())
              {
                continue;
              }
              systemWaypoints_[dockIter.waypointId].dockId() = dockIter.id;
              systemWaypoints_[dockIter.waypointId].stationArea() = computePolygonArea(systemWaypoints_.at(dockIter.waypointId).coordinate(), 0.45f, 0.95f);
              if (!dockIter.qr.empty())
              {
                systemWaypoints_[dockIter.waypointId].qr() = dockIter.qr;
              }
              if (dockIter.chargeable && !dockIter.disabled && !dockIter.qr.empty())
              {
                auto chargingStation = std::make_shared<StationModel>(systemWaypoints_.at(dockIter.waypointId));
                chargingStation->qr() = dockIter.qr;
                systemWaypoints_[dockIter.waypointId].qr() = dockIter.qr;
                chargingStation->dockId() = dockIter.id;
                if (chargingStations_.find(dockIter.waypointId) != chargingStations_.end())
                {
                  auto oldChargingStation = chargingStations_[dockIter.waypointId];
                  if (!oldChargingStation->robotOccupyThisStation().expired() &&
                      chargingStation->coordinate().distanceOf(oldChargingStation->coordinate()) > 0.1f)
                  {
                    auto robotPtr = oldChargingStation->robotOccupyThisStation().lock();
                    robotPtr->currentStation().reset();
                    oldChargingStation->robotOccupyThisStation().reset();
                  }
                  if (chargingStation->coordinate().distanceOf(oldChargingStation->coordinate()) > 0.1f)
                  {
                    oldChargingStation.reset();
                    chargingStations_.erase(dockIter.waypointId);
                    chargingStations_[dockIter.waypointId] = chargingStation;
                  }
                }
                else
                {
                  chargingStations_[dockIter.waypointId] = chargingStation;
                }
                chargingDockWaypointIdMap_[dockIter.id] = dockIter.waypointId;
                chargingStationSet.insert(dockIter.waypointId);
                chargingStationFloorMap[chargingStation->floor()].push_back(chargingStation->coordinate());
                SPDLOG_INFO("MissionSchedule receive charging station {}", chargingStation->name());
              }
            }
          }
          for (auto stationIter = elevatorDodgeStations_.begin(); stationIter != elevatorDodgeStations_.end();)
          {
            if (elevatorDodgeStationSet.find(stationIter->first) == elevatorDodgeStationSet.end())
            {
              stationIter->second.reset();
              elevatorDodgeStations_.erase(stationIter++);
            }
            else
            {
              stationIter++;
            }
          }
          for (auto stationIter = parkingStations_.begin(); stationIter != parkingStations_.end();)
          {
            if (parkingStationSet.find(stationIter->first) == parkingStationSet.end())
            {
              if (!stationIter->second->robotOccupyThisStation().expired())
              {
                auto robotPtr = stationIter->second->robotOccupyThisStation().lock();
                robotPtr->currentStation().reset();
                stationIter->second->robotOccupyThisStation().reset();
              }
              stationIter->second.reset();
              parkingStations_.erase(stationIter++);
            }
            else
            {
              stationIter++;
            }
          }
          for (auto floorIter = parkingStationFloorMap.begin(); floorIter != parkingStationFloorMap.end(); floorIter++)
          {
            StationDistributionInfo floorStationDistribution;
            Position maxProjection;
            auto line = calculateLinearRegression(floorIter->second);
            auto maxDistance = std::numeric_limits<double>::min();
            for (auto poseIter = floorIter->second.begin(); poseIter != floorIter->second.end(); poseIter++)
            {
              auto Projection = calculatePointProjectionPositionToLine(line.first, line.second, *poseIter);
              double distance = Projection.distanceOf();
              if (distance > maxDistance)
              {
                maxDistance = distance;
                maxProjection = Projection;
              }
            }
            floorStationDistribution.line = line;
            floorStationDistribution.lineAnchor = maxProjection;
            parkingDistribution_.emplace(floorIter->first, floorStationDistribution);
          }

          for (auto stationIter = chargingStations_.begin(); stationIter != chargingStations_.end();)
          {
            if (chargingStationSet.find(stationIter->first) == chargingStationSet.end())
            {
              if (!stationIter->second->robotOccupyThisStation().expired())
              {
                auto robotPtr = stationIter->second->robotOccupyThisStation().lock();
                robotPtr->currentStation().reset();
                stationIter->second->robotOccupyThisStation().reset();
              }
              stationIter->second.reset();
              chargingStations_.erase(stationIter++);
            }
            else
            {
              stationIter++;
            }
          }
          for (auto floorIter = chargingStationFloorMap.begin(); floorIter != chargingStationFloorMap.end(); floorIter++)
          {
            StationDistributionInfo floorStationDistribution;
            Position maxProjection;
            auto line = calculateLinearRegression(floorIter->second);
            auto maxDistance = std::numeric_limits<double>::min();
            for (auto poseIter = floorIter->second.begin(); poseIter != floorIter->second.end(); poseIter++)
            {
              auto Projection = calculatePointProjectionPositionToLine(line.first, line.second, *poseIter);
              double distance = Projection.distanceOf();
              if (distance > maxDistance)
              {
                maxDistance = distance;
                maxProjection = Projection;
              }
            }
            floorStationDistribution.line = line;
            floorStationDistribution.lineAnchor = maxProjection;
            chargingDistribution_[floorIter->first] = floorStationDistribution;
          }
          SPDLOG_INFO("MissionSchedule receive charging station end!");
        }

        void onCloudBayInfo(const std_msgs::String& msg)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          std::set<std::string> parkingStationSet;
          nlohmann::json baysInfo = nlohmann::json::parse(msg.data);
          std::map<std::string, std::vector<Position>> parkingStationFloorMap;
          SPDLOG_INFO("MissionSchedule receive bay info!");
          if (!baysInfo.is_array())
          {
            return ;
          }
          parkingDistribution_.clear();
          for (auto bay : baysInfo)
          {
            if (!bay.contains("waypointId") || !bay.contains("waypoint") || !bay["waypoint"].contains("indoorCoordinates") ||
                "BAY" != bay.at("waypoint").at("waypointType").get<std::string>() || !bay.at("waypoint").at("indoor").get<bool>())
            {
              continue;
            }
            StationModel station;
            Position stationPosition;
            std::string waypointId = bay.at("waypointId").get<std::string>();
            stationPosition.parseFromJson(bay["waypoint"]["indoorCoordinates"]);
            station.id() = waypointId;
            station.coordinate() = stationPosition;
            station.name() = bay.at("waypoint").at("name").get<std::string>();
            station.floor() = bay.at("floorName").get<std::string>();
            station.floorId() = bay.at("waypoint").at("floorId").get<std::string>();
            station.type() = bay.at("waypoint").at("waypointType").get<std::string>();
            if (bay.contains("disabled") && !bay.at("disabled").get<bool>()
                && bay.contains("temporary") && !bay.at("temporary").get<bool>())
            {
              auto parkingStation = std::make_shared<StationModel>(station);
              if (parkingStations_.find(waypointId) != parkingStations_.end())
              {
                auto oldParkingStation = parkingStations_[waypointId];
                if (!oldParkingStation->robotOccupyThisStation().expired() &&
                    parkingStation->coordinate().distanceOf(oldParkingStation->coordinate()) > 0.1f)
                {
                  auto robotPtr = oldParkingStation->robotOccupyThisStation().lock();
                  robotPtr->currentStation().reset();
                  oldParkingStation->robotOccupyThisStation().reset();
                }
                oldParkingStation.reset();
                parkingStations_.erase(waypointId);
              }
              parkingStationSet.insert(waypointId);
              parkingStations_[waypointId] = parkingStation;
              parkingStationFloorMap[parkingStation->floor()].push_back(parkingStation->coordinate());
              SPDLOG_INFO("MissionSchedule receive parking station {}", parkingStation->name());
            }
          }
          for (auto stationIter = parkingStations_.begin(); stationIter != parkingStations_.end();)
          {
            if (parkingStationSet.find(stationIter->first) == parkingStationSet.end())
            {
              if (!stationIter->second->robotOccupyThisStation().expired())
              {
                auto robotPtr = stationIter->second->robotOccupyThisStation().lock();
                robotPtr->currentStation().reset();
                stationIter->second->robotOccupyThisStation().reset();
              }
              stationIter->second.reset();
              parkingStations_.erase(stationIter++);
            }
            else
            {
              stationIter++;
            }
          }
          for (auto floorIter = parkingStationFloorMap.begin(); floorIter != parkingStationFloorMap.end(); floorIter++)
          {
            StationDistributionInfo floorStationDistribution;
            Position maxProjection;
            auto line = calculateLinearRegression(floorIter->second);
            auto maxDistance = std::numeric_limits<double>::min();
            for (auto poseIter = floorIter->second.begin(); poseIter != floorIter->second.end(); poseIter++)
            {
              auto Projection = calculatePointProjectionPositionToLine(line.first, line.second, *poseIter);
              double distance = Projection.distanceOf();
              if (distance > maxDistance)
              {
                maxDistance = distance;
                maxProjection = Projection;
              }
            }
            floorStationDistribution.line = line;
            floorStationDistribution.lineAnchor = maxProjection;
            parkingDistribution_.emplace(floorIter->first, floorStationDistribution);
          }
        }

        void onLocalCommunicationInfo(const std_msgs::String& msg)
        {
          RobotUtility tempInfo;

          lastZigbeeUpdateSec_ = std::chrono::system_clock::now();

          cti::common::AgentScheduleMessage agentInfo;
          agentInfo.ParseFromString(msg.data);
          tempInfo.setRobotId(std::to_string(agentInfo.agent_id()));
          if (agentInfo.has_agent_navigation_state())
          {
            if (cti::common::AgentScheduleMessage::STOPPED == agentInfo.agent_navigation_state())
            {
              tempInfo.setLocalized(false);
              tempInfo.setChassisState(ChassisState::DISABLED);
            }
            else
            {
              tempInfo.setChassisState(ChassisState::ENABLED);
            }
            RobotState state = convertRobotStateFromNavigationState(agentInfo.agent_navigation_state());
            tempInfo.setRobotState(state).setRobotStateStr(toString(state));
          }
          tempInfo.setDesiredDestination(Position());
          if (agentInfo.current_floor_size() > 1)
          {
            if (buildingHashValue_ != agentInfo.current_floor(1) || floorHashValue_ != agentInfo.current_floor(0))
            {
              return ;
            }
            tempInfo.setCurrentFloor(currentFloor_);
            if (agentInfo.current_floor_size() >= 4)
            {
              tempInfo.setChassisPower(agentInfo.current_floor(2));
              tempInfo.setHiveAttached(false);
              if (agentInfo.current_floor(3) == 1 || agentInfo.current_floor(3) == 3)
              {
                tempInfo.setHiveAttached(true);
              }
              if (agentInfo.current_floor(3) == 1 || agentInfo.current_floor(3) == 2)
              {
                tempInfo.setRobotState(RobotState::CHARGING).setRobotStateStr(toString(RobotState::CHARGING));
              }
              if (agentInfo.current_floor_size() == 6)
              {
                tempInfo.setDesiredDestination(convertIndexToPosition(agentInfo.current_floor(4), agentInfo.current_floor(5), 0.01f));
              }
            }
          }
          if (agentInfo.agent_position_size() > 1)
          {
            tempInfo.setLocation(convertIndexToPosition(agentInfo.agent_position(0), agentInfo.agent_position(1), 0.01f));
            if (4 == agentInfo.agent_position_size())
            {
              tempInfo.setLocalDestination(convertIndexToPosition(agentInfo.agent_position(2), agentInfo.agent_position(3), 0.01f));
            }
          }
          else
          {
            return ;
          }
          if (currentFloor_ != tempInfo.currentFloor())
          {
            return ;
          }
          tempInfo.setLocalized(true);
          tempInfo.setTargetFloor("");
          tempInfo.setCurrentElevatorDeviceId("");
          if (agentInfo.has_elevator_id() && agentInfo.has_target_floor())
          {
            tempInfo.setCurrentElevatorDeviceId(agentInfo.elevator_id());
            tempInfo.setRobotState(RobotState::LIFTING).setRobotStateStr(toString(RobotState::LIFTING));
          }
          tempInfo.setLastLocalUpdateSec(std::chrono::system_clock::now());
          SPDLOG_INFO("DeviceUtility update robot {} state {} through local communication interface", tempInfo.robotId(), tempInfo.toString());
          this->updateSystemInfo(std::move(tempInfo));
        }

        void onRobotConfiguration(const std_msgs::String& msg)
        {
        }

        bool isManualMode() override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (!navigationNormalMode_)
          {
            return true;
          }
          if (systemInfo_.find(robotId_) != systemInfo_.end())
          {
            if (systemInfo_[robotId_]->disabledSet() && systemInfo_[robotId_]->disabled())
            {
              return true;
            }
            if (systemInfo_[robotId_]->localizedSet() && !systemInfo_[robotId_]->localized())
            {
              return true;
            }
            if (systemInfo_[robotId_]->chassisStateSet() && systemInfo_[robotId_]->chassisState() == ChassisState::DISABLED)
            {
              return true;
            }
          }
          return manualMode_;
        }

        void onMissionStateInfrary(const std_msgs::String& msg)
        {
          nlohmann::json infraryStateJson = nlohmann::json::parse(msg.data);
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          if (!infraryStateJson.is_object())
          {
            return;
          }
          SPDLOG_INFO("DeviceUtility get infrary state :{}", msg.data);
          if (infraryStateJson.contains("paired") && infraryStateJson["paired"].is_boolean())
          {
            pairingState_ = infraryStateJson["paired"].get<bool>();
            SPDLOG_INFO("DeviceUtility infrary is {}", pairingState_);
          }
          if (infraryStateJson.contains("infraryHiveId") && infraryStateJson["infraryHiveId"].is_string())
          {
            hiveAttachedOnRobotFromInfrary_ = infraryStateJson["infraryHiveId"];
            SPDLOG_INFO("DeviceUtility update hive [{}] from infrary", infraryStateJson["infraryHiveId"]);
          }
          else
          {
            hiveAttachedOnRobotFromInfrary_.clear();
          }
        }

        void onMissionMsgInfrary(const std_msgs::String& msg)
        {
          nlohmann::json infraryMsgJson = nlohmann::json::parse(msg.data);
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          if (!infraryMsgJson.is_object() || (robotId_.empty()) || (!systemInfo_.at(robotId_)) || (!systemInfo_.at(robotId_)->localizedSet()) || (!systemInfo_.at(robotId_)->localized()))
          {
            SPDLOG_INFO("DeviceUtility system is not ready ,drop infrary msg:{}", msg.data);
            return;
          }
          auto missionCenter = cti::missionSchedule::common::getContainer()->resolve<IPlatformMissionCenter>();
          SPDLOG_INFO("DeviceUtility get infrary msg:{}", msg.data);
          if (infraryMsgJson.contains("stopMoving") && infraryMsgJson["stopMoving"].is_boolean())
          {
            if (infraryMsgJson["stopMoving"].get<bool>())
            {
              SPDLOG_INFO("DeviceUtility set still mode 10mins");
              stillMode_ = true;
              stillModeExpireTime_ = std::chrono::system_clock::now() + std::chrono::milliseconds(10 * 60 * 1000);
              isDirectedCommandAllowedInStillMode_ = false;
            }
            else
            {
              SPDLOG_INFO("DeviceUtility reset still mode");
              stillMode_ = false;
              stillModeExpireTime_ = std::chrono::system_clock::now();
              isDirectedCommandAllowedInStillMode_ = true;
            }
          }
          else
          {
            SPDLOG_INFO("stop moving error type: {}");
          }
          if (infraryMsgJson.contains("sterilizeState") && infraryMsgJson["sterilizeState"].is_boolean() && infraryMsgJson["sterilizeState"].get<bool>())
          {
            missionCenter->executeSterilizeNextStep();
          }
        }

        bool isRobotShouldCancelOrdersWhenDisabled() override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (systemInfo_.find(robotId_) == systemInfo_.end() || systemInfo_.at(robotId_)->hiveAttached() ||
              !systemInfo_[robotId_]->disabledSet() || !systemInfo_[robotId_]->disabled())
          {
            robotLastEnableSec_ = std::chrono::system_clock::now();
            return false;
          }
          std::chrono::system_clock::time_point currentSec = std::chrono::system_clock::now();
          int disabledDuration = std::chrono::duration_cast<std::chrono::seconds>(currentSec - robotLastEnableSec_).count();
          return disabledDuration > 30;
        }

        bool isRobotShouldCancelOrdersWhenFault() override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (systemInfo_.find(robotId_) == systemInfo_.end() || systemInfo_.at(robotId_)->hiveAttached() ||
              !systemInfo_[robotId_]->robotStateSet() || systemInfo_[robotId_]->robotState() != RobotState::FAULT )
          {
            // robotLastFaultSec_ = std::chrono::system_clock::now();
            return false;
          }
          std::chrono::system_clock::time_point currentSec = std::chrono::system_clock::now();
          int faultDuration = std::chrono::duration_cast<std::chrono::seconds>(currentSec - robotLastNotFaultSec_).count();
          return faultDuration > 30;
        }

        bool isDeviceIdle() override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          return RobotState::IDLE == systemInfo_[robotId_]->robotState();
        }

        bool isDeviceStatusOk() override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (RobotState::BUSY == systemInfo_[robotId_]->robotState())
          {
            Position position = systemInfo_[robotId_]->location();
            double distance = std::numeric_limits<double>::max();
            for (auto narrowAreaIter = narrowAreas_.begin(); narrowAreaIter != narrowAreas_.end(); narrowAreaIter++)
            {
              double polygonDistance = calculatePositionDistanceToPolygon(position, *narrowAreaIter);
              if (polygonDistance < distance)
              {
                distance = polygonDistance;
              }
              if (distance < 0.5f)
              {
                SPDLOG_INFO("DeviceUtility robot close to narrow area distance {}", distance);
                return false;
              }
            }
          }
          return RobotState::FAULT != systemInfo_[robotId_]->robotState() &&
                 RobotState::LIFTING != systemInfo_[robotId_]->robotState() &&
                 RobotState::UNSTOPABLE != systemInfo_[robotId_]->robotState() &&
                 RobotState::DOCKING != systemInfo_[robotId_]->robotState();
        }

        bool isDeviceStatusFault() override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          return RobotState::FAULT == systemInfo_[robotId_]->robotState();
        }

        bool isDevicePowerBad(std::string robotId) override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (robotId.empty())
          {
            robotId = robotId_;
          }

          if (systemInfo_.find(robotId) != systemInfo_.end())
          {
            SPDLOG_INFO("power :{} ,badthreshold: {}", systemInfo_.at(robotId)->chassisPower(), badPowerThreshold_);
            return systemInfo_.at(robotId)->chassisPower() <= badPowerThreshold_;
          }
          return false;
        }

        bool isDeviceChargingToBasicPower(std::string robotId) override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (robotId.empty())
          {
            robotId = robotId_;
          }

          if (systemInfo_.find(robotId) != systemInfo_.end())
          {
            SPDLOG_INFO("power :{} ,chargingToBasicPowerThreshold_: {}", systemInfo_.at(robotId)->chassisPower(), chargingToBasicPowerThreshold_);
            return systemInfo_.at(robotId)->chassisPower() > chargingToBasicPowerThreshold_;
          }
          return false;
        }

        bool  isZigbeeModuleFault() override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          return zigbeeModuleFault_;
        }

        bool isHiveLoaded() override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (RobotState::CHARGING == systemInfo_[robotId_]->robotState())
          {
            return false;
          }
          return systemInfo_.at(robotId_)->hiveAttached();
        }

        bool isHiveAttachedOnRobot() override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          // if (RobotState::CHARGING == systemInfo_[robotId_]->robotState())
          // {
          //   return false;
          // }
          return systemInfo_.at(robotId_)->hiveAttached();
        }

        bool isDevicePowerNice(std::string robotId) override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (robotId.empty())
          {
            robotId = robotId_;
          }

          if (systemInfo_.find(robotId) != systemInfo_.end())
          {
            return systemInfo_.at(robotId)->chassisPower() > nicePowerThreshold_;
          }
          return false;
        }

        bool isDevicePowerGood(std::string robotId) override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (robotId.empty())
          {
            robotId = robotId_;
          }

          if (systemInfo_.find(robotId) != systemInfo_.end())
          {
            return systemInfo_.at(robotId)->chassisPower() > goodPowerThreshold_;
          }
          return false;
        }

        bool isDeviceSignalGood() override
        {
        }

        bool isInfraryPaired() override
        {
          return hiveAttachedOnRobotFromInfrary_.empty();
        }

        std::shared_ptr<RobotUtility> getRobotInfo(std::string robotId) override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (robotId.empty())
          {
            robotId = robotId_;
          }

          if (systemInfo_.find(robotId) != systemInfo_.end())
          {
            return systemInfo_.at(robotId);
          }
          return nullptr;
        }

        bool getHiveInfo(std::string hiveId, HiveUtility& hive) override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (hiveId.empty() || hiveInfo_.find(hiveId) == hiveInfo_.end())
          {
            return false;
          }
          hive = hiveInfo_[hiveId];
          return true;
        }

        bool getWaypointInfo(std::string waypointId, StationModel& waypoint) override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (waypointId.empty() || systemWaypoints_.find(waypointId) == systemWaypoints_.end())
          {
            return false;
          }
          waypoint = systemWaypoints_[waypointId];
          return true;
        }

        std::shared_ptr<StationModel> getStationInfo(std::string stationId) override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (stationId.empty())
          {
            return nullptr;
          }

          if (chargingStations_.find(stationId) != chargingStations_.end())
          {
            return chargingStations_.at(stationId);
          }
          else if (parkingStations_.find(stationId) != parkingStations_.end())
          {
            return parkingStations_.at(stationId);
          }
          return nullptr;
        }

        bool getChargingStationDistribution(std::string floor, StationDistributionInfo& info) override
        {
          if (chargingDistribution_.find(floor) == chargingDistribution_.end())
          {
            return false;
          }
          info = chargingDistribution_.at(floor);
          return true;
        }

        bool getParkingStationDistribution(std::string floor, StationDistributionInfo& info) override
        {
          if (parkingDistribution_.find(floor) == parkingDistribution_.end())
          {
            return false;
          }
          info = parkingDistribution_.at(floor);
          return true;
        }

        void foreachSystemRobot(std::function<bool(std::shared_ptr<RobotUtility>)> handler) override
        {
          if (!handler)
          {
            return ;
          }

          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          for (auto& robot : systemInfo_)   // get all robots info
          {
            if (handler(robot.second))
            {
              break;
            }
          }
        }

        void foreachElevatorDodgeStation(std::function<bool(std::shared_ptr<StationModel>)> handler) override
        {
          if (!handler)
          {
            return ;
          }

          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          for (auto& station : elevatorDodgeStations_)
          {
            if (handler(station.second))
            {
              break;
            }
          }
        }

        void foreachWaypoint(std::function<bool(const StationModel& waypoint)> handler) override
        {
          if (!handler)
          {
            return ;
          }

          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          for (auto& station : systemWaypoints_)
          {
            if (handler(station.second))
            {
              break;
            }
          }
        }

        void foreachParkingStation(std::function<bool(std::shared_ptr<StationModel>)> handler) override
        {
          if (!handler)
          {
            return ;
          }

          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          for (auto& station : parkingStations_)
          {
            if (handler(station.second))
            {
              break;
            }
          }
        }

        void foreachChargingStation(std::function<bool(std::shared_ptr<StationModel>)> handler) override
        {
          if (!handler)
          {
            return ;
          }

          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          for (auto& station : chargingStations_)
          {
            if (handler(station.second))
            {
              break;
            }
          }
        }
    };

    std::shared_ptr<IDeviceRuntimeUtility> createRuntimeStatusMonitor()
    {
      auto monitor = std::make_shared<DeviceRuntimeUtility>();
      monitor->initialize();
      return std::dynamic_pointer_cast<IDeviceRuntimeUtility>(monitor);
    }
  }
}
