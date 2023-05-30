#include "cti_spdlog.h"
#include "deviceMiscellaneous/MissionPlanner.h"

namespace cti
{
  namespace missionSchedule
  {
    class MissionPlanner : public IMissionPlanner, public std::enable_shared_from_this<MissionPlanner>
    {
      private:
        StationModel target_;
        nlohmann::json command_;
        bool waitElevatorExpired_;
        bool stillCommandTransmited_{false};
        bool recovercommandtransmited_{false};

        bool lastClearGoalFlag_;
        bool lastMoveable_ = true;
        double densityRange_ = 10.0;
        bool partialDensity_ = true;

        int shouldRebootIdleTimeThreshold_;
        int shouldRebootUptimeThreshold_;
        int rebootLowerTime_;
        int rebootUpperTime_;
        std::string failedReason_;
        std::string locationStoragePath_ ;
        ScheduleState plannerState_{ScheduleState::READY};
        std::map<std::string, BanedStation> banedStations_;
        // std::map<std::string, BanedStation> chargingBanedStation_;
        std::vector<IStationCostFunction *> parkingStationCritics_;
        std::vector<IStationCostFunction *> chargingStationCritics_;

        Position desiredDestination_;
        nlohmann::json decisionOrderJson_;
        std::shared_ptr<ScheduleOrder> currentLocalOrder_;
        std::list<std::shared_ptr<ScheduleOrder>> localScheduleOrders_;

        std::chrono::system_clock::time_point lastAbortPublishSec_;
        std::chrono::system_clock::time_point lastOrderOrCommandUpdateSec_;

        std::shared_timed_mutex mutex_;

        PathCostFunction pathCost_;
        PowerCostFunction powerCost_;
        ParkingCostFunction parkingCost_;
        ChargingCostFunction chargingCost_;
        ElevatorCostFunction elevatorCost_;
        ElevatorCostFunction elevatorChargingCost_;
        RobotConflictCostFunction robotConflictCost_;
        StationDistributionCostFunction stationDistributionCost_;

        ros::Publisher missionPublisher_;
        ros::Publisher robotRestartPublisher_;
        ros::Publisher desiredDestinationPublisher_;
        ros::Subscriber missionResultSubscriber_;
        ros::Subscriber missionRebootTimeSubscriber_;
        ros::Subscriber missionForceRebootSubscriber_;
        ros::Subscriber missionRelocateSubscriber_;
        ros::Publisher clearGoalPublisher_;
        ros::ServiceClient densityClient_;

        bool isCommandChanged(const nlohmann::json& command)
        {
          auto firstCommandDiff = cti::missionSchedule::contrastJson(command_, command);
          if (command.contains("commandType") &&
              !firstCommandDiff.contains("commandType") &&
              (command["commandType"] == "ABORT" || command["commandType"] == "RESTART" || command["commandType"] == "SLEEP"))
          {
            return false;
          }
          if (firstCommandDiff.contains("id") ||
              firstCommandDiff.contains("waypointId") ||
              firstCommandDiff.contains("elevatorid") ||
              firstCommandDiff.contains("commandType") ||
              firstCommandDiff.contains("scheduleType"))
          {
            return true;
          }
          auto secondCommandDiff = cti::missionSchedule::contrastJson(command, command_);
          if (secondCommandDiff.contains("id") ||
              secondCommandDiff.contains("waypointId") ||
              secondCommandDiff.contains("elevatorId") ||
              secondCommandDiff.contains("commandType") ||
              secondCommandDiff.contains("scheduleType"))
          {
            return true;
          }
          return false;
        }

        bool shouldRobotAvoidDock()
        {
          bool shouldRobotAvoidDock = false;
          StationModel destinationWaypoint;
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotPtr = deviceUtility->getRobotInfo();
          SPDLOG_INFO("Mission Planner check robot {} should avoid dock", robotPtr->robotId());
          if (robotPtr->robotState() != RobotState::CHARGING || robotPtr->currentStation().expired())
          {
            SPDLOG_INFO("Mission Planner robot {} should avoid dock first", robotPtr->robotId());
            return true;
          }
          auto robotId = robotPtr->robotId();
          auto robotCurrentPower = robotPtr->chassisPower();
          auto robotCurrentStation = robotPtr->currentStation();
          deviceUtility->foreachSystemRobot([&shouldRobotAvoidDock, robotId, robotCurrentPower, robotCurrentStation](std::shared_ptr<RobotUtility> robot)
          {
            auto station = robotCurrentStation.lock();
            if (robot->robotId() == robotId || robot->currentFloor() != station->floor() || !robot->localizedSet() || !robot->localized() || robot->disabled())
            {
              return false;
            }
            if (robot->currentFloor() == station->floor() && robot->platformDestination().valid() &&
                station->isInsideOccupyArea(robot->platformDestination()))
            {
              shouldRobotAvoidDock = true;
              SPDLOG_INFO("Mission Planner robot {} should avoid dock second cause of robot {}", robotId, robot->robotId());
              return true;
            }
            if (robot->currentFloor() == station->floor() &&
                robot->localDestination().valid() && station->isInsideOccupyArea(robot->localDestination()))
            {
              shouldRobotAvoidDock = true;
              SPDLOG_INFO("Mission Planner robot {} should avoid dock third cause of robot {}", robotId, robot->robotId());
              return true;
            }
            if (robot->currentFloor() == station->floor() && robot->chassisPowerSet() &&
                robot->chassisPower() < 30.0f && robot->robotState() != RobotState::CHARGING)
            {
              shouldRobotAvoidDock = true;
              SPDLOG_INFO("Mission Planner robot {} should avoid dock fourth cause of robot {}", robotId, robot->robotId());
              return true;
            }
            if (robot->currentFloor() == station->floor() && robot->chassisPowerSet() &&
                robot->robotState() != RobotState::CHARGING && robotCurrentPower - robot->chassisPower() > 30.0f)
            {
              shouldRobotAvoidDock = true;
              SPDLOG_INFO("Mission Planner robot {} should avoid dock fifth cause of robot {}", robotId, robot->robotId());
              return true;
            }
            return false;
          });
          return shouldRobotAvoidDock;
        }

        bool isReciprocalDestinationOccupied(const std::string& destinationId, const std::string& dodgeRobotId)
        {
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotPtr = deviceUtility->getRobotInfo();   //get this robot info
          auto dodgeRobotPtr = deviceUtility->getRobotInfo(dodgeRobotId);   //get this robot info
          if (dodgeRobotPtr->desiredDestination().valid() && dodgeRobotPtr->desiredDestination().distanceOf(robotPtr->location()) < 0.7f)
          {
            SPDLOG_INFO("Mission Planner robot {} reciprocal occupied ", dodgeRobotPtr->robotId());
            return true;
          }
          return false;
        }

        bool shouldDestinationDodge(const std::string& destinationId, std::string& dodgeRobotId, bool considerBusyRobot = true)
        {
          std::string dodgeCauseRobotId;
          bool shouldDestinationDodge = false;
          StationModel destinationWaypoint;
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotPtr = deviceUtility->getRobotInfo();   //get this robot info
          auto robotId = robotPtr->robotId();
          if (destinationId.empty() ||
              !deviceUtility->getWaypointInfo(destinationId, destinationWaypoint))
          {
            SPDLOG_INFO("Mission Planner shouldDestinationDodge can't find destinationId {}", destinationId);
            return false;
          }
          if (robotPtr->currentFloor() != destinationWaypoint.floor() ||
              robotPtr->location().distanceOf(destinationWaypoint.coordinate()) >= 8.0f)
          {
            SPDLOG_INFO("Mission Planner shouldDestinationDodge floor {} stationFloor {} distance {}", robotPtr->currentFloor(), destinationWaypoint.floor(), robotPtr->location().distanceOf(destinationWaypoint.coordinate()));
            return false;
          }
          deviceUtility->foreachSystemRobot([destinationWaypoint, robotId, considerBusyRobot, &shouldDestinationDodge, &dodgeCauseRobotId](std::shared_ptr<RobotUtility> robot)
          {
            if (robot->robotId() == robotId || robot->currentFloor() != destinationWaypoint.floor() ||
                !robot->localizedSet() || !robot->localized() || (considerBusyRobot && robot->robotState() == RobotState::BUSY))
            {
              return false;
            }
            SPDLOG_INFO("Mission Planner shouldDestinationDodge check robot {}", robot->robotId());
            if (destinationWaypoint.isInsideOccupyArea(robot->location()))
            {
              dodgeCauseRobotId = robot->robotId();
              shouldDestinationDodge = true;
              SPDLOG_INFO("Mission Planner robot {} should destination dodge cause of robot {}", robotId, robot->robotId());
              return true;
            }
            return false;
          });
          if ((&dodgeRobotId != nullptr) && shouldDestinationDodge)
          {
            dodgeRobotId = dodgeCauseRobotId;
          }
          return shouldDestinationDodge;
        }

        std::string findBestDodgeStation(const std::string& destinationId)
        {
          std::string dodgeStation;
          double maxThresholdDistance = 20.0;
          double minThresholdDistance = 10.0;
          double minDistance = std::numeric_limits<double>::max();
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotPtr = deviceUtility->getRobotInfo();
          Position currentPosition = robotPtr->location();
          std::string currentFloor = robotPtr->currentFloor();
          deviceUtility->foreachWaypoint([currentFloor, minThresholdDistance, maxThresholdDistance, currentPosition, &minDistance, &dodgeStation](const StationModel& waypoint)
          {
            if (waypoint.floor() != currentFloor)
            {
              return false;
            }
            double distance = waypoint.coordinate().distanceOf(currentPosition);
            if (distance > minThresholdDistance && distance < minDistance)
            {
              minDistance = distance;
              dodgeStation = waypoint.id();
              if (distance < maxThresholdDistance)
              {
                return true;
              }
            }
            return false;
          });

          SPDLOG_INFO("Mission Planner best dodge station {}", dodgeStation);
          return dodgeStation;
        }

        std::string findDodgeWaypoint(const std::string& destinationId)
        {
          double bestDodgeWaypointCost = std::numeric_limits<double>::max();
          StationModel waypoint;
          std::string bestDodgeWaypointId;
          auto pathPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IPathPlanner>();
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotPtr = deviceUtility->getRobotInfo();
          std::string robotId = robotPtr->robotId();
          std::string currentFloor = robotPtr->currentFloor();
          Position currentPosition = robotPtr->location();
          Position targetPosition = currentPosition;
          if (deviceUtility->getWaypointInfo(destinationId, waypoint))
          {
            targetPosition = waypoint.coordinate();
          }
          deviceUtility->foreachDodgeStation([currentFloor, currentPosition, targetPosition, robotId, pathPlanner, deviceUtility, &bestDodgeWaypointId, &bestDodgeWaypointCost](std::shared_ptr<StationModel> station)
          {
            double stationDodgeCost = 0.0;
            if (currentFloor != station->floor())
            {
              return false;
            }
            SPDLOG_INFO("Mission Planner target dodge check station {}", station->name());
            double targetDodgeDistance = pathPlanner->calculatePathCost(targetPosition, station->coordinate(), currentFloor);
            double robotDodgeDistance = pathPlanner->calculatePathCost(currentPosition, station->coordinate(), currentFloor);
            if (targetDodgeDistance > 30.0)
            {
              return false;
            }
            if (!station->occupiedRobotId().empty())
            {
              if (std::find(station->occupiedRobotId().begin(), station->occupiedRobotId().end(), robotId) == station->occupiedRobotId().end())
              {
                return false;
              }
            }
            deviceUtility->foreachSystemRobot([currentFloor, robotDodgeDistance, robotId, &stationDodgeCost, stationPtr = std::weak_ptr<StationModel>(station), pathPlanner](std::shared_ptr<RobotUtility> robot)
            {
              if (robot->robotId() == robotId || robot->currentFloor() != currentFloor || !robot->localizedSet() || !robot->localized())
              {
                return false;
              }
              auto station = stationPtr.lock();
              double dodgeDistance = pathPlanner->calculatePathCost(robot->location(), station->coordinate(), currentFloor);
              if (station->isInsideOccupyArea(robot->localDestination()))
              {
                SPDLOG_INFO("Mission Planner target dodge check with robot {} id", robot->robotId());
                if (robot->robotState() != RobotState::FAULT)
                {
                  if (std::abs(dodgeDistance - robotDodgeDistance) > 0.5)
                  {
                    if (dodgeDistance > robotDodgeDistance)
                    {
                      stationDodgeCost += 10.0;
                    }
                    else
                    {
                      stationDodgeCost += 100.0;
                    }
                  }
                  else if (std::atoi(robot->robotId().data()) < std::atoi(robotId.data()))
                  {
                    stationDodgeCost += 100.0;
                  }
                  else
                  {
                    stationDodgeCost += 10.0;
                  }
                }
              }
              return false;
            });
            SPDLOG_INFO("Mission Planner target dodge check station {} cost {}", station->name(), stationDodgeCost);
            if (stationDodgeCost < bestDodgeWaypointCost && stationDodgeCost < 100)
            {
              bestDodgeWaypointCost = stationDodgeCost;
              bestDodgeWaypointId = station->id();
            }
            return false;
          });
          SPDLOG_INFO("Mission Planner best target dodge station {} cost {}", bestDodgeWaypointId, bestDodgeWaypointCost);
          return bestDodgeWaypointId;
        }

        bool findElevatorDodgeWaypoint(const std::string& elevatorId, std::string& dodgeWaypointId)
        {
          double bestDodgeWaypointCost = std::numeric_limits<double>::max();
          std::string bestDodgeWaypointId;
          auto pathPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IPathPlanner>();
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto elevatorPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IElevatorPlanner>();
          auto robotPtr = deviceUtility->getRobotInfo();
          std::string robotId = robotPtr->robotId();
          std::string currentFloor = robotPtr->currentFloor();
          Position currentPosition = robotPtr->location();
          Position elevatorEnterPosition = elevatorPlanner->getElevatorEnterPosition(elevatorId, currentFloor);
          deviceUtility->foreachDodgeStation([currentFloor, currentPosition, elevatorEnterPosition, robotId, pathPlanner, deviceUtility, &bestDodgeWaypointId, &bestDodgeWaypointCost](std::shared_ptr<StationModel> station)
          {
            double stationDodgeCost = 0.0;
            if (currentFloor != station->floor())
            {
              return false;
            }
            SPDLOG_INFO("Mission Planner elevator dodge check station {}", station->name());
            double elevatorDodgeDistance = pathPlanner->calculatePathCost(elevatorEnterPosition, station->coordinate(), currentFloor);
            double robotDodgeDistance = pathPlanner->calculatePathCost(currentPosition, station->coordinate(), currentFloor);
            if (elevatorDodgeDistance > 30.0)
            {
              return false;
            }
            if (!station->occupiedRobotId().empty())
            {
              if (std::find(station->occupiedRobotId().begin(), station->occupiedRobotId().end(), robotId) == station->occupiedRobotId().end())
              {
                return false;
              }
            }
            deviceUtility->foreachSystemRobot([currentFloor, robotDodgeDistance, robotId, &stationDodgeCost, stationPtr = std::weak_ptr<StationModel>(station), pathPlanner](std::shared_ptr<RobotUtility> robot)
            {
              if (robot->robotId() == robotId || robot->currentFloor() != currentFloor || !robot->localizedSet() || !robot->localized())
              {
                return false;
              }
              auto station = stationPtr.lock();
              double dodgeDistance = pathPlanner->calculatePathCost(robot->location(), station->coordinate(), currentFloor);
              if (station->isInsideOccupyArea(robot->localDestination()))
              {
                SPDLOG_INFO("Mission Planner elevator dodge check with robot {} id", robot->robotId());
                if (robot->robotState() != RobotState::FAULT)
                {
                  if (std::abs(dodgeDistance - robotDodgeDistance) > 0.5)
                  {
                    if (dodgeDistance > robotDodgeDistance)
                    {
                      stationDodgeCost += 10.0;
                    }
                    else
                    {
                      stationDodgeCost += 100.0;
                    }
                  }
                  else if (std::atoi(robot->robotId().data()) < std::atoi(robotId.data()))
                  {
                    stationDodgeCost += 100.0;
                  }
                  else
                  {
                    stationDodgeCost += 10.0;
                  }
                }
              }
              return false;
            });
            SPDLOG_INFO("Mission Planner elevator dodge check station {} cost {}", station->name(), stationDodgeCost);
            if (stationDodgeCost < bestDodgeWaypointCost)
            {
              bestDodgeWaypointCost = stationDodgeCost;
              bestDodgeWaypointId = station->id();
            }
            return false;
          });
          SPDLOG_INFO("Mission Planner best elevator dodge station {} cost {}", bestDodgeWaypointId, bestDodgeWaypointCost);
          if (bestDodgeWaypointCost < 100)
          {
            dodgeWaypointId = bestDodgeWaypointId;
            return true;
          }
          return false;
        }

        bool shouldElevatorDodge(const std::string& elevatorId, const std::string& waypointId)
        {
          StationModel waypoint;
          std::string currentFloor;
          Position elevatorPosition, elevatorEnterPosition, currentPosition;
          auto pathPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IPathPlanner>();
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotPtr = deviceUtility->getRobotInfo();
          auto robotId = robotPtr->robotId();
          currentPosition = robotPtr->location();
          if (deviceUtility->getWaypointInfo(waypointId, waypoint) &&
              robotPtr->currentFloor() == waypoint.floor())
          {
            return false;
          }
          bool shouldElevatorDodge = false;
          auto elevatorPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IElevatorPlanner>();
          currentFloor = robotPtr->currentFloor();
          elevatorPosition = elevatorPlanner->getElevatorInsidePosition(elevatorId, currentFloor);
          elevatorEnterPosition = elevatorPlanner->getElevatorEnterPosition(elevatorId, currentFloor);
          SPDLOG_INFO("Mission Planner elevator dodge position {}", elevatorPosition.toString());
          if (!elevatorPosition.valid() || !elevatorEnterPosition.valid())
          {
            return false;
          }
          double elevatorDistance = pathPlanner->calculatePathCost(currentPosition, elevatorPosition, currentFloor);
          double elevatorEnterDistance = pathPlanner->calculatePathCost(currentPosition, elevatorEnterPosition, currentFloor);
          deviceUtility->foreachSystemRobot([currentFloor, elevatorDistance, elevatorEnterDistance, elevatorPosition, elevatorEnterPosition, robotId, pathPlanner, &shouldElevatorDodge](std::shared_ptr<RobotUtility> robot)
          {
            if (robot->robotId() == robotId || robot->currentFloor() != currentFloor || !robot->localizedSet() || !robot->localized() || robot->disabled())
            {
              return false;
            }
            double robotElevatorDistance = pathPlanner->calculatePathCost(robot->location(), elevatorPosition, currentFloor);
            double robotElevatorEnterDistance = pathPlanner->calculatePathCost(robot->location(), elevatorEnterPosition, currentFloor);
            if (robot->localDestination().distanceOf(elevatorEnterPosition) < 1.0f)
            {
              if (robot->location().distanceOf(elevatorEnterPosition) < 1.0f)
              {
                shouldElevatorDodge = true;
                SPDLOG_INFO("Mission Planner elevator enter dodge with robot {} distance", robot->robotId());
                return true;
              }
              SPDLOG_INFO("Mission Planner elevator enter check dodge with robot {} id", robot->robotId());
              if (robot->robotState() == RobotState::BUSY || RobotState::LIFTING == robot->robotState())
              {
                if (std::abs(elevatorDistance - robotElevatorDistance) > 0.5)
                {
                  if (elevatorDistance > robotElevatorDistance)
                  {
                    shouldElevatorDodge = true;
                    SPDLOG_INFO("Mission Planner elevator dodge with robot {} distance", robot->robotId());
                    return true;
                  }
                }
                else if (std::atoi(robot->robotId().data()) < std::atoi(robotId.data()))
                {
                  shouldElevatorDodge = true;
                  SPDLOG_INFO("Mission Planner elevator dodge with robot {} id", robot->robotId());
                  return true;
                }
              }
            }

            if (robot->localDestination().distanceOf(elevatorPosition) < 1.0f)
            {
              if (robot->location().distanceOf(elevatorPosition) < 1.0f)
              {
                shouldElevatorDodge = true;
                SPDLOG_INFO("Mission Planner elevator dodge with robot {} distance", robot->robotId());
                return true;
              }
              SPDLOG_INFO("Mission Planner elevator check dodge with robot {} id", robot->robotId());
              if (robot->robotState() == RobotState::BUSY || RobotState::LIFTING == robot->robotState())
              {
                if (std::abs(elevatorEnterDistance - robotElevatorEnterDistance) > 0.5)
                {
                  if (elevatorEnterDistance > robotElevatorEnterDistance)
                  {
                    shouldElevatorDodge = true;
                    SPDLOG_INFO("Mission Planner elevator enter dodge with robot {} distance", robot->robotId());
                    return true;
                  }
                }
                else if (std::atoi(robot->robotId().data()) < std::atoi(robotId.data()))
                {
                  shouldElevatorDodge = true;
                  SPDLOG_INFO("Mission Planner elevator dodge with robot {} id", robot->robotId());
                  return true;
                }
              }
            }
            return false;
          });
          return shouldElevatorDodge;
        }

        bool computeCommandWithElevatorPlan(nlohmann::json& command)
        {
          StationModel waypoint;
          std::string preferElevatorId;
          std::vector<std::string> orderTags;
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          if (!command.contains("waypointId") || !deviceUtility->getWaypointInfo(command["waypointId"], waypoint))
          {
            command["reason"] = "目标点不可抵达";
            return false;
          }
          if (command.contains("orderTags"))
          {
            for (auto tagCount = 0; tagCount < command["orderTags"].size(); tagCount++)
            {
              orderTags.push_back(command["orderTags"][tagCount].get<std::string>());
            }
          }
          if (command.contains("elevatorId"))
          {
            preferElevatorId = command["elevatorId"].get<std::string>();
            SPDLOG_INFO("Mission Planner prefer elevator {}", preferElevatorId);
          }

          auto robotPtr = deviceUtility->getRobotInfo();
          auto elevatorPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IElevatorPlanner>();
          auto elevatorPlan = elevatorPlanner->calculateBestElevatorPlan(robotPtr->location(), robotPtr->currentFloor(), waypoint, true, orderTags, preferElevatorId);
          if (elevatorPlan.cost < 0 || elevatorPlan.cost >= 1000)
          {
            command["reason"] = "无可用电梯";
            return false;
          }
          if (!elevatorPlan.planSteps.empty())
          {
            if (command.contains("elevatorId") && command.at("elevatorId").get<std::string>() != elevatorPlan.planSteps[0].elevatorId)
            {
              command["ignoreCommandId"] = command["id"];
              command["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
              command["reason"] = "换梯";
            }
            command["elevatorId"] = elevatorPlan.planSteps[0].elevatorId;
            command["elevatorCost"] = elevatorPlan.cost;
          }
          if (!elevatorPlan.switchElevatorPath.empty())
          {
            command["switchFloor"] = true;
            command["commandType"] = "MOVE";
            command["floorName"] = elevatorPlan.switchElevatorPath[0].elevatorFloor;
            command["coordinates"] = elevatorPlan.switchElevatorPath[0].elevatorEnterLocation.toJson();
            SPDLOG_INFO("Mission Planner elevator switch floor {} coordinate:{}", elevatorPlan.switchElevatorPath[0].elevatorFloor, elevatorPlan.switchElevatorPath[0].elevatorEnterLocation.toJson().dump());
          }
          else
          {
            command = mergeJson(command, waypoint.toJson());
          }
          return true;
        }

        std::string pickBetterChargingStation(const std::string& robotId, const std::vector<std::string> chargingStations)
        {
          std::string stationId;
          std::vector<int> chargingStationAssignment;
          std::vector<std::vector<double>> chargingCostMatrix;
          std::map<std::string, std::vector<std::string>> dismissStation;
          std::map<std::string, int> robotIndex;
          std::map<int, std::string> stationIndex;

          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          deviceUtility->foreachChargingStation([&chargingStations, &stationIndex, &dismissStation, deviceUtility, robotId, self = shared_from_this()](std::shared_ptr<StationModel> station)
          {
            if (find(chargingStations.begin(), chargingStations.end(), station->id()) == chargingStations.end())
            {
              return false;
            }
            deviceUtility->foreachSystemRobot([&dismissStation, robotId, stationPtr = std::weak_ptr<StationModel>(station)](std::shared_ptr<RobotUtility> robot)
            {
              bool shouldDismiss = false;
              if (!robot->chassisPowerSet() || !robot->localizedSet() || !robot->localized())
              {
                return false;
              }
              auto station = stationPtr.lock();
              if (!station->robotOccupyThisStation().expired())
              {
                auto chargingRobot = station->robotOccupyThisStation().lock();
                if (chargingRobot->disabled())
                {
                  SPDLOG_INFO("Mission Planner local dismiss charging station {} with robot {} cause robot disabled", station->name(), robot->robotId());
                  shouldDismiss = true;
                }
              }
              if (robot->robotId() != robotId && robot->robotState() != RobotState::CHARGING && robot->robotState() != RobotState::FAULT)
              {
                if (robot->currentFloor() == station->floor() && robot->localDestination().valid() && station->isInsideOccupyArea(robot->localDestination()))
                {
                  shouldDismiss = true;
                  SPDLOG_INFO("Mission Planner local dodge charging station {} with robot {}", station->name(), robot->robotId());
                }
                if (robot->currentFloor() == station->floor() && robot->platformDestination().valid() && station->isInsideOccupyArea(robot->platformDestination()))
                {
                  shouldDismiss = true;
                  SPDLOG_INFO("Mission Planner cloud dodge charging station {} with robot {}", station->name(), robot->robotId());
                }
              }
              if (shouldDismiss)
              {
                dismissStation[station->id()].push_back(robot->robotId());
              }
              return false;

            });
            if (self->banedStations_.find(station->id()) != self->banedStations_.end())
            {
              if (std::chrono::system_clock::now() > self->banedStations_[station->id()].banningExpireTime)
              {
                self->banedStations_.erase(station->id());
              }
            }
            stationIndex.emplace(stationIndex.size(), station->id());
            return false;
          });

          deviceUtility->foreachSystemRobot([&chargingStations, &chargingCostMatrix, &robotIndex, &stationIndex, dismissStation, banedStations = banedStations_, robotId, deviceUtility, critics = chargingStationCritics_](std::shared_ptr<RobotUtility> robotPtr)
          {
            std::vector<double> stationCostVec;
            if (robotPtr->disabled())
            {
              return false;
            }
            if ((!robotPtr->chassisPowerSet() || !robotPtr->localizedSet() ||
                 !robotPtr->localized() || robotPtr->disabled() || (robotPtr->hiveAttached() && robotPtr->robotId() != robotId)) &&
                RobotState::CHARGING != robotPtr->robotState())
            {
              return false;
            }
            robotIndex.emplace(robotPtr->robotId(), robotIndex.size());
            SPDLOG_INFO("MissionPlanner evaluate charging cost of robot {}", robotPtr->robotId());
            auto robot = *robotPtr;
            for (auto chargingStationId : chargingStations)
            {
              double costSum = 0.0;
              if (!deviceUtility->isStationValid(chargingStationId))
              {
                continue;
              }
              std::shared_ptr<StationModel> stationPtr = deviceUtility->getStationInfo(chargingStationId);
              StationModel station = *stationPtr;

              if (dismissStation.find(station.id()) != dismissStation.end())
              {
                std::vector<std::string> dismissReasonRobot = dismissStation.at(station.id());
                if (std::find(dismissReasonRobot.begin(), dismissReasonRobot.end(), robot.robotId()) == dismissReasonRobot.end())
                {
                  costSum += 500.0;
                }
              }

              if (!station.occupiedRobotId().empty() &&
                  std::find(station.occupiedRobotId().begin(), station.occupiedRobotId().end(), robot.robotId()) == station.occupiedRobotId().end())
              {
                costSum += 500.0;
              }

              if (robot.robotId() == robotId)
              {
                if (banedStations.find(station.id()) != banedStations.end())
                {
                  costSum += 500.0;
                }
              }
              for (auto loopCritic = critics.begin(); loopCritic != critics.end(); ++loopCritic)
              {
                IStationCostFunction* scoreFunction = *loopCritic;
                if (0 == scoreFunction->getScale())
                {
                  continue;
                }
                double cost  = scoreFunction->scoreStation(robot, station);
                SPDLOG_INFO("MissionPlanner charging station {} critic {} cost {}", station.name(), scoreFunction->getName(), cost);
                if (cost < 0 || std::abs(costSum - std::numeric_limits<double>::max()) < 1.0)
                {
                  cost = std::numeric_limits<double>::max() - costSum;
                }
                costSum += cost;
              }
              if (costSum > 1000)
              {
                costSum = 1000;
              }

              SPDLOG_INFO("MissionPlanner charging station {} cost {}", station.name(), costSum);
              stationCostVec.push_back(costSum);
            }
            chargingCostMatrix.push_back(stationCostVec);
            return false;
          });
          if (robotIndex.find(robotId) == robotIndex.end())
          {
            return stationId;
          }
          HungarianAlgorithm::Solve(chargingCostMatrix, chargingStationAssignment);
          for (int count = 0; count < chargingStationAssignment.size(); count++)
          {
            SPDLOG_INFO("Charging allocation result count {}, station {}", count,  chargingStationAssignment[count]);
          }
          SPDLOG_INFO("Charging allocation result robotIndex {}", robotIndex.at(robotId));
          int stationPos = chargingStationAssignment[robotIndex.at(robotId)];
          if (stationIndex.find(stationPos) != stationIndex.end() &&
              chargingCostMatrix[robotIndex.at(robotId)][stationPos] < 1000 &&
              banedStations_.find(stationIndex[stationPos]) == banedStations_.end())
          {
            SPDLOG_INFO("Charging allocation result stationId {}", stationIndex[stationPos]);
            return stationIndex[stationPos];
          }
          return stationId;
        }

        bool isCurrentParkingLocationGood() override
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          bool shouldDodgeParkingRobot = false;
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotPtr = deviceUtility->getRobotInfo();
          auto robotId = robotPtr->robotId();
          if (robotPtr->currentStation().expired())
          {
            return false;
          }
          auto station = robotPtr->currentStation().lock();
          if ("BAY" != station->type())
          {
            return false;
          }

          deviceUtility->foreachSystemRobot([&shouldDodgeParkingRobot, robotId, stationPtr = std::weak_ptr<StationModel>(station)](std::shared_ptr<RobotUtility> robot)
          {
            if (!robot->chassisPowerSet() || !robot->localizedSet() || !robot->localized())
            {
              return false;
            }
            auto station = stationPtr.lock();
            if (robot->robotId() != robotId && robot->robotState() != RobotState::FAULT)
            {
              if (robot->currentFloor() == station->floor() && robot->localDestination().valid() && station->isInsideOccupyArea(robot->localDestination()))
              {
                shouldDodgeParkingRobot = true;
                SPDLOG_INFO("Mission Planner local dodge parking station {} with robot {}", station->name(), robot->robotId());
                return true;
              }
              if (robot->currentFloor() == station->floor() && robot->platformDestination().valid() && station->isInsideOccupyArea(robot->platformDestination()))
              {
                shouldDodgeParkingRobot = true;
                SPDLOG_INFO("Mission Planner cloud dodge parking station {} with robot {}", station->name(), robot->robotId());
                return true;
              }
            }
            return false;
          });
          if (shouldDodgeParkingRobot)
          {
            std::string stationId = station->id();
            addBanedStation(stationId);
            return false;
          }
          return true;
          // if (isDevicePowerGood())
          // {
          //   return true;
          // }
          // std::string currentFloor = systemInfo_.at(robotId_)->currentFloor();
          // for (auto& station : chargingStations_)
          // {
          //   if (currentFloor == station.second->floor())
          //   {
          //     hasChargingStationTheSameFloor = true;
          //     break;
          //   }
          // }
          // if (hasChargingStationTheSameFloor)
          // {
          //   return true;
          // }
          // return false;
        }

        std::string allocateChargingStationToRobot(const std::string& robotId)
        {
          std::string stationId;
          std::vector<int> chargingStationAssignment;
          std::vector<std::vector<double>> chargingCostMatrix;
          std::map<std::string, std::vector<std::string>> dismissStation;
          std::map<std::string, int> robotIndex;
          std::map<int, std::string> stationIndex;

          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          deviceUtility->foreachChargingStation([&stationIndex, &dismissStation, deviceUtility, robotId, self = shared_from_this()](std::shared_ptr<StationModel> station)
          {
            deviceUtility->foreachSystemRobot([&dismissStation, robotId, stationPtr = std::weak_ptr<StationModel>(station)](std::shared_ptr<RobotUtility> robot)
            {
              bool shouldDismiss = false;
              if (!robot->chassisPowerSet() || !robot->localizedSet() || !robot->localized())
              {
                return false;
              }
              auto station = stationPtr.lock();
              if (!station->robotOccupyThisStation().expired())
              {
                auto chargingRobot = station->robotOccupyThisStation().lock();
                if (chargingRobot->disabled())
                {
                  SPDLOG_INFO("Mission Planner local dismiss charging station {} with robot {} cause robot disabled", station->name(), robot->robotId());
                  shouldDismiss = true;
                }
              }
              if (robot->robotId() != robotId && robot->robotState() != RobotState::CHARGING)
              {
                if (robot->currentFloor() == station->floor() && robot->desiredDestination().valid() && station->isInsideOccupyArea(robot->desiredDestination()))
                {
                  shouldDismiss = true;
                  SPDLOG_INFO("Mission Planner local dodge charging station {} with robot {} by desiredDestination", station->name(), robot->robotId());
                }
                if (robot->currentFloor() == station->floor() && robot->localDestination().valid() && station->isInsideOccupyArea(robot->localDestination()))
                {
                  shouldDismiss = true;
                  SPDLOG_INFO("Mission Planner local dodge charging station {} with robot {} by localDestination", station->name(), robot->robotId());
                }
                if (robot->currentFloor() == station->floor() && robot->platformDestination().valid() && station->isInsideOccupyArea(robot->platformDestination()))
                {
                  shouldDismiss = true;
                  SPDLOG_INFO("Mission Planner cloud dodge charging station {} with robot {} by platformDestination", station->name(), robot->robotId());
                }
              }
              if (shouldDismiss)
              {
                dismissStation[station->id()].push_back(robot->robotId());
              }
              return false;
            });
            if (self->banedStations_.find(station->id()) != self->banedStations_.end())
            {
              if (std::chrono::system_clock::now() > self->banedStations_[station->id()].banningExpireTime)
              {
                self->banedStations_.erase(station->id());
              }
            }
            stationIndex.emplace(stationIndex.size(), station->id());
            return false;
          });

          deviceUtility->foreachSystemRobot([&chargingCostMatrix, &robotIndex, &stationIndex, dismissStation, banedStations = banedStations_, robotId, deviceUtility, critics = chargingStationCritics_](std::shared_ptr<RobotUtility> robot)
          {
            std::vector<double> stationCostVec;
            if (robot->disabled())
            {
              return false;
            }
            if ((!robot->chassisPowerSet() || !robot->localizedSet() ||
                !robot->localized() || robot->hiveAttached()) &&
                RobotState::CHARGING != robot->robotState())
            {
              return false;
            }
            robotIndex.emplace(robot->robotId(), robotIndex.size());
            SPDLOG_INFO("MissionPlanner evaluate charging cost of robot {}", robot->robotId());
            deviceUtility->foreachChargingStation([critics, &stationCostVec, robotId, dismissStation, banedStations, robotPtr = std::weak_ptr<RobotUtility>(robot)](std::shared_ptr<StationModel> stationPtr)
            {
              double costSum = 0.0;
              auto robot = *(robotPtr.lock());
              auto station = *stationPtr;
              if (dismissStation.find(station.id()) != dismissStation.end())
              {
                std::vector<std::string>  dismissReasonRobot = dismissStation.at(station.id());
                if (std::find(dismissReasonRobot.begin(), dismissReasonRobot.end(), robot.robotId()) == dismissReasonRobot.end())
                {
                  costSum += 1000.0;
                  SPDLOG_INFO("MissionPlanner charging station {} dismiss cost {}", station.name(), costSum);
                }
              }

              if (!station.occupiedRobotId().empty() &&
                  station.robotOccupyThisStation().expired() &&
                  std::find(station.occupiedRobotId().begin(), station.occupiedRobotId().end(), robot.robotId()) == station.occupiedRobotId().end())
              {
                costSum += 500.0;
                SPDLOG_INFO("MissionPlanner charging station {} station occupy cost {}", station.name(), costSum);
              }

              if (robot.robotId() == robotId)
              {
                if (banedStations.find(stationPtr->id()) != banedStations.end())
                {
                  costSum += 500.0;
                  SPDLOG_INFO("MissionPlanner charging station {} station baned cost {}", station.name(), costSum);
                }
              }
              for (auto loopCritic = critics.begin(); loopCritic != critics.end(); ++loopCritic)
              {
                IStationCostFunction* scoreFunction = *loopCritic;
                if (0 == scoreFunction->getScale())
                {
                  continue;
                }
                double cost  = scoreFunction->scoreStation(robot, station);
                SPDLOG_INFO("MissionPlanner charging station {} critic {} cost {}", station.name(), scoreFunction->getName(), cost);
                if (cost < 0 || std::abs(costSum - std::numeric_limits<double>::max()) < 1.0)
                {
                  cost = std::numeric_limits<double>::max() - costSum;
                }
                costSum += cost;
              }
              if (costSum > 1000)
              {
                costSum = 1000;
              }
              SPDLOG_INFO("MissionPlanner charging station {} cost {}", station.name(), costSum);
              stationCostVec.push_back(costSum);
              return false;
            });
            chargingCostMatrix.push_back(stationCostVec);
            return false;
          });
          if (robotIndex.find(robotId) == robotIndex.end())
          {
            return stationId;
          }
          HungarianAlgorithm::Solve(chargingCostMatrix, chargingStationAssignment);
          for (int count = 0; count < chargingStationAssignment.size(); count++)
          {
            SPDLOG_INFO("Charging allocation result count {}, station {}", count,  chargingStationAssignment[count]);
          }
          SPDLOG_INFO("Charging allocation result robotIndex {}", robotIndex.at(robotId));
          int stationPos = chargingStationAssignment[robotIndex.at(robotId)];
          if (stationIndex.find(stationPos) != stationIndex.end() &&
              chargingCostMatrix[robotIndex.at(robotId)][stationPos] < 500 &&
              banedStations_.find(stationIndex[stationPos]) == banedStations_.end())
          {
            SPDLOG_INFO("Charging allocation result stationId {}", stationIndex[stationPos]);
            return stationIndex[stationPos];
          }
          return stationId;
        }
      public:
        MissionPlanner()
        {
          pathCost_.setScale(1.0);
          powerCost_.setScale(1.0);
          parkingCost_.setScale(1.0);
          chargingCost_.setScale(1.0);
          elevatorCost_.setScale(10.0);
          robotConflictCost_.setScale(1.0);
          elevatorChargingCost_.setScale(0.1);
          stationDistributionCost_.setScale(1.0);
          shouldRebootIdleTimeThreshold_ = 1 * 30;
          shouldRebootUptimeThreshold_ = 2 * 24 * 60;
          rebootUpperTime_ = 2 * 60;
          rebootLowerTime_ = 6 * 60;
          lastOrderOrCommandUpdateSec_ = std::chrono::system_clock::now();

          // parkingStationCritics_.push_back(&pathCost_);
          parkingStationCritics_.push_back(&parkingCost_);
          parkingStationCritics_.push_back(&elevatorCost_);
          parkingStationCritics_.push_back(&robotConflictCost_);
          parkingStationCritics_.push_back(&stationDistributionCost_);

          // chargingStationCritics_.push_back(&pathCost_);
          chargingStationCritics_.push_back(&powerCost_);
          chargingStationCritics_.push_back(&chargingCost_);
          chargingStationCritics_.push_back(&elevatorChargingCost_);
          locationStoragePath_ = std::string(::getenv("HOME")) + "/.ros/" PROJECT_NAME + "/relocateStorage.json";

          if (auto nodeHandle = cti::missionSchedule::common::getContainer()->resolveOrNull<ros::NodeHandle>())
          {
            missionPublisher_ = nodeHandle->advertise<std_msgs::String>("/mission_schedule/mission", 10);
            desiredDestinationPublisher_ = nodeHandle->advertise<std_msgs::String>("/mission_schedule/desired_destination", 10);
            missionResultSubscriber_ = nodeHandle->subscribe("/cloud_scheduling_node/mission_result", 10, &MissionPlanner::onMissionResult, this);
            missionRebootTimeSubscriber_ = nodeHandle->subscribe("/mission_schedule/reboot_time", 10, &MissionPlanner::onMissionRobootTime, this);
            missionForceRebootSubscriber_ = nodeHandle->subscribe("/mission_schedule/force_reboot", 10, &MissionPlanner::onMissionForceReboot, this);
            missionRelocateSubscriber_ = nodeHandle->subscribe("/mission_schedule/relocate", 10, &MissionPlanner::onMissionRelocate, this);
            densityClient_ = nodeHandle->serviceClient<road_control::density_srv>("/road_control/density_info");
            clearGoalPublisher_ = nodeHandle->advertise<std_msgs::String>("/mission_schedule/clear_goal", 10);
            nodeHandle->param("/mission_schedule/partial_density", partialDensity_, true);
            nodeHandle->param("/mission_schedule/density_range", densityRange_, 10.0);
          }

          lastClearGoalFlag_ = true;
          ros::Duration(0.1).sleep();
          // init : pub clear goal to false
          publishClearGoal(false);
        }

        ~MissionPlanner()
        {
        }

        bool isCommandInConsideration(const std::string& commandType)
        {
          if (commandType == "JACK")
          {
            return false;
          }
          return true;
        }

        void onMissionResult(const std_msgs::String& result)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          nlohmann::json missionResult = nlohmann::json::parse(result.data);
          if (!missionResult.contains("commandType") || !isCommandInConsideration(missionResult["commandType"]))
          {
            return;
          }
          SPDLOG_INFO("Mission Planner update mission result {}", result.data);
          // 如果mission result 返回的commandState 为FAILED ，而且需要电梯避让，重新发布一次电梯避让指令
          if (missionResult.contains("dodgeCommandId") && command_.contains("dodgeCommandId") &&
              missionResult.at("dodgeCommandId").get<std::string>() == command_.at("dodgeCommandId").get<std::string>() &&
              missionResult.contains("commandState") && "FAILED" == missionResult.at("commandState").get<std::string>())
          {
            command_.clear();
            return ;
          }
          if (missionResult.contains("commandState") && "FAILED" == missionResult.at("commandState").get<std::string>())
          {
            nlohmann::json compareJson = missionResult;
            compareJson.erase(compareJson.find("commandState"));
            if (compareJson.find("reason") != compareJson.end())
            {
              compareJson.erase(compareJson.find("reason"));
            }
            auto firstCommandDiff = cti::missionSchedule::contrastJson(command_, compareJson);
            auto secondCommandDiff = cti::missionSchedule::contrastJson(compareJson, command_);
            if (firstCommandDiff.empty() && secondCommandDiff.empty())
            {
              command_.clear();
            }
          }
          if (missionResult.contains("commandState") && "COMPLETED" == missionResult.at("commandState").get<std::string>())
          {
            if (missionResult.contains("commandType") && "PICK_UP" == missionResult.at("commandType").get<std::string>())
            {
            }
            else if (missionResult.contains("commandType") && "DROP_OFF" == missionResult.at("commandType").get<std::string>())
            {
            }
          }
          auto missionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>();
          if (!missionResult.contains("orderId"))
          {
            missionCenter->updatePlatformMissionState(missionResult);
            return;
          }
          std::string missionId = missionResult.at("id").get<std::string>();
          std::string orderId = missionResult.at("orderId").get<std::string>();
          std::string missionState = missionResult.at("commandState").get<std::string>();
          for (auto orderIter = localScheduleOrders_.begin(); orderIter != localScheduleOrders_.end();)
          {
            if (!isOrderStateOnFinished((*orderIter)->orderState()))
            {
              (*orderIter)->updateOrderProgress(missionResult);
              SPDLOG_INFO("Mission Planner update local order {} to state {}", (*orderIter)->id(), ScheduleOrder::toString((*orderIter)->orderState()));
            }
            if (isOrderStateOnFinished((*orderIter)->orderState()))
            {
              if (currentLocalOrder_)
              {
                if ((*orderIter)->orderState() == OrderState::FAILED && (*orderIter)->id() == currentLocalOrder_->id())
                {
                  std::string stationId = (*orderIter)->targetStationId();
                  addBanedStation(stationId);
                }
                if ((*orderIter)->id() == currentLocalOrder_->id())
                {
                  currentLocalOrder_.reset();
                }
              }
              (*orderIter).reset();
              localScheduleOrders_.erase(orderIter++);
            }
            else
            {
              orderIter++;
            }
          }
          missionCenter->updatePlatformMissionState(missionResult);
          lastOrderOrCommandUpdateSec_ = std::chrono::system_clock::now();
        }

        void onMissionRelocate(const mission_schedule::coordinatesMsg& msg)
        {
          Position location;
          std_msgs::String missionMsg;
          if (fabs(msg.x) > 1e-15 || fabs(msg.y) > 1e-15 || fabs(msg.yaw) > 1e-15)
          {
            location.shift(msg.x, msg.y, msg.yaw);
          }
          nlohmann::json relocateJson = computeRelocateCommand(location, msg.floorName, msg.buildingName);
          if (relocateJson.empty())
          {
            return;
          }
          SPDLOG_INFO("MissionPlanner relocate command {}", relocateJson.dump());
          publishRelocateCommand(relocateJson);
        }

        void onMissionForceReboot(const mission_schedule::coordinatesMsg& msg)
        {
          nlohmann::json command;
          std_msgs::String missionMsg;
          double angle = 0;
          Position location;
          if (fabs(msg.x) > 1e-15 || fabs(msg.y) > 1e-15 || fabs(msg.yaw) > 1e-15)
          {
            location.shift(msg.x, msg.y, msg.yaw);
          }
          storeRelocateToLocalStorage(location, msg.floorName, msg.buildingName);
          command["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
          command["commandType"] = "RESTART";
          command["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
          // unsigned int durationNum = std::atoi(result.data.data());
          command["duration"] = 5 * 60000;
          missionMsg.data = command.dump();
          missionPublisher_.publish(missionMsg);
          SPDLOG_INFO("MissionPlanner publish restart command {}", missionMsg.data);
        }

        void onMissionRobootTime(const mission_schedule::rebootTimeMsg::ConstPtr& msg)
        {
          SPDLOG_INFO("Mission Planner before: shouldRebootUptime:{}(min) rebootUpperTime:{}(min) rebootLowerTime_:{}(min) shouldRebootIdleTime:{}(min)",
                      shouldRebootUptimeThreshold_, rebootUpperTime_, rebootLowerTime_, shouldRebootIdleTimeThreshold_);
          if (msg->upTime != -1000)
          {
            shouldRebootUptimeThreshold_ = msg->upTime;
          }
          if (msg->upperTime != -1000)
          {
            rebootUpperTime_ = msg->upperTime;
          }
          if (msg->lowerTime != -1000)
          {
            rebootLowerTime_ = msg->lowerTime;
          }
          if (msg->idleTime != -1000)
          {
            shouldRebootIdleTimeThreshold_ = msg->idleTime;
          }
          SPDLOG_INFO("Mission Planner after: shouldRebootUptime:{}(min) rebootUpperTime:{}(min) rebootLowerTime_:{}(min) shouldRebootIdleTime:{}(min)",
                      shouldRebootUptimeThreshold_, rebootUpperTime_, rebootLowerTime_, shouldRebootIdleTimeThreshold_);
        }

        bool addBanedStation(std::string& stationId)
        {
          if (!stationId.empty())
          {
            BanedStation banedStation;
            banedStation.stationId = stationId;
            banedStation.banningStartTime = std::chrono::system_clock::now();
            banedStation.banningExpireTime = std::chrono::system_clock::now() + std::chrono::minutes(10);
            banedStations_[banedStation.stationId] = banedStation;
            SPDLOG_INFO("MissionPlanner ban station : {}", banedStation.stationId);
            return true;
          }
          return false;
        }

        bool isRobotFreeOfSystemPowerManagement()
        {
          bool needDodge = false;
          std::string chargingStationId;
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotInfo = deviceUtility->getRobotInfo();
          if (!deviceUtility->isDevicePowerGood() || RobotState::CHARGING == robotInfo->robotState())
          {
            return false;
          }
          chargingStationId = allocateChargingStationToRobot(robotInfo->robotId());
          if (RobotState::CHARGING != robotInfo->robotState() && deviceUtility->isDevicePowerGood())
          {
            return true;
          }
          return needDodge || chargingStationId.empty();
        }

        bool broadcastDesiredDestination()
        {
          nlohmann::json msgJson;
          std_msgs::String msgData;
          msgJson["desiredDestination"] = desiredDestination_.toJson();
          msgData.data = msgJson.dump();
          desiredDestinationPublisher_.publish(msgData);
          if (desiredDestination_.valid())
          {
            desiredDestination_.reset();
          }
          return true;
        }

        bool findParkingLocation()
        {
          double bestScore = -1;
          StationModel bestParking;
          std::string lastParkingStationId;
          for (auto loopCritic = parkingStationCritics_.begin(); loopCritic != parkingStationCritics_.end(); ++loopCritic)
          {
            IStationCostFunction* scoreFunction = *loopCritic;
            if (!scoreFunction->prepare())
            {
              return false;
            }
          }
          if (currentLocalOrder_ && currentLocalOrder_->orderType() == OrderType::ROBOT_RETURN)
          {
            lastParkingStationId = currentLocalOrder_->targetStationId();
          }

          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotId = deviceUtility->getRobotInfo()->robotId();
          deviceUtility->foreachParkingStation([&bestScore, &bestParking, lastParkingStationId, robotId, self = shared_from_this(), critics = parkingStationCritics_](std::shared_ptr<StationModel> parking)
          {
            auto station = *parking;
            SPDLOG_INFO("MissionPlanner evaluate parking station {} id {}", station.name(), station.id());
            double stationCost = 0;
            if (!parking->robotOccupyThisStation().expired())
            {
              return false;
            }
            if (self->banedStations_.find(parking->id()) != self->banedStations_.end())
            {
              if (std::chrono::system_clock::now() > self->banedStations_[parking->id()].banningExpireTime)
              {
                self->banedStations_.erase(parking->id());
              }
              else
              {
                return false;
              }
            }

            if (!station.occupiedRobotId().empty() &&
                std::find(station.occupiedRobotId().begin(), station.occupiedRobotId().end(), robotId) == station.occupiedRobotId().end())
            {
              stationCost += 500;
              SPDLOG_INFO("MissionPlanner parking station {} occupy cost {}", station.name(), stationCost);
            }

            if (parking->id() != lastParkingStationId && !lastParkingStationId.empty())
            {
              stationCost += 10;
              SPDLOG_INFO("MissionPlanner parking station {} last station cost {}", station.name(), stationCost);
            }

            for (auto loopCritic = critics.begin(); loopCritic != critics.end(); ++loopCritic)
            {
              IStationCostFunction* scoreFunction = *loopCritic;
              if (0 == scoreFunction->getScale())
              {
                continue;
              }
              double cost = scoreFunction->scoreStation(station);
              SPDLOG_INFO("MissionPlanner parking station {} critic {} cost {}", station.name(), scoreFunction->getName(), cost);
              if (cost >= 0)
              {
                stationCost += cost * scoreFunction->getScale();
              }
              else
              {
                stationCost = cost;
                break;
              }
              if (bestScore > 0 && stationCost > bestScore)
              {
                break;
              }
            }
            SPDLOG_INFO("MissionPlanner parking station {} cost {}", station.name(), stationCost);
            if (stationCost >= 0 && (bestScore < 0 || stationCost < bestScore))
            {
              bestScore = stationCost;
              bestParking = station;
            }
            return false;
          });
          if (bestScore >= 0 && bestScore < 500)
          {
            nlohmann::json orderJson;
            orderJson["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
            orderJson["shouldPickHive"] = false;
            orderJson["transitMode"] = "ROBOT_RETURN";
            orderJson["endWaypointId"] = bestParking.id();
            orderJson["orderScheduleType"] = "PARKING";
            orderJson["contexts"] = nlohmann::json::array();
            nlohmann::json contextJson;
            contextJson["contextType"] = "SCHEDULING";
            contextJson["context"] = orderJson["id"];
            orderJson["contexts"].push_back(contextJson);
            decisionOrderJson_ = orderJson;
            SPDLOG_INFO("MissionPlanner best parking station {} id {}", bestParking.name(), bestParking.id());
          }
          else
          {
            SPDLOG_INFO("MissionPlanner can't find suitable parking station");
          }
          return bestScore >= 0 && bestScore < 500;
        }

        bool pickPlatformMission()
        {
          auto missionCenter = cti::missionSchedule::common::getContainer()->resolve<IPlatformMissionCenter>();
          nlohmann::json newCommand = missionCenter->computeOrderCommand();
          if ((newCommand.empty() || newCommand.contains("useMock")) || !newCommand.contains("waypointId"))
          {
            return false;
          }
          return true;
        }

        bool findChargingLocation()
        {
          StationModel waypoint;
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotInfo = deviceUtility->getRobotInfo();
          auto chargingStationId = allocateChargingStationToRobot(robotInfo->robotId());
          if (!deviceUtility->isStationValid(chargingStationId) && RobotState::CHARGING != robotInfo->robotState())
          {
            return false;
          }
          if (!chargingStationId.empty())
          {
            // if (deviceUtility->isDevicePowerGood(robotInfo->robotId()))
            // {
              if (deviceUtility->isStationOccupied(chargingStationId) && deviceUtility->getWaypointInfo(chargingStationId, waypoint))
              {
                auto occupyRobotId = deviceUtility->getStationOccupiedRobotId(chargingStationId);
                if (robotInfo->robotId() != occupyRobotId && !occupyRobotId.empty() &&
                    robotInfo->currentFloor() == waypoint.floor())
                {
                  return false;
                }
              }
            // }
            nlohmann::json orderJson;
            orderJson["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
            orderJson["shouldPickHive"] = false;
            orderJson["transitMode"] = "ROBOT_CHARGE";
            orderJson["endWaypointId"] = chargingStationId;
            orderJson["contexts"] = nlohmann::json::array();
            nlohmann::json contextJson;
            contextJson["contextType"] = "SCHEDULING";
            contextJson["context"] = orderJson["id"];
            orderJson["contexts"].push_back(contextJson);
            if (!deviceUtility->getStationDockId(chargingStationId).empty())
            {
              orderJson["targetDockId"] = deviceUtility->getStationDockId(chargingStationId);
            }
            orderJson["orderScheduleType"] = "CHARGING";
            decisionOrderJson_ = orderJson;
          }
          else if (chargingStationId.empty() && RobotState::CHARGING == robotInfo->robotState() && !shouldRobotAvoidDock())
          {
            return true;
          }
          return !chargingStationId.empty();
        }

        bool isSwitchFloorNeed()
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          std::string currentFloor = deviceUtility->getRobotInfo()->currentFloor();
          if (command_.contains("floorName") && command_.at("floorName") != currentFloor)
          {
            return true;
          }
          return false;
        }

        bool allocateElevatorPlan()
        {
          return true;
        }

        bool resetRecoveryFlag()
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          if (stillCommandTransmited_)
          {
            stillCommandTransmited_ = !stillCommandTransmited_;
          }
          if (recovercommandtransmited_)
          {
            recovercommandtransmited_ = !recovercommandtransmited_;
          }
          return true;
        }

        bool executeRobotStillBehavior()
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotInfo = deviceUtility->getRobotInfo();
          if (RobotState::IDLE == robotInfo->robotState() || (RobotState::CHARGING == robotInfo->robotState() && !robotInfo->currentStation().expired()))
          {
            return true;
          }
          nlohmann::json command;
          std_msgs::String missionMsg;
          auto id = boost::uuids::to_string(boost::uuids::random_generator()());
          command["id"] = id;
          command["coopMode"] = "LOCAL";
          command["commandType"] = "ABORT";
          command["scheduleType"] = "ROBOT_STILL";
          command["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
          std::chrono::system_clock::time_point currentSec = std::chrono::system_clock::now();
          int lastAbortDuration = std::chrono::duration_cast<std::chrono::seconds>(currentSec - lastAbortPublishSec_).count();
          if (!stillCommandTransmited_ || lastAbortDuration > 9)
          {
            command_ = command;
            missionMsg.data = command.dump();
            missionPublisher_.publish(missionMsg);
            lastAbortPublishSec_ = std::chrono::system_clock::now();
            stillCommandTransmited_ = !stillCommandTransmited_ ;
            SPDLOG_INFO("MissionPlanner still behavior publish command {}", missionMsg.data);
            return true;
          }
          return false;
        }

        bool executeRobotRecoverBehavior()
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          nlohmann::json command;
          std_msgs::String missionMsg;
          auto id = boost::uuids::to_string(boost::uuids::random_generator()());
          command["id"] = id;
          command["coopMode"] = "LOCAL";
          command["commandType"] = "ABORT";
          command["scheduleType"] = "ROBOT_RECOVERY";
          command["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
          if (!recovercommandtransmited_)
          {
            command_ = command;
            missionMsg.data = command.dump();
            missionPublisher_.publish(missionMsg);
            recovercommandtransmited_ = !recovercommandtransmited_;
            SPDLOG_INFO("MissionPlanner recovery behavior publish command {}", missionMsg.data);
            return true;
          }
          return false;
        }

        bool executeCloudDirectCommand()
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          auto missionCenter = cti::missionSchedule::common::getContainer()->resolve<IPlatformMissionCenter>();
          nlohmann::json command = missionCenter->computeCloudDirectCommand();

          ROS_INFO("[density]executeCloudDirectCommand :  command size: %d, Data: %s", command.size(), command.dump().c_str());
          SPDLOG_INFO("[density]executeCloudDirectCommand : command size: {}, Data: {}", command.size(), command.dump());

          if (!command.empty() && command.contains("moveable") && command.contains("id"))
          {
            auto moveable = command["moveable"].get<bool>();
            auto commandId = command["id"].get<std::string>();
            SPDLOG_INFO("[density]executeCloudDirectCommand : moveable: {}, lastMoveable: {}", moveable, lastMoveable_);
            if (!moveable)
            {
              publishAbortCommand("", true);
              auto newCommand = command;
              newCommand["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
              newCommand["commandState"] = "PENDING";
              missionCenter->updatePlatformCommand(commandId, newCommand);
              lastMoveable_ = moveable;
              return false;
            }
            else
            {
              lastMoveable_ = moveable;
              publishClearGoal(false);
            }

          }
          else if (!command.empty() && command.contains("commandType") && command["commandType"] == "ABORT")
          {
            publishClearGoal(true);
          }
          else if (!command.empty() && command.contains("coordinates") && command.contains("id") && 
              command.contains("commandType") && command["commandType"] == "MOVE")
          {
            auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
            auto robotPtr = deviceUtility->getRobotInfo();
            Position coordinate{};
            coordinate.parseFromJson(command.at("coordinates"));
            auto densityRes = callDensityServer(robotPtr, coordinate);
            bool moveable = densityRes.moveable;
            auto resdata = densityRes.data;
            auto commandId = command["id"].get<std::string>();
            
            SPDLOG_INFO("[density]executeCloudDirectCommand : moveable: {}, lastMoveable: {}", moveable, lastMoveable_);
            if (!moveable)
            {
              publishAbortCommand("", true);
              auto newCommand = command;
              newCommand["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
              newCommand["commandState"] = "PENDING";
              missionCenter->updatePlatformCommand(commandId, newCommand);
              lastMoveable_ = moveable;
              return false;
            }
            else
            {
              lastMoveable_ = moveable;
              publishClearGoal(false);
            }
          }

          if (!command.empty() && command.contains("queueing") && command["queueing"].get<bool>())
          {
            std_msgs::String missionMsg;
            command_ = command;
            command["commandState"] = "STARTUP";
            // currentLocalOrder_.reset();
            missionMsg.data = command_.dump();
            missionCenter->updatePlatformMissionState(command);
            missionPublisher_.publish(missionMsg);
            plannerState_ = ScheduleState::PLATFORM_RUNNING;
            SPDLOG_INFO("[density]MissionPlanner Cloud direct publish command {}", missionMsg.data);
            SPDLOG_INFO("MissionPlanner Cloud direct publish command {}", missionMsg.data);
          }
          return true;
        }

        bool executePlatformMission()
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          StationModel waypoint;
          std_msgs::String missionMsg;
          auto missionCenter = cti::missionSchedule::common::getContainer()->resolve<IPlatformMissionCenter>();
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotPtr = deviceUtility->getRobotInfo();
          auto elevatorPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IElevatorPlanner>();
          if ((deviceUtility->isPositionNearNarrowArea(robotPtr->location()) ||
               elevatorPlanner->isRobotLocationOccupyElevator(robotPtr->currentFloor(), robotPtr->location())) &&
              RobotState::IDLE != robotPtr->robotState())
          {
            return true;
          }

          nlohmann::json command = missionCenter->computeOrderCommand();   // get command

          if (!command.empty() && command.contains("moveable"))
          {
            bool moveable = command["moveable"].get<bool>();
            SPDLOG_INFO("executePlatformMission[density]: moveable: {}, lastMoveable: {}", moveable, lastMoveable_);
            if (!moveable)
            {
              publishAbortCommand("", true);
              lastMoveable_ = moveable;
              return true;
            }
            lastMoveable_ = moveable;
            publishClearGoal(false);
          }
          if (command.empty() || command.contains("useMock"))
          {
            if (command.contains("useMock"))
            {
              SPDLOG_INFO("executePlatformMission[density]: use mock command.");
            }
            command.clear();
            SPDLOG_INFO("executePlatformMission[density]: command clear, emptied[{}]", command.empty());
          }

          if (!command.empty() && command.contains("orderId") && command.contains("waypointId") && 
            command.contains("shouldDodge") && command["shouldDodge"].get<bool>())
          {
            std::string dodgeWaypointId = findDodgeWaypoint(command["waypointId"]);
            if (!dodgeWaypointId.empty())
            {
              SPDLOG_INFO("MissionPlanner target dodge waypointId {}", dodgeWaypointId);
              StationModel waypoint;
              auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
              if (deviceUtility->getWaypointInfo(dodgeWaypointId, waypoint))
              {
                nlohmann::json dodgeCommand = command;
                dodgeCommand["commandType"] = "MOVE";
                dodgeCommand["horizontalOffset"] = 0.0;
                if (dodgeCommand.find("id") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("id"));
                }
                if (dodgeCommand.find("orderId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("orderId"));
                }
                if (dodgeCommand.find("elevatorId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("elevatorId"));
                }
                if (dodgeCommand.find("floorId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("floorId"));
                }
                if (dodgeCommand.find("floorName") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("floorName"));
                }
                if (dodgeCommand.find("waypointId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("waypointId"));
                }
                if (dodgeCommand.find("waypointType") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("waypointType"));
                }
                if (dodgeCommand.find("coordinates") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("coordinates"));
                }
                if (dodgeCommand.find("sterilizationMovement") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("sterilizationMovement"));
                }
                dodgeCommand = mergeJson(dodgeCommand, waypoint.toJson());
                dodgeCommand["coopMode"] = "LOCAL";
                dodgeCommand["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
                dodgeCommand["dodgeCommandId"] = boost::uuids::to_string(boost::uuids::random_generator()());
                SPDLOG_INFO("MissionPlanner target dodge origin command {}", dodgeCommand.dump());
                if (!dodgeCommand.empty() && isCommandChanged(dodgeCommand))
                {
                  command_ = dodgeCommand;
                  dodgeCommand["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
                  missionMsg.data = dodgeCommand.dump();
                  missionPublisher_.publish(missionMsg);
                  SPDLOG_INFO("MissionPlanner target dodge publish command {}", missionMsg.data);
                }
                return true;
              }
            }
          }

          if (command.empty() &&
              command_.contains("orderId") &&
              command_.contains("elevatorId") &&
              command_.contains("scheduleType") &&
              command_["scheduleType"] == "PLATFORM")
          {
            command = command_;
          }
          if (command.contains("orderId") && !computeCommandWithElevatorPlan(command))
          {
            command["commandState"] = "DIRECT_FAILED";
            missionCenter->updatePlatformMissionState(command);
            return false;
          }

          std::string dodgeRobotId;
          if (command.contains("orderId") && command.contains("waypointId") && shouldDestinationDodge(command["waypointId"], dodgeRobotId, false))
          {
            desiredDestination_.parseFromJson(command["coordinates"]);
            if (!dodgeRobotId.empty() && isReciprocalDestinationOccupied(command["waypointId"], dodgeRobotId))
            {
              StationModel waypoint;
              std::string dodgeWaypointId = findBestDodgeStation(command["waypointId"]);
              if (!dodgeWaypointId.empty() && deviceUtility->getWaypointInfo(dodgeWaypointId, waypoint))
              {
                nlohmann::json dodgeCommand = command;
                dodgeCommand["commandType"] = "MOVE";
                dodgeCommand["horizontalOffset"] = 0.0;
                if (dodgeCommand.find("id") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("id"));
                }
                if (dodgeCommand.find("orderId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("orderId"));
                }
                if (dodgeCommand.find("elevatorId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("elevatorId"));
                }
                if (dodgeCommand.find("floorId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("floorId"));
                }
                if (dodgeCommand.find("floorName") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("floorName"));
                }
                if (dodgeCommand.find("waypointId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("waypointId"));
                }
                if (dodgeCommand.find("waypointType") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("waypointType"));
                }
                if (dodgeCommand.find("coordinates") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("coordinates"));
                }
                if (dodgeCommand.find("sterilizationMovement") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("sterilizationMovement"));
                }
                dodgeCommand = mergeJson(dodgeCommand, waypoint.toJson());
                dodgeCommand["coopMode"] = "LOCAL";
                dodgeCommand["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
                dodgeCommand["dodgeOriginCoordinate"] = robotPtr->location().toJson();
                dodgeCommand["dodgeCommandId"] = boost::uuids::to_string(boost::uuids::random_generator()());
                SPDLOG_INFO("MissionPlanner reciprocal dodge origin command {}", dodgeCommand.dump());
                if (!dodgeCommand.empty() && isCommandChanged(dodgeCommand))
                {
                  command_ = dodgeCommand;
                  dodgeCommand["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
                  missionMsg.data = dodgeCommand.dump();
                  missionPublisher_.publish(missionMsg);
                  SPDLOG_INFO("MissionPlanner reciprocal dodge publish command {}", missionMsg.data);
                }
                return true;
              }
            }
          }

          if (command_.contains("dodgeOriginCoordinate"))
          {
            Position dodgeOriginPosition;
            dodgeOriginPosition.parseFromJson(command_["dodgeOriginCoordinate"]);
            if (dodgeOriginPosition.valid() && robotPtr->location().distanceOf(dodgeOriginPosition) > 3.0f)
            {
              publishAbortCommand();
            }
            return true;
          }

          if (command.contains("orderId") && command.contains("waypointId") && shouldDestinationDodge(command["waypointId"], dodgeRobotId))
          {
            if (robotPtr->robotState() != RobotState::CHARGING)
            {
              publishAbortCommand();
            }
            return true;
          }
          if (command.contains("orderId") && command.contains("elevatorId") && command.contains("waypointId") &&
              shouldElevatorDodge(command["elevatorId"], command["waypointId"]))
          {
            std::string dodgeWaypointId;
            if (findElevatorDodgeWaypoint(command["elevatorId"], dodgeWaypointId))
            {
              SPDLOG_INFO("MissionPlanner Elevator dodge waypointId {}", dodgeWaypointId);
              StationModel waypoint;
              auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
              if (deviceUtility->getWaypointInfo(dodgeWaypointId, waypoint))
              {
                nlohmann::json dodgeCommand = command;
                dodgeCommand["commandType"] = "MOVE";
                dodgeCommand["horizontalOffset"] = 0.0;
                if (dodgeCommand.find("id") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("id"));
                }
                if (dodgeCommand.find("orderId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("orderId"));
                }
                if (dodgeCommand.find("elevatorId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("elevatorId"));
                }
                if (dodgeCommand.find("floorId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("floorId"));
                }
                if (dodgeCommand.find("floorName") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("floorName"));
                }
                if (dodgeCommand.find("waypointId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("waypointId"));
                }
                if (dodgeCommand.find("waypointType") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("waypointType"));
                }
                if (dodgeCommand.find("coordinates") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("coordinates"));
                }
                if (dodgeCommand.find("sterilizationMovement") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("sterilizationMovement"));
                }
                dodgeCommand = mergeJson(dodgeCommand, waypoint.toJson());
                dodgeCommand["coopMode"] = "LOCAL";
                dodgeCommand["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
                dodgeCommand["dodgeCommandId"] = boost::uuids::to_string(boost::uuids::random_generator()());
                SPDLOG_INFO("MissionPlanner Elevator dodge origin command {}", dodgeCommand.dump());
                if (!dodgeCommand.empty() && isCommandChanged(dodgeCommand))
                {
                  command_ = dodgeCommand;
                  dodgeCommand["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
                  missionMsg.data = dodgeCommand.dump();
                  missionPublisher_.publish(missionMsg);
                  SPDLOG_INFO("MissionPlanner Elevator dodge publish command {}", missionMsg.data);
                }
                return true;
              }
            }
            if (robotPtr->robotState() != RobotState::CHARGING)
            {
              publishAbortCommand();
            }
            return true;
          }
          if (!command.empty() && command.contains("id") && command_.contains("id") &&
              command_.contains("orderId") && command.contains("orderId") && command_["orderId"] == command["orderId"] &&
              command.contains("skipSterilizationMovement") && command["skipSterilizationMovement"].get<bool>() &&
              command.contains("commandType") && command_.contains("commandType") && command["commandType"].get<std::string>() == "MOVE")
          {
            command["skipedCommandId"] = command_["id"];
          }
          else if (command.find("skipSterilizationMovement") != command.end())
          {
            command.erase(command.find("skipSterilizationMovement"));
          }
          if (!command.empty() && isCommandChanged(command))
          {
            if (command.contains("orderId"))
            {
              command["coopMode"] = "LOCAL";
              command["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
            }
            if (command.contains("orderId") && command.contains("elevatorId") && command.contains("elevatorCost"))
            {
              missionCenter->reportElevatorAllocationResult(command["orderId"], command["elevatorId"], command["elevatorCost"]);
            }
            command_ = command;
            command["commandState"] = "QUEUEING";
            // currentLocalOrder_.reset();
            missionMsg.data = command_.dump();
            missionCenter->updatePlatformMissionState(command);
            missionPublisher_.publish(missionMsg);
            plannerState_ = ScheduleState::PLATFORM_RUNNING;
            SPDLOG_INFO("MissionPlanner Cloud publish command {}", missionMsg.data);
          }
          return true;
        }

        bool isChargingTobeContinue()
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          if (!currentLocalOrder_)
          {
            return false;
          }
          if (currentLocalOrder_->orderType() == OrderType::ROBOT_CHARGE)
          {
            return currentLocalOrder_->robotChargingContinue();
          }
          return false;
        }

        bool executeChargingMission()
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          StationModel waypoint;
          std_msgs::String missionMsg;
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto missionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>();
          auto robotInfo = deviceUtility->getRobotInfo();
          if (RobotState::CHARGING == robotInfo->robotState() && !robotInfo->currentStation().expired())
          {
            auto robotChargingStation = robotInfo->currentStation().lock();
            if (robotChargingStation->id() == decisionOrderJson_["endWaypointId"])
            {
              return true;
            }
          }

          auto elevatorPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IElevatorPlanner>();
          if ((deviceUtility->isPositionNearNarrowArea(robotInfo->location()) ||
               elevatorPlanner->isRobotLocationOccupyElevator(robotInfo->currentFloor(), robotInfo->location())) &&
              RobotState::IDLE != robotInfo->robotState())
          {
            return true;
          }

          if (!currentLocalOrder_ || currentLocalOrder_->orderType() != OrderType::ROBOT_CHARGE ||
              currentLocalOrder_->targetStationId() != decisionOrderJson_["endWaypointId"])
          {
            std::string localOrderResponse, errorMessage;
            bool createOk = missionCenter->uploadLocalScheduleOrder(decisionOrderJson_, localOrderResponse, errorMessage);
            if (!localOrderResponse.empty())
            {
              nlohmann::json orderJson = nlohmann::json::parse(localOrderResponse);
              if (orderJson.contains("rejected") && orderJson["rejected"])
              {
                std::string stationId = decisionOrderJson_["endWaypointId"];
                addBanedStation(stationId);
                return true;
              }
              if (orderJson.contains("id") && !orderJson.at("id").get<std::string>().empty())
              {
                decisionOrderJson_["id"] = orderJson["id"];
                if (decisionOrderJson_.find("contexts") != decisionOrderJson_.end())
                {
                  decisionOrderJson_.erase(decisionOrderJson_.find("contexts"));
                }
                nlohmann::json contextJson;
                contextJson["contextType"] = "SCHEDULING";
                contextJson["context"] = orderJson["id"];
                decisionOrderJson_["contexts"].push_back(contextJson);
              }
              SPDLOG_INFO("MissionPlanner report response {}", localOrderResponse);
            }
            localScheduleOrders_.push_back(ScheduleOrder::parse(decisionOrderJson_));
            currentLocalOrder_ = localScheduleOrders_.back();
          }
          if (isOrderStateOnFinished(currentLocalOrder_->orderState()))
          {
            return true;
          }
          auto command = currentLocalOrder_->computeOrderExecuteCommand();
          if ((command.empty() || command.contains("useMock")) &&
              command_.contains("orderId") &&
              command_.contains("elevatorId") &&
              command_.contains("scheduleType") &&
              command_["scheduleType"] == "CHARGING")
          {
            command = command_;
          }
          if (command.contains("orderId") && !computeCommandWithElevatorPlan(command))
          {
            return false;
          }

          std::string dodgeRobotId;
          if (command.contains("orderId") && command.contains("waypointId") && shouldDestinationDodge(command["waypointId"], dodgeRobotId))
          {
            publishAbortCommand();
            return true;
          }

          if (command.contains("orderId") && command.contains("elevatorId") && command.contains("waypointId") &&
              shouldElevatorDodge(command["elevatorId"], command["waypointId"]))
          {
            std::string dodgeWaypointId;
            if (findElevatorDodgeWaypoint(command["elevatorId"], dodgeWaypointId))
            {
              StationModel waypoint;
              auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
              if (deviceUtility->getWaypointInfo(dodgeWaypointId, waypoint))
              {
                nlohmann::json dodgeCommand = command;
                dodgeCommand["commandType"] = "MOVE";
                dodgeCommand["horizontalOffset"] = 0.0;
                if (dodgeCommand.find("id") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("id"));
                }
                if (dodgeCommand.find("orderId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("orderId"));
                }
                if (dodgeCommand.find("elevatorId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("elevatorId"));
                }
                if (dodgeCommand.find("floorId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("floorId"));
                }
                if (dodgeCommand.find("floorName") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("floorName"));
                }
                if (dodgeCommand.find("waypointId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("waypointId"));
                }
                if (dodgeCommand.find("waypointType") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("waypointType"));
                }
                if (dodgeCommand.find("coordinates") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("coordinates"));
                }
                if (dodgeCommand.find("sterilizationMovement") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("sterilizationMovement"));
                }
                dodgeCommand = mergeJson(dodgeCommand, waypoint.toJson());
                dodgeCommand["coopMode"] = "LOCAL";
                dodgeCommand["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
                dodgeCommand["dodgeCommandId"] = boost::uuids::to_string(boost::uuids::random_generator()());
                if (!dodgeCommand.empty() && isCommandChanged(dodgeCommand))
                {
                  command_ = dodgeCommand;
                  dodgeCommand["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
                  missionMsg.data = dodgeCommand.dump();
                  missionPublisher_.publish(missionMsg);
                  SPDLOG_INFO("MissionPlanner Elevator dodge publish command {}", missionMsg.data);
                }
                return true;
              }
            }
            publishAbortCommand();
            return true;
          }
          if (!command.empty() && isCommandChanged(command))
          {
            command["coopMode"] = "LOCAL";
            command["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
            command_ = command;
            command["commandState"] = "QUEUEING";
            missionMsg.data = command_.dump();
            if (command.contains("orderId") && command.contains("elevatorId") && command.contains("elevatorCost"))
            {
              missionCenter->reportElevatorAllocationResult(command["orderId"], command["elevatorId"], command["elevatorCost"]);
            }
            currentLocalOrder_->updateOrderProgress(command);
            missionPublisher_.publish(missionMsg);
            plannerState_ = ScheduleState::LOCAL_RUNNING;
            SPDLOG_INFO("MissionPlanner Charging publish command {}", missionMsg.data);
          }
          return true;
        }

        bool executeParkingMission()
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          StationModel waypoint;
          std_msgs::String missionMsg;
          auto missionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>();
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotPtr = deviceUtility->getRobotInfo();
          if (!currentLocalOrder_ || currentLocalOrder_->orderType() != OrderType::ROBOT_RETURN ||
              currentLocalOrder_->targetStationId() != decisionOrderJson_["endWaypointId"])
          {
            std::string localOrderResponse, errorMessage;
            missionCenter->uploadLocalScheduleOrder(decisionOrderJson_, localOrderResponse, errorMessage);
            if (!localOrderResponse.empty())
            {
              nlohmann::json orderJson = nlohmann::json::parse(localOrderResponse);
              if (orderJson.contains("id") && !orderJson.at("id").get<std::string>().empty())
              {
                decisionOrderJson_["id"] = orderJson["id"];
                if (decisionOrderJson_.find("contexts") != decisionOrderJson_.end())
                {
                  decisionOrderJson_.erase(decisionOrderJson_.find("contexts"));
                }
                nlohmann::json contextJson;
                contextJson["contextType"] = "SCHEDULING";
                contextJson["context"] = orderJson["id"];
                decisionOrderJson_["contexts"].push_back(contextJson);
              }
              SPDLOG_INFO("MissionPlanner report response {}", localOrderResponse);
            }
            localScheduleOrders_.push_back(ScheduleOrder::parse(decisionOrderJson_));
            currentLocalOrder_ = localScheduleOrders_.back();
          }
          if (isOrderStateOnFinished(currentLocalOrder_->orderState()))
          {
            return true;
          }
          auto elevatorPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IElevatorPlanner>();
          if ((deviceUtility->isPositionNearNarrowArea(robotPtr->location()) ||
               elevatorPlanner->isRobotLocationOccupyElevator(robotPtr->currentFloor(), robotPtr->location())) &&
              RobotState::IDLE != robotPtr->robotState())
          {
            return true;
          }
          auto command = currentLocalOrder_->computeOrderExecuteCommand();
          if ((command.empty() || command.contains("useMock")) &&
              command_.contains("orderId") &&
              command_.contains("elevatorId") &&
              command_.contains("scheduleType") &&
              command_["scheduleType"] == "PARKING")
          {
            command = command_;
          }
          if (command.contains("orderId") && !computeCommandWithElevatorPlan(command))
          {
            return false;
          }

          std::string dodgeRobotId;
          if (command.contains("orderId") && command.contains("waypointId") && shouldDestinationDodge(command["waypointId"], dodgeRobotId))
          {
            if (robotPtr->robotState() != RobotState::CHARGING)
            {
              publishAbortCommand();
            }
            return true;
          }

          if (command.contains("orderId") && command.contains("elevatorId") && command.contains("waypointId") &&
              shouldElevatorDodge(command["elevatorId"], command["waypointId"]))
          {
            std::string dodgeWaypointId;
            if (findElevatorDodgeWaypoint(command["elevatorId"], dodgeWaypointId))
            {
              StationModel waypoint;
              auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
              if (deviceUtility->getWaypointInfo(dodgeWaypointId, waypoint))
              {
                nlohmann::json dodgeCommand = command;
                if (dodgeCommand.find("id") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("id"));
                }
                if (dodgeCommand.find("orderId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("orderId"));
                }
                if (dodgeCommand.find("elevatorId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("elevatorId"));
                }
                if (dodgeCommand.find("floorId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("floorId"));
                }
                if (dodgeCommand.find("floorName") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("floorName"));
                }
                if (dodgeCommand.find("waypointId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("waypointId"));
                }
                if (dodgeCommand.find("waypointType") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("waypointType"));
                }
                if (dodgeCommand.find("coordinates") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("coordinates"));
                }
                if (dodgeCommand.find("sterilizationMovement") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("sterilizationMovement"));
                }
                dodgeCommand = mergeJson(dodgeCommand, waypoint.toJson());
                dodgeCommand["coopMode"] = "LOCAL";
                dodgeCommand["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
                dodgeCommand["dodgeCommandId"] = boost::uuids::to_string(boost::uuids::random_generator()());
                if (!dodgeCommand.empty() && isCommandChanged(dodgeCommand))
                {
                  command_ = dodgeCommand;
                  dodgeCommand["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
                  missionMsg.data = dodgeCommand.dump();
                  missionPublisher_.publish(missionMsg);
                  SPDLOG_INFO("MissionPlanner Elevator dodge publish command {}", missionMsg.data);
                }
                return true;
              }
            }
            if (robotPtr->robotState() != RobotState::CHARGING)
            {
              publishAbortCommand();
            }
            return true;
          }
          if (!command.empty() && isCommandChanged(command))
          {
            command["coopMode"] = "LOCAL";
            command["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
            command_ = command;
            command["commandState"] = "QUEUEING";
            if (command.contains("orderId") && command.contains("elevatorId") && command.contains("elevatorCost"))
            {
              missionCenter->reportElevatorAllocationResult(command["orderId"], command["elevatorId"], command["elevatorCost"]);
            }
            missionMsg.data = command_.dump();
            currentLocalOrder_->updateOrderProgress(command);
            missionPublisher_.publish(missionMsg);
            plannerState_ = ScheduleState::LOCAL_RUNNING;
            SPDLOG_INFO("MissionPlanner Parking publish command {}", missionMsg.data);
          }
          return true;
        }

        bool publishAbortCommand(const std::string orderId = "", const bool densityCtrl = false)
        {
          nlohmann::json command;
          std_msgs::String missionMsg;
          command["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotInfo = deviceUtility->getRobotInfo();
          if (!orderId.empty())
          {
            command["orderId"] = orderId;
          }
          command["densityCtrl"] = densityCtrl;
          command["commandType"] = "ABORT";
          command["scheduleType"] = "LOCAL_CANCEL";
          command["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
          command["localCreateTime"] = std::chrono::high_resolution_clock::now();
          publishClearGoal(!densityCtrl);
          if (isCommandChanged(command) || RobotState::IDLE != robotInfo->robotState())
          {
            command_ = command;
            missionMsg.data = command_.dump();
            missionPublisher_.publish(missionMsg);
            SPDLOG_INFO("MissionPlanner publish abort command {}", missionMsg.data);
            SPDLOG_INFO("[density]MissionPlanner publish abort command {}", missionMsg.data);
            return true;
          }
          return false;
        }

        bool executeManualOrder()
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          StationModel waypoint;
          std_msgs::String missionMsg;
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotPtr = deviceUtility->getRobotInfo();
          auto missionCenter = cti::missionSchedule::common::getContainer()->resolve<IPlatformMissionCenter>();
          nlohmann::json command = missionCenter->computeManualOrderCommand();
          SPDLOG_INFO("get manual command :{}", command.dump().data());
          if (command.empty() &&
              command_.contains("orderId") &&
              command_.contains("elevatorId") &&
              command_.contains("scheduleType") &&
              command_["scheduleType"] == "CHARGING")
          {
            command = command_;
          }
          if (command.contains("orderId") && !computeCommandWithElevatorPlan(command))
          {
            return false;
          }
          auto elevatorPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<IElevatorPlanner>();
          if ((deviceUtility->isPositionNearNarrowArea(robotPtr->location()) ||
               elevatorPlanner->isRobotLocationOccupyElevator(robotPtr->currentFloor(), robotPtr->location())) &&
              RobotState::IDLE != robotPtr->robotState())
          {
            return true;
          }
          std::string dodgeRobotId;
          if (command.contains("orderId") && command.contains("waypointId") && shouldDestinationDodge(command["waypointId"], dodgeRobotId))
          {
            if (robotPtr->robotState() != RobotState::CHARGING)
            {
              publishAbortCommand();
            }
            return true;
          }
          if (command.contains("orderId") && command.contains("elevatorId") && shouldElevatorDodge(command["elevatorId"], command["waypointId"]))
          {
            std::string dodgeWaypointId;
            if (findElevatorDodgeWaypoint(command["elevatorId"], dodgeWaypointId))
            {
              StationModel waypoint;
              auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
              if (deviceUtility->getWaypointInfo(dodgeWaypointId, waypoint))
              {
                nlohmann::json dodgeCommand = command;
                dodgeCommand["commandType"] = "MOVE";
                dodgeCommand["horizontalOffset"] = 0.0;
                if (dodgeCommand.find("id") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("id"));
                }
                if (dodgeCommand.find("orderId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("orderId"));
                }
                if (dodgeCommand.find("elevatorId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("elevatorId"));
                }
                if (dodgeCommand.find("floorId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("floorId"));
                }
                if (dodgeCommand.find("floorName") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("floorName"));
                }
                if (dodgeCommand.find("waypointId") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("waypointId"));
                }
                if (dodgeCommand.find("waypointType") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("waypointType"));
                }
                if (dodgeCommand.find("coordinates") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("coordinates"));
                }
                if (dodgeCommand.find("sterilizationMovement") != dodgeCommand.end())
                {
                  dodgeCommand.erase(dodgeCommand.find("sterilizationMovement"));
                }
                dodgeCommand = mergeJson(dodgeCommand, waypoint.toJson());
                dodgeCommand["coopMode"] = "LOCAL";
                dodgeCommand["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
                dodgeCommand["dodgeCommandId"] = boost::uuids::to_string(boost::uuids::random_generator()());
                if (!dodgeCommand.empty() && isCommandChanged(dodgeCommand))
                {
                  command_ = dodgeCommand;
                  dodgeCommand["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
                  missionMsg.data = dodgeCommand.dump();
                  missionPublisher_.publish(missionMsg);
                  SPDLOG_INFO("MissionPlanner Elevator dodge publish command {}", missionMsg.data);
                }
                return true;
              }
            }
            if (robotPtr->robotState() != RobotState::CHARGING)
            {
              publishAbortCommand();
            }
            return true;
          }
          if (!command.empty() && isCommandChanged(command))
          {
            command["coopMode"] = "LOCAL";
            command["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
            command_ = command;
            command["commandState"] = "QUEUEING";
            missionMsg.data = command_.dump();
            if (command.contains("orderId") && command.contains("elevatorId") && command.contains("elevatorCost"))
            {
              missionCenter->reportElevatorAllocationResult(command["orderId"], command["elevatorId"], command["elevatorCost"]);
            }
            missionCenter->updatePlatformMissionState(command);
            missionPublisher_.publish(missionMsg);
            plannerState_ = ScheduleState::LOCAL_RUNNING;
            SPDLOG_INFO("MissionPlanner executeManualOrder publish command {}", missionMsg.data);
          }
          return true;
        }

        bool abortAllPlatformMissions() override
        {
          publishAbortCommand();
          return true;
        }

        bool cancelAllPlatformMissions(std::string reason)
        {
          auto missionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>();
          if (missionCenter->cancelAllPlatformOrders(reason))
          {
            return true;
          }
          return false;
        }

        bool cancelManualOrder() override
        {
          auto missionCenter = cti::missionSchedule::common::getContainer()->resolveOrNull<IPlatformMissionCenter>();
          if (missionCenter->cancelManualOrder())
          {
            return publishAbortCommand();
          }
          return false;
        }

        bool isAppropriateToReboot() override
        {
          time_t now;
          struct sysinfo info;
          if (sysinfo(&info))
          {
            return false;
          }
          auto uptimePtr = gmtime(&info.uptime);
          int uptimeMinutes = uptimePtr->tm_yday * 24 * 60 + uptimePtr->tm_hour * 60 + uptimePtr->tm_min;
          SPDLOG_INFO("Mission Planner uptime: day:{} hour:{} min:{}", uptimePtr->tm_yday, uptimePtr->tm_hour, uptimePtr->tm_min);

          time(&now);
          auto localtimePtr = localtime(&now);
          SPDLOG_INFO("Mission Planner localtime: day:{} hour:{} min:{}", localtimePtr->tm_yday, localtimePtr->tm_hour, localtimePtr->tm_min);
          int localtimeMinutes = localtimePtr->tm_hour * 60 + localtimePtr->tm_min;

          SPDLOG_INFO("shouldRebootUptime:{}(min) rebootUpperTime:{}(min) rebootLowerTime_:{}(min) shouldRebootIdleTime:{}(min)",
                      shouldRebootUptimeThreshold_, rebootUpperTime_, rebootLowerTime_, shouldRebootIdleTimeThreshold_);
          SPDLOG_INFO("uptime:{}(min) timeNow:{}(min) idleTime:{}(min)",
                      uptimeMinutes,
                      localtimeMinutes,
                      std::chrono::duration_cast<std::chrono::minutes>(std::chrono::system_clock::now() - lastOrderOrCommandUpdateSec_).count());

          if (uptimeMinutes < shouldRebootUptimeThreshold_)
          {
            return false;
          }
          if (((rebootUpperTime_ < rebootLowerTime_) && (localtimeMinutes < rebootUpperTime_ || localtimeMinutes > rebootLowerTime_)) ||
              ((rebootUpperTime_ >= rebootLowerTime_) && (localtimeMinutes < rebootUpperTime_ && localtimeMinutes > rebootLowerTime_)))
          {
            return false;
          }
          if (std::chrono::duration_cast<std::chrono::minutes>(std::chrono::system_clock::now() - lastOrderOrCommandUpdateSec_).count() < shouldRebootIdleTimeThreshold_)
          {
            return false;
          }
          SPDLOG_INFO("Mission Planner robot should reboot!");
          return true;
        }

        nlohmann::json computeRelocateCommand(const Position& location, std::string floorName, std::string buildingName)
        {
          nlohmann::json relocateJson;
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotInfo = deviceUtility->getRobotInfo();
          if ((robotInfo->localized() && robotInfo->location().valid()) || location.valid())
          {
            relocateJson["commandType"] = "RELOCATE";
            relocateJson["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
            relocateJson["coopMode"] = "LOCAL";
            relocateJson["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
            relocateJson["buildingId"] = deviceUtility->getBuildingId();
            relocateJson["mapFileDirectory"] = robotInfo->buildingName();
            relocateJson["floorName"] = deviceUtility->getCurrentFloor();
            if (location.valid())
            {
              if (!floorName.empty())
              {
                relocateJson["floorName"] = floorName;
              }
              if (!buildingName.empty())
              {
                relocateJson["mapFileDirectory"] = buildingName;
              }
              relocateJson["coordinates"] = location.toJson();
            }
            else if (!robotInfo->currentStation().expired())
            {
              auto station = robotInfo->currentStation().lock();
              relocateJson = mergeJson(relocateJson, station->toJson());
            }
            else if (robotInfo->location().valid())
            {
              relocateJson["coordinates"] = robotInfo->location().toJson();
            }
          }
          return relocateJson;
        }

        bool storeRelocateToLocalStorage(const Position& location = Position(), std::string floorName = std::string(), std::string buildingName = std::string())
        {
          nlohmann::json relocateJson;
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotInfo = deviceUtility->getRobotInfo();
          time_t now;
          time(&now);
          relocateJson["time"] = now;
          if (deviceUtility->isZigbeeModuleFault())
          {
            relocateJson["zigbeeFault"] = true;
          }
          relocateJson = mergeJson(relocateJson, computeRelocateCommand(location, floorName, buildingName));
          if (auto robotInfo = deviceUtility->getRobotInfo())
          {
            if (!robotInfo->hiveAttachedOnRobot().empty())
            {
              relocateJson["hiveId"] = robotInfo->hiveAttachedOnRobot();
            }
          }
          std::string content = relocateJson.dump();
          std::ofstream fs(locationStoragePath_, std::ios::out | std::ios::trunc);
          fs.write(content.c_str(), content.size());
          SPDLOG_INFO("Mission Planner store relocate json :{}", content);
          fs.flush();
          fs.close();
          return true;
        }

        nlohmann::json readRelocateFromLocalStorage() override
        {
          nlohmann::json relocateJson;
          time_t now;
          time(&now);
          try
          {
            std::ifstream fs(locationStoragePath_, std::ios::in);
            fs >> relocateJson;
            SPDLOG_INFO("MissionPlanner read relocate json:{} ,time now:{}", relocateJson.dump(), now);
            if (!relocateJson.contains("time") || !relocateJson["time"].is_number_unsigned() || (now - relocateJson["time"].get<unsigned int>() > 60 * 4))
            {
              SPDLOG_INFO("MissionPlanner read relocate json'time error");
              relocateJson.clear();
            }
            fs.close();
          }
          catch (std::exception& ex)
          {
            SPDLOG_WARN("Mission Planner read local relocate storage file {} exception - {}", locationStoragePath_, ex.what());
          }
          std::ofstream fs(locationStoragePath_, std::ios::out | std::ios::trunc);
          fs.close();
          return relocateJson;
        }

        bool robotAutonomicReboot() override
        {
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto robotState = deviceUtility->getRobotInfo()->robotState();
          if (robotState == RobotState::DOCKING || robotState == RobotState::UNSTOPABLE || robotState == RobotState::LIFTING)
          {
            return true;
          }
          storeRelocateToLocalStorage();
          nlohmann::json command;
          std_msgs::String missionMsg;
          command["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
          command["commandType"] = "RESTART";
          command["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
          command["duration"] = 5 * 60000;
          if (isCommandChanged(command))
          {
            command_ = command;
            missionMsg.data = command_.dump();
            missionPublisher_.publish(missionMsg);
            SPDLOG_INFO("MissionPlanner publish restart command {}", missionMsg.data);
          }
          return true;
        }

        bool publishRelocateCommand(const nlohmann::json& relocateJson) override
        {
          std_msgs::String missionMsg;
          command_ = relocateJson;
          missionMsg.data = command_.dump();
          missionPublisher_.publish(missionMsg);
          SPDLOG_INFO("MissionPlanner publish relocate command {}", missionMsg.data);
          return true;
        }

        bool bindLocalHive(bool up, const std::string& hiveId) override
        {
          nlohmann::json command;
          std_msgs::String missionMsg;
          command["commandType"] = "JACK";
          command["requestedBy"] = "39136da7-56a5-49ff-85b3-967803f4e1ee";
          command["id"] = boost::uuids::to_string(boost::uuids::random_generator()());
          command["up"] = up;
          command["hiveId"] = hiveId;
          command["operationMode"] = "DELIVERY";
          missionMsg.data = command.dump();
          SPDLOG_INFO("MissionPlanner binding local hive :[{}]",command.dump());
          missionPublisher_.publish(missionMsg);
          return true;
        }

        bool publishClearGoal(const bool clearGoal)
        {
          // pub false if have density_info
          nlohmann::json command;
          std_msgs::String msg;
          // if (clearGoal != lastClearGoalFlag_)
          {
            SPDLOG_INFO("[density]publishClearGoal : pub clear goal {}", clearGoal);
            command["clearGoal"] = clearGoal;
            msg.data = command.dump();
            // ros::Duration(0.1).sleep();
            clearGoalPublisher_.publish(msg);
            lastClearGoalFlag_ = clearGoal;
          }
          return true;
        }

        road_control::density_srvResponse callDensityServer(std::shared_ptr<cti::missionSchedule::RobotUtility> robot, Position destination)
        {
          road_control::density_srv densitySrv;
          std::string robotId = robot->robotId();
          auto robotPosition = robot->location();

          geometry_msgs::PoseStamped start;
          geometry_msgs::PoseStamped goal;

          start.header.frame_id = "map";
          start.header.stamp = ros::Time::now();
          start.pose.position.x = robotPosition.slam.position.x;
          start.pose.position.y = robotPosition.slam.position.y;
          start.pose.position.z = robotPosition.slam.position.z;
          start.pose.orientation.w = robotPosition.slam.orientation.w;
          start.pose.orientation.x = robotPosition.slam.orientation.x;
          start.pose.orientation.y = robotPosition.slam.orientation.y;
          start.pose.orientation.z = robotPosition.slam.orientation.z;

          goal.header.frame_id = "map";
          goal.header.stamp = ros::Time::now();
          goal.pose.position.x = destination.slam.position.x;
          goal.pose.position.y = destination.slam.position.y;
          goal.pose.position.z = destination.slam.position.z;
          goal.pose.orientation.w = destination.slam.orientation.w;
          goal.pose.orientation.x = destination.slam.orientation.x;
          goal.pose.orientation.y = destination.slam.orientation.y;
          goal.pose.orientation.z = destination.slam.orientation.z;

          densitySrv.request.robot_id = robotId;
          densitySrv.request.start = start;
          densitySrv.request.goal = goal;
          densitySrv.request.partial = partialDensity_;
          densitySrv.request.length = densityRange_;

          SPDLOG_INFO("callDensityServer[density]: {} Call service {}. partial: {}, length: {}, start({},{}), goal({},{})", robotId, "/road_control/density_info", 
            partialDensity_, densityRange_, start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
          ROS_INFO("callDensityServer[density] : %s Call service /road_control/density_info", densitySrv.request.robot_id.c_str());

          if (densityClient_.call(densitySrv))
          {
            ROS_INFO("callDensityServer[density]: %s[%d], %s", robotId.c_str(), densitySrv.response.moveable, densitySrv.response.data.c_str());
            SPDLOG_INFO("callDensityServer[density]: {}[{}], {}", robotId, densitySrv.response.moveable, densitySrv.response.data);
            return densitySrv.response;
          }
          else
          {
            SPDLOG_INFO("callDensityServer[density]: {} Failed to call service {}", robotId, "/road_control/density_info");
            road_control::density_srvResponse res;
            res.data = "";
            res.moveable = true;
            return res;
          }
        }
    };

    std::shared_ptr<IMissionPlanner> createMissionPlanner()
    {
      auto missionPlanner = std::make_shared<MissionPlanner>();
      return std::dynamic_pointer_cast<IMissionPlanner>(missionPlanner);
    }
  }
}
