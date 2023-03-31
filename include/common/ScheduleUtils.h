#ifndef __MISSION_SCHEDULE_SCHEDULEUTILS_HH__
#define __MISSION_SCHEDULE_SCHEDULEUTILS_HH__

#include <memory>
#include <chrono>
#include "cti_spdlog.h"
#include <unordered_map>
#include <unordered_set>
#include "geometry_msgs/Pose.h"
#include "nlohmann/json.hpp"
#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>

namespace nlohmann {
    template<typename Clock, typename Duration>
    struct adl_serializer<std::chrono::time_point<Clock, Duration>>
    {
        static void to_json(json& j, const std::chrono::time_point<Clock, Duration>& tp)
        {
            j["since_epoch"] = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch()).count();
            j["unit"] = "microseconds";
        }
    };
}

namespace cti
{
  namespace missionSchedule
  {
    typedef enum
    {
      GENERAL = 0,
      DELIVER,
      STERILIZATION,
      NULLOPERATIONMODE,
    } OperationMode;

    static std::map<std::pair<OperationMode, OperationMode>, bool> \
      operationModeMatchTable = { {{OperationMode::GENERAL, OperationMode::GENERAL}, true},
                                  {{OperationMode::GENERAL, OperationMode::DELIVER}, true},
                                  {{OperationMode::GENERAL, OperationMode::STERILIZATION}, true},
                                  {{OperationMode::DELIVER, OperationMode::GENERAL}, true},
                                  {{OperationMode::DELIVER, OperationMode::DELIVER}, true},
                                  {{OperationMode::DELIVER, OperationMode::STERILIZATION}, false},
                                  {{OperationMode::STERILIZATION, OperationMode::GENERAL}, true},
                                  {{OperationMode::STERILIZATION, OperationMode::DELIVER}, false},
                                  {{OperationMode::STERILIZATION, OperationMode::STERILIZATION}, true} };

    static OperationMode toOperationModeEnum(const std::string& operationModeStr)
    {
      if (operationModeStr.empty())
      {
        return OperationMode::NULLOPERATIONMODE;
      }
      if (operationModeStr == "GENERIC")
      {
        return OperationMode::GENERAL;
      }
      else if (operationModeStr == "DELIVERY")
      {
        return OperationMode::DELIVER;
      }
      else if (operationModeStr == "STERILIZATION")
      {
        return OperationMode::STERILIZATION;
      }
      return OperationMode::NULLOPERATIONMODE;
    }

    class GridLocation
    {
      public:
        int x, y;
        double px, py;

        inline bool operator==(const GridLocation& other) const
        {
          return x == other.x && y == other.y;
        }

        inline bool operator!=(const GridLocation& other) const
        {
          return x != other.x || y != other.y;
        }

        inline bool operator<(const GridLocation& other) const
        {
          return x < other.x && y < other.y;
        } inline bool operator>(const GridLocation& other) const
        {
          return x > other.x || y > other.y;
        }

        inline double computeDirection(const GridLocation& other)
        {
          double lineX = 1.0;
          double lineY = 0.0;
          double diffX = other.px - this->px;
          double diffY = other.py - this->py;
          double dot = lineX * diffX + lineY * diffY;
          double det = lineX * diffY - lineY * diffX;
          return std::atan2(det, dot);
        }

        inline double computeDistance(const GridLocation& other)
        {
          return std::hypot((other.px - this->px), (other.py - this->py));
        }

        GridLocation(int x_, int y_) : x(x_), y(y_) {};
        GridLocation(int x_, int y_, double px_, double py_) : x(x_), y(y_), px(px_), py(py_) {};
        GridLocation() : x(0), y(0) {};
    };
  }
}

namespace std
{
  // implement hash function so we can put GridLocation into an unordered_set
  template <> struct hash<cti::missionSchedule::GridLocation> {
      typedef cti::missionSchedule::GridLocation argument_type;
      typedef std::size_t result_type;
      std::size_t operator()(const cti::missionSchedule::GridLocation& id) const noexcept {
        return std::hash<int>()(id.x ^ (id.y << 4));
      }
  };
}

namespace cti
{
  namespace missionSchedule
  {
    class RobotUtility;

    typedef enum
    {
      IDLE = 0,
      UNSTOPABLE,
      CHARGING,
      WAITINGLIFT,
      LIFTING,
      DOCKING,
      BUSY,
      FAULT,
    } RobotState;

    typedef enum
    {
      READY = 0,
      LOCAL_RUNNING,
      PLATFORM_RUNNING,
    } ScheduleState;

    typedef enum
    {
      COMMAND_QUEUEING = 0,
      COMMAND_STARTUP,
      COMMAND_COMPLETED,
      COMMAND_CANCELLED,
      COMMAND_FAILED,
      COMMAND_DIRECT_FAILED,
    } CommandState;

    typedef enum
    {
      DISABLED = 0,
      ENABLED,
    } ChassisState;

    struct Position
    {
      inline static double getAngle(double ox, double oy, double oz, double ow)
      {
        if (oz == 0 && ow == 0)
        {
          return 0;
        }
        return std::atan2(2 * (ow * oz + ox * oy), 1 - 2 * (oy * oy + oz * oz));
      }

      double getAngle() const
      {
        if (valid())
        {
            return getAngle(slam.orientation.x, slam.orientation.y, slam.orientation.z, slam.orientation.w);
        }
        return 0;
      }

      double distanceOf() const
      {
        return std::hypot(slam.position.x - 0.0, slam.position.y - 0.0);
      }

      double distanceOf(const Position& other) const
      {
        return std::hypot(slam.position.x - other.slam.position.x, slam.position.y - other.slam.position.y);
      }

      double angleOf(const Position& other) const
      {
        return this->getAngle() - other.getAngle();
      }

      bool operator == (const Position& other) const
      {
        return memcmp(this, &other, sizeof(Position)) == 0;
      }

      bool operator != (const Position& other) const
      {
        return !(*this == other);
      }

      inline bool valid() const
      {
        return slam.orientation.w != 0;
      }

      inline void reset()
      {
        slam.position.x = slam.position.y = slam.position.z = 0;
        slam.orientation.x = slam.orientation.y = slam.orientation.z = slam.orientation.w = 0;
      }

      bool parseFromJson(nlohmann::json value)
      {
        if (value.is_array() && std::all_of(value.begin(), value.end(), [](auto& v){ return v.is_number(); }))
        {
          if (value.size() == 8)
          {
            slam.position.x = value[0].get<double>();
            slam.position.y = value[1].get<double>();
            slam.position.z = value[2].get<double>();

            slam.orientation.x = value[4].get<double>();
            slam.orientation.y = value[5].get<double>();
            slam.orientation.z = value[6].get<double>();
            slam.orientation.w = value[7].get<double>();

            if (slam.orientation.z == 0 && slam.orientation.w == 0)
            {
              auto yaw = value[3].get<double>();
              slam.orientation.x = 0;
              slam.orientation.y = 0;
              slam.orientation.z = std::sin(yaw / 2);
              slam.orientation.w = std::cos(yaw / 2);
            }
            return true;
          }
        }
        return false;
      }

      nlohmann::json toJson() const
      {
        nlohmann::json jsonObjects = nlohmann::json::array();
        if (valid())
        {
          jsonObjects.push_back(slam.position.x);
          jsonObjects.push_back(slam.position.y);
          jsonObjects.push_back(slam.position.z);
          jsonObjects.push_back(getAngle());
          jsonObjects.push_back(slam.orientation.x);
          jsonObjects.push_back(slam.orientation.y);
          jsonObjects.push_back(slam.orientation.z);
          jsonObjects.push_back(slam.orientation.w);
        }
        return jsonObjects;
      }

      std::string toString() const
      {
        std::stringstream ss;
        ss << "position:[" << slam.position.x << ", ";
        ss << slam.position.y << ", " << slam.position.z << "], ";
        ss << "yaw:" << getAngle() << ", ";
        ss << "orientation:[" << slam.orientation.x << ", ";
        ss << slam.orientation.y << ", " << slam.orientation.z << ", " << slam.orientation.w << "]";
        return std::move(std::string(ss.str()));
      }

      geometry_msgs::Pose toPose() const
      {
        geometry_msgs::Pose pose;
        pose.position.x = slam.position.x;
        pose.position.y = slam.position.y;
        pose.position.z = slam.position.z;
        pose.orientation.x = slam.orientation.x;
        pose.orientation.y = slam.orientation.y;
        pose.orientation.z = slam.orientation.z;
        pose.orientation.w = slam.orientation.w;
        return pose;
      }

      void transferFromPose(const geometry_msgs::Pose& pose)
      {
        this->slam.position.x = pose.position.x;
        this->slam.position.y = pose.position.y;
        this->slam.orientation.w = pose.orientation.w;
        this->slam.orientation.x = pose.orientation.x;
        this->slam.orientation.y = pose.orientation.y;
        this->slam.orientation.z = pose.orientation.z;
      }

      Position& shift(double front, double left, double yaw = 0)
      {
        double arc =  this->getAngle();
        this->slam.position.x += front * ::cos(arc) - left * ::sin(arc);
        this->slam.position.y += front * ::sin(arc) + left * ::cos(arc);
        yaw += arc;

        this->slam.orientation.w = std::cos(yaw / 2);
        this->slam.orientation.x = 0;
        this->slam.orientation.y = 0;
        this->slam.orientation.z = std::sin(yaw / 2);
        return *this;
      }

      struct
      {
        struct
        {
          double x {0};
          double y {0};
          double z {0};
        } position;
        struct
        {
          double x {0};
          double y {0};
          double z {0};
          double w {0};
        } orientation;
      } slam;
    };

    //点是否在区域内
    static bool isInside(const std::vector<Position>& polygon, const Position& p)
    {
      int i, j, c = 0;
      int n = polygon.size();
      if (n < 3)
        return false;

      for (i = 0, j = n - 1; i < n; j = i++)
      {
        if (((polygon[i].slam.position.y > p.slam.position.y) != (polygon[j].slam.position.y > p.slam.position.y)) &&
          (p.slam.position.x < (polygon[j].slam.position.x - polygon[i].slam.position.x) * (p.slam.position.y - polygon[i].slam.position.y) / (polygon[j].slam.position.y - polygon[i].slam.position.y) + polygon[i].slam.position.x))
        {
          c = !c;
        }
      }

      if (c == 1)
      {
        return true;
      }
      return false;
    }

    struct StationDistributionInfo
    {
      Position lineAnchor;
      std::pair<Position, Position> line;
    };

    class CommandVertex
    {
      private:
        // bool withBox_{false};
        bool switchFloor_{false};
        bool dropInPlace_{false};
        bool ignoreHiveChargingStatus_{false};
        bool chargingContinueFlag_{false};
        double horizontalOffset_{0.0};
        nlohmann::json data_;
        nlohmann::json contexts_;
        Position coordinate_{};
        CommandState commandState_{CommandState::COMMAND_QUEUEING};
        std::string id_, qr_, hiveQr_, orderId_, hiveOrderSetId_, commandType_, scheduleType_, stationId_, orderState_, reason_;
      public:
        CommandVertex() = default;

        inline const auto& id() const { return id_; }

        inline const auto& qr() const { return qr_; }

        inline const auto& data() const { return data_; }

        inline const auto& hiveQr() const { return hiveQr_; }

        // inline const auto& withBox() const { return withBox_; }

        inline const auto& dropInPlace() const { return dropInPlace_; }

        inline const auto& ignoreHiveChargingStatus() const { return ignoreHiveChargingStatus_; }

        inline const auto& chargingContinueFlag() const { return chargingContinueFlag_; }

        inline const auto& orderId() const { return orderId_; }

        inline const auto& reason() const { return reason_; }

        inline const auto& orderState() const { return orderState_; }

        inline const auto& switchFloor() const { return switchFloor_; }

        inline const auto& stationId() const { return stationId_; }

        inline const auto& commandType() const { return commandType_; }

        inline const auto& commandState() const {return commandState_; }

        inline const auto& scheduleType() const { return scheduleType_; }

        inline const auto& hiveOrderSetId() const { return hiveOrderSetId_; }

        inline const auto& horizontalOffset() const { return horizontalOffset_; }

        inline void setCommandState(const CommandState& state) { commandState_ = state; }

        // inline auto& id() { return id_; }

        // inline auto& orderId() { return orderId_; }

        // inline auto& commandType() { return commandType_; }

        // inline auto& scheduleType() { return scheduleType_; }

        // inline auto& stationId() { return stationId_; }

        inline nlohmann::json toJson() const
        {
          nlohmann::json jsonObjects;
          jsonObjects["id"] = id_;
          // jsonObjects["withBox"] = withBox_;
          jsonObjects["waypointId"] = stationId_;
          jsonObjects["orderState"] = orderState_;
          jsonObjects["switchFloor"] = switchFloor_;
          jsonObjects["commandType"] = commandType_;
          jsonObjects["scheduleType"] = scheduleType_;
          jsonObjects["coordinates"] = coordinate_.toJson();
          if (dropInPlace_)
          {
            jsonObjects["dropInPlace"] = dropInPlace_;
          }
          if (ignoreHiveChargingStatus_)
          {
            jsonObjects["ignoreHiveChargingStatus"] = ignoreHiveChargingStatus_;
          }
          if (chargingContinueFlag_)
          {
            jsonObjects["chargingContinueFlag"] = chargingContinueFlag_;
          }
          if (!contexts_.empty())
          {
            jsonObjects["contexts"] = contexts_;
          }
          if (horizontalOffset_ != 0.0)
          {
            jsonObjects["horizontalOffset"] = horizontalOffset_;
          }
          if (!qr_.empty())
          {
            jsonObjects["qr"] = qr_;
          }
          if (!hiveQr_.empty())
          {
            jsonObjects["hiveQr"] = hiveQr_;
          }
          if (!orderId_.empty())
          {
            jsonObjects["orderId"] = orderId_;
          }
          if (!hiveOrderSetId_.empty())
          {
            jsonObjects["hiveOrderSetId"] = hiveOrderSetId_;
          }
          if (commandState_ == CommandState::COMMAND_COMPLETED)
          {
            jsonObjects["commandState"] = "COMPLETED";
          }
          else if (commandState_ == CommandState::COMMAND_CANCELLED)
          {
            jsonObjects["commandState"] = "CANCELLED";
          }
          else if (commandState_ == CommandState::COMMAND_DIRECT_FAILED)
          {
            jsonObjects["commandState"] = "DIRECT_FAILED";
          }
          else if (commandState_ == CommandState::COMMAND_QUEUEING)
          {
            jsonObjects["commandState"] = "QUEUEING";
          }
          else if (commandState_ == CommandState::COMMAND_FAILED)
          {
            jsonObjects["commandState"] = "FAILED";
          }
          else if (commandState_ == CommandState::COMMAND_STARTUP)
          {
            jsonObjects["commandState"] = "STARTUP";
          }
          return jsonObjects;
        }
        static std::shared_ptr<CommandVertex> parseFromJson(const nlohmann::json& json);
    };

    class StationModel
    {
      private:
        bool chargeable_{false}, baned_{false};
        Position coordinate_{};
        std::vector<std::string> tags_;
        std::vector<std::string> occupiedRobotId_;
        std::string id_, name_, floor_, floorId_, type_, qr_, dockId_, hiveOccupyThisStation_, operationMode_;
        std::vector<Position> stationArea_;
        std::weak_ptr<RobotUtility> robotOccupyThisStation_;
      public:
        StationModel() = default;

        inline auto& id() const { return id_; }

        inline auto& baned() const { return baned_; }

        inline auto& qr() const { return qr_; }

        inline auto& tags() const { return tags_; }

        inline auto& name() const { return name_;}

        inline auto& type() const { return type_; }

        inline auto& floor() const { return floor_; }

        inline auto& dockId() const { return dockId_; }

        inline auto& floorId() const { return floorId_; }

        inline auto& chargeable() const { return chargeable_; }

        inline auto& coordinate() const { return coordinate_; }

        inline auto& stationArea() const { return stationArea_; }

        inline auto& operationMode() const { return operationMode_; }

        inline auto& occupiedRobotId() const { return occupiedRobotId_; }

        inline auto& hiveOccupyThisStation() const { return hiveOccupyThisStation_; }

        inline auto& robotOccupyThisStation() const {return robotOccupyThisStation_;}

        inline auto& id() { return id_; }

        inline auto& baned() { return baned_; }

        inline auto& qr() { return qr_; }

        inline auto& tags() { return tags_; }

        inline auto& name() { return name_;}

        inline auto& type() { return type_; }

        inline auto& floor() { return floor_; }

        inline auto& dockId() { return dockId_; }

        inline auto& floorId() { return floorId_; }

        inline auto& chargeable() { return chargeable_; }

        inline auto& coordinate() { return coordinate_; }

        inline auto& stationArea() { return stationArea_; }

        inline auto& occupiedRobotId() { return occupiedRobotId_; }

        inline auto& hiveOccupyThisStation() { return hiveOccupyThisStation_; }

        inline auto& robotOccupyThisStation() { return robotOccupyThisStation_; }

        inline bool isInsideOccupyArea(const Position& location) const
        {
          if (!location.valid())
          {
            return false;
          }
          if (stationArea_.empty())
          {
            return coordinate_.distanceOf(location) < 0.7f;
          }
          return isInside(stationArea_, location);
        }

        inline nlohmann::json toJson() const
        {
          nlohmann::json jsonObjects;
          if (!qr_.empty())
          {
            jsonObjects["qr"] = qr_;
          }
          jsonObjects["name"] = name_;
          if (!dockId_.empty())
          {
            jsonObjects["dockId"] = dockId_;
          }
          if (!hiveOccupyThisStation_.empty())
          {
            jsonObjects["hiveOccupyThisStation"] = hiveOccupyThisStation_;
          }
          jsonObjects["waypointId"] = id_;
          jsonObjects["floorName"] = floor_;
          jsonObjects["floorId"] = floorId_;
          jsonObjects["waypointType"] = type_;
          jsonObjects["coordinates"] = coordinate_.toJson();
          return jsonObjects;
        }
    };

    struct HiveUtility
    {
      int hivePower;
      std::string id, qr, robotId, dockId, waypointId;
    };

    class RobotUtility
    {
      private:
        bool disabled_{false};
        bool localized_{false};
        bool hiveAttached_{false};
        int robotSignal_;
        Position location_;
        double chassisPower_{0.0};
        std::string robotId_;
        Position localDestination_;
        Position desiredDestination_;
        Position platformDestination_;
        std::string parkName_;
        RobotState robotState_;
        std::string targetFloor_;
        std::string buildingName_;
        std::string currentFloor_;
        std::string operationMode_;
        std::vector<std::string> tags_;
        ChassisState chassisState_{ChassisState::ENABLED};
        std::string robotStateStr_;
        std::string chassisStateStr_{"ENABLED"};
        std::string currentElevatorId_;
        std::string currentElevatorDeviceId_;
        std::string hiveAttachedOnRobot_;
        std::weak_ptr<StationModel> currentStation_;
        std::chrono::system_clock::time_point lastLocalUpdateSec_;
        std::chrono::system_clock::time_point lastPlatformUpdateSec_;
        bool disabledSet_{false}, localizedSet_{false}, robotSignalSet_{false}, locationSet_{false}, chassisPowerSet_{false}, robotIdSet_{false}, localDestinationSet_{false}, platformDestinationSet_{false},\
          parkNameSet_{false}, robotStateSet_{false}, targetFloorSet_{false}, buildingNameSet_{false}, currentFloorSet_{false}, hiveAttachedOnRobotSet_{false}, lastPlatformUpdateSecSet_{false},\
          chassisStateSet_{false}, robotStateStrSet_{false}, desiredDestinationSet_{false}, chassisStateStrSet_{false}, currentElevatorIdSet_{false}, currentElevatorDeviceIdSet_{false}, lastLocalUpdateSecSet_{false}, hiveAttachedSet_{false}, operationModeSet_ {false}, tagsSet_{false};
      public:
        RobotUtility() = default;

        inline bool tagsSet()                        const   { return tagsSet_; }
        inline bool robotIdSet()                     const   { return robotIdSet_; }
        inline bool disabledSet()                    const   { return disabledSet_; }
        inline bool localizedSet()                   const   { return localizedSet_; }
        inline bool robotSignalSet()                 const   { return robotSignalSet_; }
        inline bool locationSet()                    const   { return locationSet_;  }
        inline bool chassisPowerSet()                const   { return chassisPowerSet_; }
        inline bool operationModeSet()               const   { return operationModeSet_; }
        inline bool localDestinationSet()            const   { return localDestinationSet_; }
        inline bool desiredDestinationSet()          const   { return desiredDestinationSet_; }
        inline bool platformDestinationSet()         const   { return platformDestinationSet_; }
        inline bool parkNameSet()                    const   { return parkNameSet_; }
        inline bool robotStateSet()                  const   { return robotStateSet_; }
        inline bool targetFloorSet()                 const   { return targetFloorSet_; }
        inline bool buildingNameSet()                const   { return buildingNameSet_; }
        inline bool currentFloorSet()                const   { return currentFloorSet_; }
        inline bool chassisStateSet()                const   { return chassisStateSet_; }
        inline bool robotStateStrSet()               const   { return robotStateStrSet_; }
        inline bool chassisStateStrSet()             const   { return chassisStateStrSet_; }
        inline bool currentElevatorIdSet()           const   { return currentElevatorIdSet_; }
        inline bool currentElevatorDeviceIdSet()     const   { return currentElevatorDeviceIdSet_; }
        inline bool hiveAttachedSet()                const   { return hiveAttachedSet_; }
        inline bool hiveAttachedOnRobotSet()         const   { return hiveAttachedOnRobotSet_; }
        inline bool lastLocalUpdateSecSet()          const   { return lastLocalUpdateSecSet_; }
        inline bool lastPlatformUpdateSecSet()       const   { return lastPlatformUpdateSecSet_; }

        inline const auto& tags()                    const   { return tags_; }
        inline const auto& disabled()                const   { return disabled_; }
        inline const auto& localized()               const   { return localized_; }
        inline const auto& hiveAttached()            const   { return hiveAttached_; }
        inline const auto& robotSignal()             const   { return robotSignal_; }
        inline const auto& location()                const   { return location_; }
        inline const auto& chassisPower()            const   { return chassisPower_; }
        inline const auto& robotId()                 const   { return robotId_; }
        inline const auto& operationMode()           const   { return operationMode_; }
        inline const auto& localDestination()        const   { return localDestination_; }
        inline const auto& desiredDestination()      const   { return desiredDestination_; }
        inline const auto& platformDestination()     const   { return platformDestination_; }
        inline const auto& parkName()                const   { return parkName_; }
        inline const auto& robotState()              const   { return robotState_; }
        inline const auto& targetFloor()             const   { return targetFloor_; }
        inline const auto& buildingName()            const   { return buildingName_; }
        inline const auto& currentFloor()            const   { return currentFloor_; }
        inline const auto& chassisState()            const   { return chassisState_; }
        inline const auto& robotStateStr()           const   { return robotStateStr_; }
        inline const auto& chassisStateStr()         const   { return chassisStateStr_; }
        inline const auto& currentElevatorId()       const   { return currentElevatorId_; }
        inline const auto& currentElevatorDeviceId() const   { return currentElevatorDeviceId_; }
        inline const auto& currentStation()          const   { return currentStation_; }
        inline const auto& lastLocalUpdateSec()      const   { return lastLocalUpdateSec_; }
        inline const auto& lastPlatformUpdateSec()   const   { return lastPlatformUpdateSec_; }
        inline const auto& hiveAttachedOnRobot()     const   { return hiveAttachedOnRobot_; }

        inline auto& currentStation() { return currentStation_; }

        inline RobotUtility& setDisabled(bool disabled)
        {
          disabled_ = disabled;
          disabledSet_ = true;
          return *this;
        }
        inline RobotUtility& setLocalized(bool localized)
        {
          localized_ = localized;
          localizedSet_ = true;
          return *this;
        }
        inline RobotUtility& setRobotSignal(int sig)
        {
          robotSignal_ = sig;
          robotSignalSet_ = true;
          return *this;
        }
        inline RobotUtility& setLocation(const Position& pos)
        {
          location_ = pos;
          locationSet_ = true;
          return *this;
        }
        inline RobotUtility& setChassisPower(double power)
        {
          chassisPower_ = power;
          chassisPowerSet_ = true;
          return *this;
        }
        inline RobotUtility& setRobotId(const std::string& robotId)
        {
          robotId_ = robotId;
          robotIdSet_ = true;
          return *this;
        }
        inline RobotUtility& setPlatformDestination(const Position& pos)
        {
          platformDestination_ = pos;
          platformDestinationSet_ = true;
          return *this;
        }
        inline RobotUtility& setDesiredDestination(const Position& pos)
        {
          desiredDestination_ = pos;
          desiredDestinationSet_ = true;
          return *this;
        }
        inline RobotUtility& setLocalDestination(const Position& pos)
        {
          localDestination_ = pos;
          localDestinationSet_ = true;
          return *this;
        }
        inline RobotUtility& setTags(const std::vector<std::string>& tags)
        {
          tags_ = tags;
          tagsSet_ = true;
          return *this;
        }
        inline RobotUtility& setOperationMode(const std::string& operationMode)
        {
          operationMode_ = operationMode;
          operationModeSet_ = true;
          return *this;
        }
        inline RobotUtility& setParkName(const std::string& parkName)
        {
          parkName_ = parkName;
          parkNameSet_ = true;
          return *this;
        }
        inline RobotUtility& setRobotState(const RobotState& state)
        {
          robotState_ = state;
          robotStateSet_ = true;
          return *this;
        }
        inline RobotUtility& setTargetFloor(const std::string floor)
        {
          targetFloor_ = floor;
          targetFloorSet_ = true;
          return *this;
        }
        inline RobotUtility& setBuildingName(const std::string buildingName)
        {
          buildingName_ = buildingName;
          buildingNameSet_ = true;
          return *this;
        }
        inline RobotUtility& setCurrentFloor(const std::string& floor)
        {
          currentFloor_ = floor;
          currentFloorSet_ = true;
          return *this;
        }
        inline RobotUtility& setChassisState(const ChassisState state)
        {
          chassisState_ = state;
          chassisStateSet_ = true;
          return *this;
        }
        inline RobotUtility& setRobotStateStr(const std::string& str)
        {
          robotStateStr_ = str;
          robotStateStrSet_ = true;
          return *this;
        }
        inline RobotUtility& setChassisStateStr(const std::string& str)
        {
          chassisStateStr_ = str;
          chassisStateStrSet_ = true;
          return *this;
        }
        inline RobotUtility& setCurrentElevatorDeviceId(const std::string& str)
        {
          currentElevatorDeviceId_ = str;
          currentElevatorDeviceIdSet_ = true;
          return *this;
        }
        inline RobotUtility& setCurrentElevatorId(const std::string& str)
        {
          currentElevatorId_ = str;
          currentElevatorIdSet_ = true;
          return *this;
        }
        inline RobotUtility& setHiveAttached(bool attachStatus)
        {
          hiveAttached_ = attachStatus;
          hiveAttachedSet_ = true;
          return *this;
        }
        inline RobotUtility& setHiveAttachedOnRobot(const std::string& str)
        {
          hiveAttachedOnRobot_ = str;
          hiveAttachedOnRobotSet_ = true;
          return *this;
        }
        inline RobotUtility& setLastLocalUpdateSec(const std::chrono::system_clock::time_point& time)
        {
          lastLocalUpdateSecSet_ = true;
          lastLocalUpdateSec_ = time;
          return *this;
        }
        inline RobotUtility& setLastPlatformUpdateSec(const std::chrono::system_clock::time_point& time)
        {
          lastPlatformUpdateSecSet_ = true;
          lastPlatformUpdateSec_ = time;
          return *this;
        }

        std::string toString() const
        {
          return toJson().dump();
        }

        nlohmann::json toJson() const
        {
          nlohmann::json jsonObjects;
          jsonObjects["id"] = robotId_;
          jsonObjects["disabled"] = disabled_;
          jsonObjects["state"] = robotStateStr_;
          jsonObjects["power"] = chassisPower_;
          jsonObjects["floor"] = currentFloor_;
          jsonObjects["targetFloor"] = targetFloor_;
          jsonObjects["chassisState"] = chassisStateStr_;
          jsonObjects["location"] = location_.toJson();
          jsonObjects["localized"] = localized_;
          jsonObjects["localDestination"] = localDestination_.toJson();
          if (!hiveAttachedOnRobot_.empty())
          {
            jsonObjects["hiveAttachedOnRobot"] = hiveAttachedOnRobot_;
          }
          if (!tags_.empty())
          {
            jsonObjects["tags"] = tags_;
          }
          if (!operationMode_.empty())
          {
            jsonObjects["operationMode"] = operationMode_;
          }
          if (!currentElevatorId_.empty())
          {
            jsonObjects["elevatorId"] = currentElevatorId_;
          }
          if (!currentElevatorDeviceId_.empty())
          {
            jsonObjects["elevatorDeviceId"] = currentElevatorDeviceId_;
          }
          jsonObjects["hiveAttached"] = hiveAttached_;
          jsonObjects["platformDestination"] = platformDestination_.toJson();
          jsonObjects["desiredDestination"] = desiredDestination_.toJson();
          if (!currentStation_.expired())
          {
            auto station = currentStation_.lock();
            jsonObjects["currentStation"] = station->toJson();
          }
          jsonObjects["lastLocalUpdateSec"] = lastLocalUpdateSec_;
          jsonObjects["lastPlatformUpdateSec"] = lastPlatformUpdateSec_;
          return jsonObjects;
        }
    };

    struct ElevatorPlan
    {
      struct ElevatorPlanStep
      {
        std::string elevatorFloor;
        std::string elevatorId;
        Position elevatorLocation;
        Position elevatorEnterLocation;
      };

      double cost{0.0};
      std::vector<ElevatorPlanStep> planSteps;
      std::vector<ElevatorPlanStep> switchElevatorPath;
    };

    struct OrderElement
    {
      int orderIndex;
      double orderCost;
      std::string hiveId;
      bool assignedToRobot;
      std::chrono::system_clock::time_point orderCreateTime;

      bool operator<(const OrderElement& other) const
      {
        return orderCost > other.orderCost;
      }
    };

    struct BanedStation
    {
      std::string stationId;
      std::chrono::system_clock::time_point banningStartTime;
      std::chrono::system_clock::time_point banningExpireTime;
    };

    static std::vector<Position> computePolygonArea(const Position& center, double length, double width)
    {
      std::vector<Position> returnValue;
      Position tempFront, tempEnd, tempPoint;
      double startAngle = center.getAngle();
      double tempAngle = startAngle;
      tempFront.slam.position.x = center.slam.position.x + length * std::cos(startAngle);
      tempFront.slam.position.y = center.slam.position.y + length * std::sin(startAngle);
      tempEnd.slam.position.x = center.slam.position.x - length * std::cos(startAngle);
      tempEnd.slam.position.y = center.slam.position.y - length * std::sin(startAngle);

      tempAngle = startAngle + 1.57f;
      tempPoint.slam.position.x = tempFront.slam.position.x + width * std::cos(tempAngle);
      tempPoint.slam.position.y = tempFront.slam.position.y + width * std::sin(tempAngle);
      returnValue.push_back(tempPoint);
      tempAngle = startAngle - 1.57f;
      tempPoint.slam.position.x = tempFront.slam.position.x + width * std::cos(tempAngle);
      tempPoint.slam.position.y = tempFront.slam.position.y + width * std::sin(tempAngle);
      returnValue.push_back(tempPoint);

      tempAngle = startAngle - 1.57f;
      tempPoint.slam.position.x = tempEnd.slam.position.x + width * std::cos(tempAngle);
      tempPoint.slam.position.y = tempEnd.slam.position.y + width * std::sin(tempAngle);
      returnValue.push_back(tempPoint);
      tempAngle = startAngle + 1.57f;
      tempPoint.slam.position.x = tempEnd.slam.position.x + width * std::cos(tempAngle);
      tempPoint.slam.position.y = tempEnd.slam.position.y + width * std::sin(tempAngle);
      returnValue.push_back(tempPoint);
      return returnValue;
    }

    static bool isCommandStateOnFinished(const CommandState& state)
    {
      if (state == CommandState::COMMAND_STARTUP || state == CommandState::COMMAND_QUEUEING)
      {
        return false;
      }
      return true;
    }

    static double calculatePointProjectionValueToLine(const Position& lineStart,
                                                      const Position& lineEnd,
                                                      const Position& point)
    {
      Position projectionPoint;
      double lineDistance = lineStart.distanceOf(lineEnd);
      lineDistance = lineDistance * lineDistance;
      if (lineDistance == 0.0f)
      {
        return lineStart.distanceOf(point);
      }
      double dotValue = (point.slam.position.x - lineStart.slam.position.x) *
                        (lineEnd.slam.position.x - lineStart.slam.position.x) +
                        (point.slam.position.y - lineStart.slam.position.y) *
                        (lineEnd.slam.position.y - lineStart.slam.position.y);
      dotValue = dotValue / lineDistance;
      return dotValue;
    }

    static Position calculatePointProjectionPositionToLine(const Position& lineStart,
                                                           const Position& lineEnd,
                                                           const Position& point)
    {
      Position projectionPoint;
      double lineDistance = lineStart.distanceOf(lineEnd);
      lineDistance = lineDistance * lineDistance;
      if (lineDistance == 0.0f)
      {
        return lineStart;
      }
      double dotValue = (point.slam.position.x - lineStart.slam.position.x) *
                        (lineEnd.slam.position.x - lineStart.slam.position.x) +
                        (point.slam.position.y - lineStart.slam.position.y) *
                        (lineEnd.slam.position.y - lineStart.slam.position.y);

      double t = dotValue / lineDistance;
      projectionPoint.slam.position.x = lineStart.slam.position.x +
                      t * (lineEnd.slam.position.x - lineStart.slam.position.x);
      projectionPoint.slam.position.y = lineStart.slam.position.y +
                      t * (lineEnd.slam.position.y - lineStart.slam.position.y);
      return projectionPoint;
    }

    static Position calculatePointProjectionPositionToLineSegment(const Position& lineStart,
                                                                  const Position& lineEnd,
                                                                  const Position& point)
    {
      Position projectionPoint;
      double lineDistance = lineStart.distanceOf(lineEnd);
      lineDistance = lineDistance * lineDistance;
      if (lineDistance == 0.0f)
      {
        return lineStart;
      }
      double dotValue = (point.slam.position.x - lineStart.slam.position.x) *
                        (lineEnd.slam.position.x - lineStart.slam.position.x) +
                        (point.slam.position.y - lineStart.slam.position.y) *
                        (lineEnd.slam.position.y - lineStart.slam.position.y);

      double t = std::max(0.0, std::min(1.0, dotValue / lineDistance));
      projectionPoint.slam.position.x = lineStart.slam.position.x +
                      t * (lineEnd.slam.position.x - lineStart.slam.position.x);
      projectionPoint.slam.position.y = lineStart.slam.position.y +
                      t * (lineEnd.slam.position.y - lineStart.slam.position.y);
      return projectionPoint;
    }

    static double calculatePointDistanceToLine(const Position& lineStart,
                                               const Position& lineEnd,
                                               const Position& point)
    {
      Position projectionPoint = calculatePointProjectionPositionToLine(lineStart, lineEnd, point);
      return point.distanceOf(projectionPoint);
    }

    static double calculatePointDistanceToLineSegment(const Position& lineStart,
                                                      const Position& lineEnd,
                                                      const Position& point)
    {
      Position projectionPoint = calculatePointProjectionPositionToLineSegment(lineStart, lineEnd, point);
      return point.distanceOf(projectionPoint);
    }

    //计算机器中心点的和区域的距离
    static double calculatePositionDistanceToPolygon(const Position &p, const std::vector<Position> &polygon)
    {
      double distance = std::numeric_limits<double>::max();
      if (isInside(polygon, p))
      {
        distance = 0.0f;
        return distance;
      }
      for (auto edgeCount = 0; edgeCount < polygon.size(); edgeCount++)
      {
        int edgeTarget = (edgeCount == (polygon.size() - 1) ? 0 : edgeCount + 1);
        double edgeDistance = calculatePointDistanceToLineSegment(polygon[edgeCount], polygon[edgeTarget], p);
        if (edgeDistance < distance)
        {
          distance = edgeDistance;
        }
      }
      return distance;
    }

    static std::pair<Position, Position> calculateLinearRegression(const std::vector<Position> &path)
    {
      int dataSize = path.size();
      std::pair<Position, Position> returnValue;
      double sumX, sumY, sumXSquared, sumYSquared, sumXY;

      for (auto pathIter = path.cbegin(); pathIter != path.cend(); pathIter++)
      {
        sumX += pathIter->slam.position.x;
        sumY += pathIter->slam.position.y;
        sumXSquared += pathIter->slam.position.x * pathIter->slam.position.x;
        sumYSquared += pathIter->slam.position.y * pathIter->slam.position.y;
        sumXY += pathIter->slam.position.x * pathIter->slam.position.y;
      }

      if (dataSize)
      {
        Position pos;
        if (fabs(double(dataSize) * sumXSquared - sumX * sumX) > std::numeric_limits<double>::epsilon())
        {
          double slope = (double(dataSize) * sumXY - sumY * sumX) /
                   (double(dataSize) * sumXSquared - sumX * sumX);
          double yIntercept = (sumY - slope * sumX) / double(dataSize);
          pos.slam.position.x = 0.0f;
          pos.slam.position.y = yIntercept;
          returnValue.first = pos;
          pos.slam.position.x = 1.0f;
          pos.slam.position.y = yIntercept + slope;
          returnValue.second = pos;
        }
        else
        {
          pos.slam.position.x = sumX / double(dataSize);
          pos.slam.position.y = 2.0f;
          returnValue.first = pos;
          pos.slam.position.x = sumX / double(dataSize);
          pos.slam.position.y = -2.0f;
          returnValue.second = pos;
        }
      }
      return returnValue;
    }

    static nlohmann::json mergeJson(nlohmann::json object, nlohmann::json sub)
    {
      nlohmann::json itemJson;
      if (!object.is_object() || !sub.is_object())
      {
          return itemJson;
      }
      itemJson = object;
      for (auto& item : sub.items())
      {
        if (item.value().is_object())
        {
          if (object.contains(item.key()) && object.at(item.key()).is_object())
          {
            itemJson[item.key()] = mergeJson(object.at(item.key()), item.value());
          }
          else if (!object.contains(item.key()))
          {
            itemJson[item.key()] = item.value();
          }
        }
        else if (!object.contains(item.key()))
        {
          itemJson[item.key()] = item.value();
        }
      }
      return itemJson;
    }
    /**
     * @brief 判断sub是否匹配object
     * @param object 超集对象
     * @param sub 子集匹配对象
     * @return sub能够与object相匹配
     * @remark 若sub匹配object，且object匹配sub，则两者完全相等
     */
    static bool sameJson(nlohmann::json object, nlohmann::json sub)
    {
      if (object.is_object() && sub.is_object())
      {
        if (object.size() != sub.size())
        {
          return false;
        }
        for (auto& item : sub.items())
        {
          if (!object.contains(item.key()))
          {
            return false;
          }
          if (!sameJson(object[item.key()], item.value()))
          {
            return false;
          }
        }
        return true;
      }
      else if (object.is_array() && sub.is_array())
      {
        if (object.size() != sub.size())
        {
          return false;
        }
        for (int i = 0; i < object.size(); i++)
        {
          if (!sameJson(object[i], sub[i]))
          {
            return false;
          }
        }
        return true;
      }
      else if (object.is_string() && sub.is_string())
      {
        return object.get<std::string>() == sub.get<std::string>();
      }
      else if (object.is_number_integer() && sub.is_number_integer())
      {
        return object.get<int>() == sub.get<int>();
      }
      else if (object.is_number_float() && sub.is_number_float())
      {
        return object.get<float>() == sub.get<float>();
      }
      else if (object.is_boolean() && sub.is_boolean())
      {
        return object.get<bool>() == sub.get<bool>();
      }
      else
      {
        return object.is_null() && sub.is_null();
      }
    }

    /**
     * @brief 比较object与sub，并返回差异对象
     * @param object 待比较的超集对象
     * @param sub 子集比较对象
     * @return 差异对象
     * @remark 如 object: {"a":1, "b":{"c":2}, "d":3}, sub: {"a":1, "b":{"c":{"e":7}, "f":4}}, 其结果: {"b":{"c":{"e":7}, "f":4}}
     */
    static nlohmann::json contrastJson(nlohmann::json object, nlohmann::json sub)
    {
      nlohmann::json contrast;

      if (sub.is_object())
      {
        for (auto& item : sub.items())
        {
          if (!object.is_object() || !object.contains(item.key())
              || (object[item.key()].is_object() && !item.value().is_object())
              || (!object[item.key()].is_object() && item.value().is_object())
              || (!object[item.key()].is_object() && !item.value().is_object() && !sameJson(object[item.key()], item.value())))
          {
            contrast[item.key()] = item.value();
          }
          else
          {
            auto o = contrastJson(object[item.key()], item.value());
            if (o.size())
            {
              contrast[item.key()] = o;
            }
          }
        }
      }
      return contrast;
    }

    static bool isTagsMatched(const std::vector<std::string>& firstTags, const std::vector<std::string>& secondTags)
    {
      if (firstTags.empty() || secondTags.empty())
      {
        return true;
      }
      auto tempRobotTags = firstTags;
      auto tempHiveTags = secondTags;

      std::sort(tempRobotTags.begin(), tempRobotTags.end());
      std::sort(tempHiveTags.begin(), tempHiveTags.end());
      if (tempRobotTags.size() > tempHiveTags.size())
      {
        return std::includes(tempRobotTags.begin(), tempRobotTags.end(), tempHiveTags.begin(), tempHiveTags.end());
      }
      return std::includes(tempHiveTags.begin(), tempHiveTags.end(), tempRobotTags.begin(), tempRobotTags.end());
    }

    static bool isOperationModeMatched(const std::string& operationModeFirst, const std::string& operationModeSecond)
    {
      if (operationModeFirst.empty() || operationModeSecond.empty())
      {
        return true;
      }
      auto operationModeFirstEnum = toOperationModeEnum(operationModeFirst);
      auto operationModeSecondEnum = toOperationModeEnum(operationModeSecond);
      auto modePair = std::make_pair(operationModeFirstEnum, operationModeSecondEnum);
      if (operationModeMatchTable.find(modePair) == operationModeMatchTable.end())
      {
        return false;
      }
      return operationModeMatchTable[modePair];
    }
  }
}

#endif
