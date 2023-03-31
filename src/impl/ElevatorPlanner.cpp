#include <shared_mutex>
#include "cti_msgs/CloudElevatorInfo.h"
#include "std_msgs/String.h"
#include "deviceMiscellaneous/ElevatorPlanner.h"
#include "std_msgs/String.h"

namespace cti
{
  namespace missionSchedule
  {
    class ElevatorPlanner : public IElevatorPlanner, public std::enable_shared_from_this<ElevatorPlanner>
    {
      struct VertexInfo
      {
        int floorNumber;
        std::string floor;
        std::string elevatorId;
        Position elevatorPosition;
        Position elevatorEnterPosition;
        std::vector<Position> elevatorPolygon;
      };

      // Elevator Edge weight.
      struct EdgeInfo
      {
        double waitTime;
        std::vector<std::string> elevatorIds;
      };

      // Elevator Graph.
      typedef boost::adjacency_list< boost::vecS,
                                     boost::vecS,
                                     boost::undirectedS,
                                     VertexInfo,
                                     EdgeInfo > ElevatorGraph;
      typedef typename boost::graph_traits<ElevatorGraph>::vertex_descriptor ElevatorVertex;
      typedef typename boost::graph_traits<ElevatorGraph>::edge_descriptor ElevatorEdge;
      struct VertexWriter
      {
        VertexWriter(const ElevatorGraph& _g) : g (_g) {};
        template <class Vertex> void operator()(std::ostream& out, Vertex v) {
                out << " [label=\"" << g[v].elevatorId + "." + g[v].floor << "\"]" << std::endl;
        };
        ElevatorGraph g;
      };
      struct EdgeWriter
      {
        EdgeWriter(const ElevatorGraph& _g) : g (_g) {};
        template <class Edge> void operator()(std::ostream& out, Edge e) {
                out << " [label=\"" << g[e].waitTime << "\"]" << std::endl;
        };
        ElevatorGraph g;
      };
      private:
        std::shared_timed_mutex mutex_;
        std::unordered_map<std::string, std::vector<std::string>> elevatorTags_;
        std::unordered_map<std::string, std::vector<ElevatorVertex>> floorVertices_;
          //string: floor; vector: elevator vertices on the same floor 每层楼的电梯节点
        std::unordered_map<std::string, std::vector<ElevatorVertex>> shaftVertices_;
          //string: elevator.id; vector: elevator vertices along the same shaft (elevator) 每部电梯构造的节点（垂直方向）
        std::unordered_map<std::string, std::string> elevatorDeviceIdMappingWithElevatorId_;
        ElevatorGraph originalElevatorGraph_;
        ElevatorGraph updatedElevatorGraph_;
        ros::Subscriber buildingElevatorSubscriber_;
        ros::Subscriber cloudElevatorStateUpdateSubscriber_;
        ros::Subscriber buildingElevatorTagSubscriber_;
      public:
        ElevatorPlanner() {}
        ~ElevatorPlanner() {}

        bool initialize()
        {
          if (auto nodeHandle = cti::missionSchedule::common::getContainer()->resolveOrNull<ros::NodeHandle>())
          {
            buildingElevatorSubscriber_ = nodeHandle->subscribe("/cloud_scheduling_node/cache/elevators_info", 1, &ElevatorPlanner::constructElevatorGraph, this);
            cloudElevatorStateUpdateSubscriber_ = nodeHandle->subscribe("/cloud_scheduling_node/elevator_platform_update", 10, &ElevatorPlanner::onCloudElevatorUpdate, this);
            buildingElevatorTagSubscriber_ = nodeHandle->subscribe("/cloud_scheduling_node/cache/elevators_json", 1, &ElevatorPlanner::updateElevatorTags, this);
          }
          return true;
        }

        void onCloudElevatorUpdate(const cti_msgs::CloudElevatorInfo& elevator)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          SPDLOG_INFO("ElevatorPlaner get cloud elevator update ");
          std::string elevatorId = elevator.id;
          deleteElevator(elevatorId);
          if (elevator.enabled)
          {
            addElevator(elevator);
          }
        }

        bool isRobotLocationOccupyElevator(const std::string& floor, const Position& robotLocation) override
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          if (floorVertices_.find(floor) == floorVertices_.end())
          {
            return false;
          }

          for (auto vertex : floorVertices_[floor])
          {
            if (!originalElevatorGraph_[vertex].elevatorPolygon.empty())
            {
              if (cti::missionSchedule::isInside(originalElevatorGraph_[vertex].elevatorPolygon, robotLocation))
              {
                SPDLOG_INFO("ElevatorPlanner robot inside elevator {} floor {} area", originalElevatorGraph_[vertex].elevatorId, floor);
                return true;
              }
            }
          }
          return false;
        }

        void updateElevatorTags(const std_msgs::String& msg)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          nlohmann::json elevatorJson = nlohmann::json::parse(msg.data);
          if (elevatorJson.is_array())
          {
            elevatorTags_.clear();
            for (auto elevatorInfo : elevatorJson)
            {
              if (elevatorInfo.contains("id") && elevatorInfo.contains("tags") && elevatorInfo["id"].is_string() && elevatorInfo["tags"].is_array())
              {
                std::string elevatorId = elevatorInfo["id"].get<std::string>();
                std::vector<std::string> elevatorTags;
                for (auto tag : elevatorInfo["tags"])
                {
                  if (!tag.is_null() && tag.is_string())
                  {
                    elevatorTags.push_back(tag.get<std::string>());
                    SPDLOG_INFO("ElevatorPlanner elevator {} tag {} ", elevatorId, tag);
                  }
                }

                elevatorTags_[elevatorId] = elevatorTags;
              }
            }
          }
        }

        void constructElevatorGraph(const cti_msgs::CloudBuildingInfo& msg)
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          originalElevatorGraph_.clear();
          elevatorDeviceIdMappingWithElevatorId_.clear();
          floorVertices_.clear();
          shaftVertices_.clear();
          for (auto elevator : msg.elevators)
          {
            addElevator(elevator);
          }
          // std::cout << "The elvator graph has been initialized as follows: \n";
          // printGraph(originalElevatorGraph_);
          // std::string filename = "originalElevatorGraph.dot";
          // std::cout << "The elvator graph has been initialized and saved to " << filename << '\n';
          // showElevatorGraph(originalElevatorGraph_, filename);
        }

        bool updateElevatorGraph() override
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          updatedElevatorGraph_ = originalElevatorGraph_;

          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          std::string robotId = deviceUtility->getRobotInfo()->robotId();
          std::string currentFloor = deviceUtility->getRobotInfo()->currentFloor();
          deviceUtility->foreachSystemRobot([self = shared_from_this(), currentFloor, robotId](std::shared_ptr<RobotUtility> robot)
          {
            if (robot->robotId() == robotId)
            {
              return false;
            }
            if (RobotState::LIFTING != robot->robotState() && (!(robot->currentFloor() == currentFloor && RobotState::BUSY == robot->robotState())))
            {
              return false;
            }

            std::string elevatorId = robot->currentElevatorId();
            if (elevatorId.empty() && !robot->currentElevatorDeviceId().empty() &&
                self->elevatorDeviceIdMappingWithElevatorId_.find(robot->currentElevatorDeviceId()) != self->elevatorDeviceIdMappingWithElevatorId_.end())
            {
              elevatorId = self->elevatorDeviceIdMappingWithElevatorId_[robot->currentElevatorDeviceId()];
            }
            if (elevatorId.empty() && robot->currentFloor() == currentFloor && RobotState::BUSY == robot->robotState())
            {
              for (auto elevatorIter = self->floorVertices_[currentFloor].begin(); elevatorIter != self->floorVertices_[currentFloor].end(); elevatorIter++)
              {
                if (!self->originalElevatorGraph_[*elevatorIter].elevatorId.empty() &&
                    self->originalElevatorGraph_[*elevatorIter].elevatorEnterPosition.distanceOf(robot->localDestination()) <= 0.5f)
                {
                  elevatorId = self->originalElevatorGraph_[*elevatorIter].elevatorId;
                  break;
                }
              }
            }
            if (elevatorId.empty() || self->shaftVertices_[elevatorId].empty())
            {
              return false;
            }
            for (auto it = self->shaftVertices_[elevatorId].begin(); it != --(self->shaftVertices_[elevatorId].end()); it++)
            {
              ElevatorEdge e = boost::edge(*it, *(it + 1), self->updatedElevatorGraph_).first;
              self->updatedElevatorGraph_[e].waitTime += 10.0;
            }
            return false;
          });
          // std::string filename = "updatedElevatorGraph.dot";
          // std::cout << "The elvator graph has been updated and saved to " << filename << '\n';
          // printGraph(updatedElevatorGraph_);
          // showElevatorGraph(updatedElevatorGraph_, filename);
          return true;
        }

        Position getElevatorInsidePosition(const std::string& elevatorId, const std::string& floor) override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          Position elevatorPosition;
          if (floorVertices_.find(floor) != floorVertices_.end())
          {
            for (auto elevatorIter = floorVertices_[floor].begin(); elevatorIter != floorVertices_[floor].end(); elevatorIter++)
            {
              if (!originalElevatorGraph_[*elevatorIter].elevatorId.empty() && originalElevatorGraph_[*elevatorIter].elevatorId == elevatorId)
              {
                elevatorPosition = originalElevatorGraph_[*elevatorIter].elevatorPosition;
                break;
              }
            }
          }
          return elevatorPosition;
        }

        Position getElevatorEnterPosition(const std::string& elevatorId, const std::string& floor) override
        {
          std::shared_lock<std::shared_timed_mutex> lk(mutex_);
          Position elevatorPosition;
          if (floorVertices_.find(floor) != floorVertices_.end())
          {
            for (auto elevatorIter = floorVertices_[floor].begin(); elevatorIter != floorVertices_[floor].end(); elevatorIter++)
            {
              if (!originalElevatorGraph_[*elevatorIter].elevatorId.empty() && originalElevatorGraph_[*elevatorIter].elevatorId == elevatorId)
              {
                elevatorPosition = originalElevatorGraph_[*elevatorIter].elevatorEnterPosition;
                break;
              }
            }
          }
          return elevatorPosition;
        }

        ElevatorPlan calculateBestElevatorPlan(const Position& currentPosition,
                                               const std::string& currentFloor,
                                               const StationModel& targetStation,
                                               bool usePathCost,
                                               const std::vector<std::string>& tags) override
        {
          std::unique_lock<std::shared_timed_mutex> lk(mutex_);
          ElevatorPlan plan;
          double waitTime = 20;
          // updateElevatorGraph();
          std::string goalFloor = targetStation.floor();
          if (currentFloor == goalFloor)
          {
            return plan;
          }

          ElevatorGraph updatedElevatorGraph = updatedElevatorGraph_;

          ElevatorVertex startVertex = boost::add_vertex(updatedElevatorGraph);
          updatedElevatorGraph[startVertex].floor = currentFloor;
          ElevatorVertex goalVertex = boost::add_vertex(updatedElevatorGraph);
          updatedElevatorGraph[goalVertex].floor = targetStation.floor();
          if (floorVertices_.find(currentFloor) != floorVertices_.end())
          {
            for (auto otherVertex : floorVertices_[currentFloor])
            {
              if (usePathCost)
              {
                auto pathPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<cti::missionSchedule::IPathPlanner>();
                waitTime = pathPlanner->calculatePathCost(currentPosition,
                                                          updatedElevatorGraph[otherVertex].elevatorEnterPosition,
                                                          currentFloor);
                if (waitTime >= 600)
                {
                  SPDLOG_INFO("ElevatorPlanner compute floor {} start {} goal {} cost {}", currentFloor,
                    currentPosition.toJson().dump(), updatedElevatorGraph[otherVertex].elevatorEnterPosition.toJson().dump(), waitTime);
                  waitTime = 600.0;
                }
              }

              waitTime = waitTime / 10.0;
              std::string otherVertexElevatorId = originalElevatorGraph_[otherVertex].elevatorId;
              boost::add_edge(startVertex, otherVertex, {waitTime, {otherVertexElevatorId}}, updatedElevatorGraph);
            }
          }
          waitTime = 20.0;
          if (floorVertices_.find(goalFloor) != floorVertices_.end())
          {
            for (auto otherVertex : floorVertices_[goalFloor])
            {
              if (usePathCost)
              {
                auto pathPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<cti::missionSchedule::IPathPlanner>();
                waitTime = pathPlanner->calculatePathCost(updatedElevatorGraph[otherVertex].elevatorEnterPosition,
                                                          targetStation.coordinate(),
                                                          goalFloor);
                if (waitTime >= 600)
                {
                  SPDLOG_INFO("ElevatorPlanner compute floor {} start {} goal {} cost {}", currentFloor,
                    targetStation.coordinate().toJson().dump(), updatedElevatorGraph[otherVertex].elevatorEnterPosition.toJson().dump(), waitTime);
                  waitTime = 600.0;
                }
              }
              waitTime = waitTime / 10.0;
              std::string otherVertexElevatorId = originalElevatorGraph_[otherVertex].elevatorId;
              boost::add_edge(otherVertex, goalVertex, {waitTime, {otherVertexElevatorId}}, updatedElevatorGraph);
            }
          }
          auto weightMap = boost::make_transform_value_property_map(
            [tags, elevatorTags = elevatorTags_](EdgeInfo& e)
            {
              if (tags.empty())
              {
                return e.waitTime;
              }
              double edgeCost = e.waitTime;
              for (auto elevatorCount = 0; elevatorCount < e.elevatorIds.size(); elevatorCount++)
              {
                if (elevatorTags.find(e.elevatorIds[elevatorCount]) != elevatorTags.end())
                {
                  if (!isTagsMatched(tags, elevatorTags.at(e.elevatorIds[elevatorCount])))
                  {
                    edgeCost = 1000;
                    break;
                  }
                }
              }
              return edgeCost;
            }, boost::get(boost::edge_bundle, updatedElevatorGraph));
          std::vector<ElevatorVertex> predecessor(boost::num_vertices(updatedElevatorGraph));
          std::vector<double> distance(boost::num_vertices(updatedElevatorGraph));
          boost::dijkstra_shortest_paths(updatedElevatorGraph, startVertex, boost::predecessor_map(&predecessor[0]).
            distance_map(&distance[0]).weight_map(weightMap));
          auto pathNode = goalVertex;
          if (pathNode == predecessor[pathNode])
          {
            SPDLOG_INFO("ElevatorPlanner can't find solution to station {}", targetStation.name());
            plan.cost = 1000;
            return plan;
          }
          while (pathNode != startVertex)
          {
            if (pathNode != goalVertex && predecessor[pathNode] != startVertex)
            {
              if (updatedElevatorGraph[pathNode].floor == updatedElevatorGraph[predecessor[pathNode]].floor)
              {
                plan.switchElevatorPath.push_back({updatedElevatorGraph[pathNode].floor,
                                                   updatedElevatorGraph[pathNode].elevatorId,
                                                   updatedElevatorGraph[pathNode].elevatorPosition,
                                                   updatedElevatorGraph[pathNode].elevatorEnterPosition});
              }
            }
            SPDLOG_INFO("ElevatorPlanner find solution elevatorId {} floor {}", updatedElevatorGraph[pathNode].elevatorId, updatedElevatorGraph[pathNode].floor);
            pathNode = predecessor[pathNode];
            if (pathNode == startVertex)
            {
              break;
            }
            plan.planSteps.push_back({updatedElevatorGraph[pathNode].floor,
                                      updatedElevatorGraph[pathNode].elevatorId,
                                      updatedElevatorGraph[pathNode].elevatorPosition,
                                      updatedElevatorGraph[pathNode].elevatorEnterPosition});
          }
          std::reverse(plan.planSteps.begin(), plan.planSteps.end());
          std::reverse(plan.switchElevatorPath.begin(), plan.switchElevatorPath.end());
          plan.cost = distance[goalVertex];
          return plan;
        }

        bool printGraph(const ElevatorGraph &_graph)
        {
          // write_graphviz(std::cout, _graph, boost::make_label_writer(&elevatorIDs_[0]));
          boost::graph_traits<ElevatorGraph>::vertex_iterator v_it, v_itEnd;
          int i = 0;
          std::cout << "Number of vertices: " << boost::num_vertices(_graph) << '\n';
          for(boost::tie(v_it, v_itEnd) = boost::vertices(_graph); v_it != v_itEnd; ++v_it, ++i)
          {
            std::cout << "Vertex " << i << ": Elevator " <<_graph[*v_it].elevatorId << ", floor " << _graph[*v_it].floor << '\n';
          }

          boost::graph_traits<ElevatorGraph>::edge_iterator e_it, e_itEnd;
          std::cout << "Number of edges: "<< boost::num_edges(_graph) << '\n';
          for(boost::tie(e_it, e_itEnd) = boost::edges(_graph); e_it != e_itEnd; ++e_it)
          {
            std::cout << boost::source(*e_it, _graph) << "<->" << boost::target(*e_it, _graph) << ": wait time " << _graph[*e_it].waitTime << "s" << '\n';
          }
          return true;
        }

        void showElevatorGraph(const ElevatorGraph& _graph, const std::string& _filename)
        {
          std::ofstream gout;
          gout.open(_filename);
          boost::write_graphviz(gout, _graph, VertexWriter(_graph), EdgeWriter(_graph));
        }

        void deleteElevator(const std::string& elevatorId)
        {
          SPDLOG_INFO("finding {}", elevatorId);
          for (auto thisFloorVertices = floorVertices_.begin(); thisFloorVertices != floorVertices_.end();)
          {
            SPDLOG_INFO("floor: {}", thisFloorVertices->first);
            for (auto elevatorIter = thisFloorVertices->second.begin(); elevatorIter != thisFloorVertices->second.end();)
            {
              SPDLOG_INFO("  elevatorId: {}", originalElevatorGraph_[*elevatorIter].elevatorId);
              if (originalElevatorGraph_[*elevatorIter].elevatorId == elevatorId)
              {
                SPDLOG_INFO("  originalElevatorGraph delete floorVertices[{}] elevator [{}]", thisFloorVertices->first, elevatorId);
                originalElevatorGraph_.removing_vertex(*elevatorIter, originalElevatorGraph_);
                thisFloorVertices->second.erase(elevatorIter);
                continue;
              }
              elevatorIter++;
            }
            if (thisFloorVertices->second.size() == 0)
            {
              SPDLOG_INFO("floor is empty erase floor {}", thisFloorVertices->first);
              floorVertices_.erase(thisFloorVertices++);
              continue;
            }
            thisFloorVertices++;
          }
          auto targetElevatorVertex = shaftVertices_.find(elevatorId);
          if (targetElevatorVertex != shaftVertices_.end())
          {
            shaftVertices_.erase(targetElevatorVertex);
            SPDLOG_INFO("erase shaftVertices_'s {}", elevatorId);
          }
        }

        void addElevator(const cti_msgs::CloudElevatorInfo& elevator)
        {
          SPDLOG_INFO("ElevatorPlaner add elevator {}", elevator.id);
          for (auto waypoint : elevator.wayponts)
          {
            if (waypoint.floor.empty())
            {
              continue;
            }
            int floorNumber = std::atoi(waypoint.floor.data());
            ElevatorVertex currentVertex = boost::add_vertex(originalElevatorGraph_);
            originalElevatorGraph_[currentVertex].floor = waypoint.floor;
            originalElevatorGraph_[currentVertex].floorNumber = floorNumber;
            originalElevatorGraph_[currentVertex].elevatorId = elevator.id;
            originalElevatorGraph_[currentVertex].elevatorPosition.transferFromPose(waypoint.inside);
            originalElevatorGraph_[currentVertex].elevatorEnterPosition.transferFromPose(waypoint.wait);
            if (!originalElevatorGraph_[currentVertex].elevatorEnterPosition.valid())
            {
              originalElevatorGraph_[currentVertex].elevatorEnterPosition = originalElevatorGraph_[currentVertex].elevatorPosition;
              originalElevatorGraph_[currentVertex].elevatorEnterPosition.shift(waypoint.waitPoseOffsetV, 0.0, 0.0);
            }
            if (floorVertices_.find(waypoint.floor) != floorVertices_.end())
            {
              double waitTime = 60.0;
              for (auto otherVertex : floorVertices_[waypoint.floor])
              {
                boost::add_edge(currentVertex, otherVertex, {waitTime}, originalElevatorGraph_);
              }
            }
            floorVertices_[waypoint.floor].emplace_back(currentVertex);
            shaftVertices_[elevator.id].emplace_back(currentVertex);
          }
          if (shaftVertices_[elevator.id].empty())
          {
            return;
          }
          elevatorDeviceIdMappingWithElevatorId_[elevator.deviceId] = elevator.id;
          std::sort(shaftVertices_[elevator.id].begin(), shaftVertices_[elevator.id].end(),
                    [graph = originalElevatorGraph_](ElevatorVertex const& L, ElevatorVertex const& R) -> bool
                    { return graph[L].floorNumber < graph[R].floorNumber; });

          double waitTime = 1.0;
          for (auto it = shaftVertices_[elevator.id].begin(); it != --shaftVertices_[elevator.id].end(); it++)
          {
            waitTime = std::abs(originalElevatorGraph_[*it].floorNumber - originalElevatorGraph_[*(it + 1)].floorNumber);
            waitTime += 1 / waitTime;
            boost::add_edge(*it, *(it + 1), {waitTime}, originalElevatorGraph_);
          }
        }
    };

    std::shared_ptr<IElevatorPlanner> createElevatorPlanner()
    {
      auto planner = std::make_shared<ElevatorPlanner>();
      planner->initialize();
      return std::dynamic_pointer_cast<IElevatorPlanner>(planner);
    }
  }
}
