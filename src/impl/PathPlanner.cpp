#include "deviceMiscellaneous/PathPlanner.h"

namespace cti
{
  namespace missionSchedule
  {
    class PathPlanner : public IPathPlanner
    {
      struct GraphInfo
      {
        std::map<uint, std::pair<long unsigned int, long unsigned int>> lineVertexMap;
        std::map<uint, std::pair<Position, Position>> linePositionMap;
      };
      struct VertexInfo
      {
        Position topoPosition;
      };
      typedef boost::adjacency_list< boost::vecS,
                                     boost::vecS,
                                     boost::undirectedS,
                                     VertexInfo,
                                     boost::property<boost::edge_weight_t, double>,
                                     GraphInfo> PathGraph;
      typedef typename boost::graph_traits<PathGraph>::vertex_descriptor PathVertex;
      private:
        std::string mapFilePrefix_;
        std::unordered_map<std::string, PathGraph> map_;

        bool loadNavMap(const std::string& building, const std::string& floor)
        {
          PathGraph topoGraph;
          cti::common::Graph topoInfo;
          std::string mapName = building + floor;
          std::map<uint, std::pair<Position, Position>> linePositionMap;
          std::map<uint, std::pair<PathVertex, PathVertex>> lineVertexMap;
          std::string mapLocation = mapFilePrefix_ + building + "/" + floor + ".topo";
          if (!cti::common::ReadProtoStringFromFile(mapLocation, topoInfo))
          {
            SPDLOG_INFO("PathPlanner can't read building {} floor {} topoFile", mapLocation);
            return false;
          }

          for (auto lineCount = 0; lineCount < topoInfo.vertex_size(); lineCount++)
          {
            if (topoInfo.vertex(lineCount).path_size() < 2)
            {
              continue;
            }
            auto line = topoInfo.vertex(lineCount);
            uint lineId = line.id();
            PathVertex startVertex, endVertex;
            Position startPosition, endPosition;
            bool findStart{false}, findEnd{false};
            startPosition.slam.position.x = line.path(0).x();
            startPosition.slam.position.y = line.path(0).y();
            endPosition.slam.position.x = line.path(line.path_size() - 1).x();
            endPosition.slam.position.y = line.path(line.path_size() - 1).y();
            for (auto preCount = 0; preCount < line.predecessors_size(); preCount++)
            {
              uint preLineId = line.predecessors(preCount);
              if (linePositionMap.find(preLineId) == linePositionMap.end())
              {
                continue;
              }
              if (startPosition.distanceOf(linePositionMap[preLineId].first) < 0.05f)
              {
                findStart = true;
                startVertex = lineVertexMap[preLineId].first;
                break;
              }
              if (startPosition.distanceOf(linePositionMap[preLineId].second) < 0.05f)
              {
                findStart = true;
                startVertex = lineVertexMap[preLineId].second;
                break;
              }
            }
            for (auto sucCount = 0; sucCount < line.successors_size(); sucCount++)
            {
              uint sucLineId = line.successors(sucCount);
              if (linePositionMap.find(sucLineId) == linePositionMap.end())
              {
                continue;
              }
              if (endPosition.distanceOf(linePositionMap[sucLineId].first) < 0.05f)
              {
                findEnd = true;
                endVertex = lineVertexMap[sucLineId].first;
                break;
              }
              if (endPosition.distanceOf(linePositionMap[sucLineId].second) < 0.05f)
              {
                findEnd = true;
                endVertex = lineVertexMap[sucLineId].second;
                break;
              }
            }
            if (!findEnd)
            {
              endVertex = boost::add_vertex(topoGraph);
              topoGraph[endVertex].topoPosition = endPosition;
            }
            if (!findStart)
            {
              startVertex = boost::add_vertex(topoGraph);
              topoGraph[startVertex].topoPosition = startPosition;
            }

            double edgeCost = startPosition.distanceOf(endPosition);
            boost::add_edge(startVertex, endVertex, {edgeCost}, topoGraph);
            lineVertexMap.emplace(lineId, std::make_pair(startVertex, endVertex));
            linePositionMap.emplace(lineId, std::make_pair(startPosition, endPosition));
          }
          topoGraph[boost::graph_bundle].lineVertexMap = lineVertexMap;
          topoGraph[boost::graph_bundle].linePositionMap = linePositionMap;
          map_.emplace(mapName, topoGraph);
          return true;
        }

      public:
        PathPlanner() {}
        ~PathPlanner() {}

        void initialize()
        {
          mapFilePrefix_ = "/home/cti/.ros/map_center/cloudmap/";
        }

        double calculatePathCost(const Position& start, const Position& goal, const std::string& floor) override
        {
          auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<IDeviceRuntimeUtility>();
          auto currentBuilding = deviceUtility->getRobotInfo()->buildingName();
          if (map_.find(currentBuilding + floor) == map_.end())
          {
            if (!loadNavMap(currentBuilding, floor))
            {
              SPDLOG_INFO("PathPlanner can't load building {} floor {} navMap", currentBuilding, floor);
              return 1000;
            }
          }
          uint startLine, goalLine;
          auto navGraph = map_.at(currentBuilding + floor);
          auto lineVertexMap = navGraph[boost::graph_bundle].lineVertexMap;
          auto linePositionMap = navGraph[boost::graph_bundle].linePositionMap;
          double goalMinLineDistance = std::numeric_limits<double>::max();
          double startMinLineDistance = std::numeric_limits<double>::max();
          for (auto lineIter = linePositionMap.begin(); lineIter != linePositionMap.end(); lineIter++)
          {
            double startDistance = calculatePointDistanceToLineSegment(lineIter->second.first, lineIter->second.second, start);
            double goalDistance = calculatePointDistanceToLineSegment(lineIter->second.first, lineIter->second.second, goal);
            if (startDistance < startMinLineDistance)
            {
              startLine = lineIter->first;
              startMinLineDistance = startDistance;
            }
            if (goalDistance < goalMinLineDistance)
            {
              goalLine = lineIter->first;
              goalMinLineDistance = goalDistance;
            }
          }
          auto goalValue = calculatePointProjectionValueToLine(linePositionMap[goalLine].first,
                                                               linePositionMap[goalLine].second,
                                                               goal);
          auto startValue = calculatePointProjectionValueToLine(linePositionMap[startLine].first,
                                                                linePositionMap[startLine].second,
                                                                start);
          auto goalVertex = boost::add_vertex(navGraph);
          auto startVertex = boost::add_vertex(navGraph);
          if (startValue > 0.0 && startValue < 1.0)
          {
            double edgeCost = start.distanceOf(linePositionMap[startLine].first);
            boost::add_edge(startVertex, lineVertexMap[startLine].first, {edgeCost}, navGraph);
            edgeCost = start.distanceOf(linePositionMap[startLine].second);
            boost::add_edge(startVertex, lineVertexMap[startLine].second, {edgeCost}, navGraph);
          }
          else if (startValue <= 0.0)
          {
            double edgeCost = start.distanceOf(linePositionMap[startLine].first);
            boost::add_edge(startVertex, lineVertexMap[startLine].first, {edgeCost}, navGraph);
          }
          else
          {
            double edgeCost = start.distanceOf(linePositionMap[startLine].second);
            boost::add_edge(startVertex, lineVertexMap[startLine].second, {edgeCost}, navGraph);
          }

          if (goalValue > 0.0 && goalValue < 1.0)
          {
            double edgeCost = goal.distanceOf(linePositionMap[goalLine].first);
            boost::add_edge(goalVertex, lineVertexMap[goalLine].first, {edgeCost}, navGraph);
            edgeCost = goal.distanceOf(linePositionMap[goalLine].second);
            boost::add_edge(goalVertex, lineVertexMap[goalLine].second, {edgeCost}, navGraph);
          }
          else if (goalValue <= 0.0)
          {
            double edgeCost = goal.distanceOf(linePositionMap[goalLine].first);
            boost::add_edge(goalVertex, lineVertexMap[goalLine].first, {edgeCost}, navGraph);
          }
          else
          {
            double edgeCost = goal.distanceOf(linePositionMap[goalLine].second);
            boost::add_edge(goalVertex, lineVertexMap[goalLine].second, {edgeCost}, navGraph);
          }
          if (startLine == goalLine && startValue > 0.0 && startValue < 1.0 && goalValue > 0.0 && goalValue < 1.0)
          {
            double edgeCost = goal.distanceOf(start);
            boost::add_edge(goalVertex, startVertex, {edgeCost}, navGraph);
            return edgeCost;
          }
          std::vector<PathVertex> p(boost::num_vertices(navGraph));
          std::vector<double> d(boost::num_vertices(navGraph));
          boost::dijkstra_shortest_paths(navGraph, startVertex, boost::predecessor_map(&p[0]).distance_map(&d[0]));
          return d[goalVertex];
        }

        std::vector<Position> calculatePath(const Position& start, const Position& goal, const std::string& floor) override
        {
          std::vector<Position> path;
          return path;
        }
    };

    std::shared_ptr<IPathPlanner> createPathPlanner()
    {
      auto planner = std::make_shared<PathPlanner>();
      planner->initialize();
      return std::dynamic_pointer_cast<IPathPlanner>(planner);
    }
  }
}