#ifndef ALGORITHMS_HPP
#define ALGORITHMS_HPP
#include "Graph.hpp"
#include <vector>
#include <unordered_set>
#include<string>
struct PathResult{
    bool possible;
    double cost;
    std::vector<int> path;
};
struct Constraints{
    std::unordered_set<int> forbidden_nodes;
    std::unordered_set<std::string> forbidden_road_types;

};
class Algorithms{
    private:
    static int findNearestNode(const Graph&graph,
    double lat,double lon);
    public:
    static double speed(const Graph&graph);

    static PathResult shortestPathDistance(
        const Graph&graph,
        int src,
        int dst,
        const Constraints&connstraints
    );
    static PathResult shortestPathTime(
        const Graph&graph,
        int src,
        int dst,
        const Constraints&constrains
    );
    static std::vector<int> knnEuclidean(
        const Graph &graph,
        double query_lat,
        double query_lon,
        const std::string &poi_type,
        int k
    );
    static std::vector<int> knnShortestPath(
        const Graph &graph,
        double query_lat,
        double query_lon,
        const std::string&poi_type,
        int k
    );
};
#endif