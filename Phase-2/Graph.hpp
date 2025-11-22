#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <limits>
#include <algorithm>

const double INF = std::numeric_limits<double>::infinity();

struct Node {
    int id;
    double lat;
    double lon;
    std::vector<std::string> pois;
};

struct Edge {
    int id;
    int u;
    int v;
    double length;
    double average_time;
    std::vector<double> speed_profile; // 96 slots for 15-min intervals
    bool oneway;
    std::string road_type;
    bool is_removed;
    
    Edge() : id(-1), u(-1), v(-1), length(0), average_time(0), 
             oneway(false), is_removed(false) {}
};

class Graph {
private:
    std::vector<Node> nodes;
    std::unordered_map<int, Edge> edges; // edge_id -> Edge
    std::unordered_map<int, std::vector<int>> adj_list; // node_id -> list of edge_ids
    std::unordered_map<int, int> node_id_to_index;
    
public:
    void addNode(const Node& node);
    void addEdge(const Edge& edge);
    void removeEdge(int edge_id);
    void modifyEdge(int edge_id, const Edge& patch);
    
    const Node* getNode(int node_id) const;
    const Edge* getEdge(int edge_id) const;
    const std::vector<int>& getAdjacentEdges(int node_id) const;
    
    int getNodeCount() const { return nodes.size(); }
    const std::vector<Node>& getNodes() const { return nodes; }
    
    double getEdgeTime(int edge_id, double start_time = 0.0) const;
    double euclideanDistance(double lat1, double lon1, double lat2, double lon2) const;
};
#endif