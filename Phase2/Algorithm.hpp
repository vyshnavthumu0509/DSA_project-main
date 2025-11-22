#include "Graph.hpp"
#include<queue>
#include<set>
#include <algorithm>
#include<unordered_map>
struct path{
    std::vector<int> edges; //edge ids in order
    std::vector<int> nodes; //node ids in order
    double cost;    //total cost of the path
    bool operator >(const path&other) const{
        return cost>other.cost;
    }
    bool operator <(const path&other) const{
        return cost<other.cost;
    }
    bool operator==(const path&other) const {
    return nodes == other.nodes && edges == other.edges;
    }
};
class Yen{
    private:
    const Graph*graph;
    std::unordered_map<int, double> heuristic_map;
    std::unordered_map<int, std::vector<int>> reverse_adj_list;
    void buildReverseGraph();
    void computeHeuristics(int target_id);
    struct AStarNode {
        int id;
        double g_score; 
        double f_score; 
        int parent_node;
        int parent_edge;

        bool operator>(const AStarNode& other) const {
            return f_score > other.f_score;
        }
    };
    path a_star(int start,int end,const std::set<int>&banned_edges,const std::set<int>&banned_nodes);
    public:
    Yen(const Graph*g):graph(g){buildReverseGraph();}
    std::vector<path> findK_paths(int start,int end,int k);

};
struct PathResult {
    std::vector<int> path; // sequence of Node IDs
    double length;         // total length in meters
};

// Struct for Task 3 results
struct ApproxResult {
    int source;
    int target;
    double distance;
};

class Algorithm {

public:

    static std::vector<path> kShortestPathsHeuristic(
        const Graph& graph, 
        int source_id, 
        int target_id, 
        int k, 
        double overlap_threshold_pct
    );

    static void precomputeLandmarks(const Graph& graph);

        static double approximateShortestPathDistance(
        const Graph& graph, 
        int source_id, 
        int target_id
    );
private:
    // Internal helper: Dijkstra's algorithm that accepts dynamic edge penalties
    static bool getShortestPathWithPenalty(
        const Graph& graph,
        int source_id,
        int target_id,
        const std::unordered_map<int, double>& edge_penalties,
        path& result
    );

    static std::vector<std::vector<double>> landmarks_to_nodes; // dist[L_idx][node_idx]
    static std::vector<std::vector<double>> nodes_to_landmarks; // dist[node_idx][L_idx]
    static std::vector<int> landmark_indices;
    static bool is_precomputed;
};