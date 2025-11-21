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
    Yen(const Graph*g):graph(g){}
    std::vector<path> findK_paths(int start,int end,int k);

};

class Heuristic{

public:

    static std::vector<path> kShortestPathsHeuristic(
        const Graph& graph, 
        int source_id, 
        int target_id, 
        int k, 
        double overlap_threshold_pct
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
};