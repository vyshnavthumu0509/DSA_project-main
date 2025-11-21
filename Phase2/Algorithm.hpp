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
};
class Yen{
    private:
    const Graph*graph;
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