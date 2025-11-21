#include "Algorithm.hpp"
#include "Graph.hpp"
#include <queue>
#include <algorithm>
#include <set>
#include <map>


using State = std::pair<double, int>; 

std::vector<path> Heuristic::kShortestPathsHeuristic(
    const Graph& graph, 
    int source_id, 
    int target_id, 
    int k, 
    double overlap_threshold_pct
){
    std::vector<path> results;
    std::unordered_map<int, double> penality_map;
    std::set<std::vector<int>> seen_node_sequences;

    const double INITIAL_PENALTY = 1.5;
    const double PENALTY_GROWTH = 1.2;
    const double MAX_ITERATIONS = k*5;

    for(int i=0; i<MAX_ITERATIONS && results.size()<k; i++){
        path current_path;

        bool found = getShortestPathWithPenalty(
            graph, 
            source_id, 
            target_id, 
            penality_map, 
            current_path
        );

        if(!found) break;

        if(seen_node_sequences.find(current_path.nodes) == seen_node_sequences.end()){
            results.push_back(current_path);
            seen_node_sequences.insert(current_path.nodes);
        }

        // Update penalties for edges in the next iteration
        for(int edge_id : current_path.edges){
            if(penality_map.find(edge_id) == penality_map.end()){
                penality_map[edge_id] = INITIAL_PENALTY;
            } else {
                penality_map[edge_id] *= PENALTY_GROWTH;
            }
        }
    }

    std::sort(results.begin(), results.end(), [](const path& a, const path& b){
        return a.cost < b.cost;
    });

    if(results.size() > k){
        results.resize(k);
    }
    return results;
}

bool Heuristic::getShortestPathWithPenalty(
    const Graph& graph,
    int source_id,
    int target_id,
    const std::unordered_map<int, double>& edge_penalties,
    path& result
){
   const Node* source_node = graph.getNode(source_id);
   const Node* target_node = graph.getNode(target_id);
   if(!source_node || !target_node) return false;

   std::unordered_map<int, double> costn;
   std::unordered_map<int, int> parent_node;
   std::unordered_map<int, int> parent_edge;

   std::priority_queue<State, std::vector<State>, std::greater<State>> pq;

   costn[source_id] = 0.0;

   const Node* startNode = source_node;
   double start_h = graph.euclideanDistance(
       startNode->lat, startNode->lon,
       target_node->lat, target_node->lon
   );

   pq.push({start_h, source_id});
   parent_node[source_id] = -1;

   while(!pq.empty()){
        double current_f = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if(u == target_id){
            break;
        }

        const auto & adj = graph.getAdjacentEdges(u);
        const Node* current_node = graph.getNode(u);

        for(int edge_id: adj){
            const Edge* e = graph.getEdge(edge_id);
            if (!e || e->is_removed) continue;

            int v = (e->u == u) ? e->v : e->u;

            double weight = e->length;
            auto it = edge_penalties.find(edge_id);
            if (it != edge_penalties.end()) {
                weight *= it->second;
            }

            double new_cost = costn[u] + weight;

            if(costn.find(v) == costn.end() || new_cost < costn[v]){
                costn[v] = new_cost;
                parent_node[v] = u;
                parent_edge[v] = edge_id;

                const Node* neighbor_node = graph.getNode(v);
                double h = 0.0;
                if(neighbor_node){
                    h = graph.euclideanDistance(
                        neighbor_node->lat, neighbor_node->lon,
                        target_node->lat, target_node->lon
                    );
                }
                pq.push({new_cost + h, v});
                
            }
        }
    }

    if(costn.find(target_id) == costn.end()){
        return false;
    }

    result.cost = 0;
    result.edges.clear();
    result.nodes.clear();

    int curr = target_id;
    while(curr != -1){
        result.nodes.push_back(curr);
        if(curr == source_id) break;

        int p_edge = parent_edge[curr];
        int p_node = parent_node[curr];

        result.edges.push_back(p_edge);

        const Edge* e = graph.getEdge(p_edge);
        if(e)
            result.cost += e->length;
        curr = p_node;
    }

    std::reverse(result.nodes.begin(), result.nodes.end());
    std::reverse(result.edges.begin(), result.edges.end());

    return true;
}