#include "Algorithm.hpp"
#include "Graph.hpp"
#include <climits>
#include <queue>
#include <algorithm>
#include <set>
#include <map>


using State = std::pair<double, int>; 

Yen::Yen(const Graph*g):graph(g){
    buildReverseGraph();
}
void Yen::buildReverseGraph() {
    reverse_adj_list.clear();
    const auto& nodes = graph->getNodes();

    for (const auto& node : nodes) {
        // Iterate through all outgoing edges of every node
        const auto& outgoing_edges = graph->getAdjacentEdges(node.id);
        for (int edge_id : outgoing_edges) {
            const Edge* e = graph->getEdge(edge_id);
            if (!e || e->is_removed) continue;
            int u = node.id;
            int v = -1;
            if (e->oneway) {
                // If oneway, strictly u -> v
                if (e->u == u) v = e->v;
                // If e->u != u (impossible if getAdjacentEdges is correct), skip
            } else {
                // Two-way: if we are at u, v is the other end
                v = (e->u == u) ? e->v : e->u;
            }
            if (v != -1) {
                reverse_adj_list[v].push_back(edge_id);
            }
        }
    }
}
void Yen::computeHeuristics(int target){
    heuristic_map.clear();
    std::priority_queue<std::pair<double,int>,std::vector<std::pair<double,int>>,std::greater<std::pair<double,int>>> pq;
    heuristic_map[target]=0.0;
    pq.push({0.0,target});
    while (!pq.empty()) {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d > heuristic_map[u]) continue;

        // Expand backwards using reverse_adj_list
        // These are edges that enter u in the forward graph
        if (reverse_adj_list.find(u) != reverse_adj_list.end()) {
            for (int edge_id : reverse_adj_list[u]) {
                const Edge* e = graph->getEdge(edge_id);
                if (!e || e->is_removed) continue;

                // In forward graph: src -> u with weight w
                // In reverse traversal: u -> src with weight w
                int src = (e->u == u) ? e->v : e->u; // Get the other end

                double new_dist = d + e->length;

                if (heuristic_map.find(src) == heuristic_map.end() || new_dist < heuristic_map[src]) {
                    heuristic_map[src] = new_dist;
                    pq.push({new_dist, src});
                }
            }
        }
    }
}
path Yen::a_star(int start,int target,const std::set<int>&banned_edges,const std::set<int>&banned_nodes){
    path result;
    if (banned_nodes.count(start) || banned_nodes.count(target))return result;
    std::unordered_map<int,double>dist;
    std::unordered_map<int,int> parent;
    std::unordered_map<int,int> parent_edge;
    std::priority_queue<std::pair<double,int>,std::vector<std::pair<double,int>>,std::greater<std::pair<double,int>>> pq;
    dist[start]=0.0;
    pq.push({0.0,start});
    while(!pq.empty()){
        double d=pq.top().first;
        int u=pq.top().second;
        pq.pop();
        if(dist.count(u)&& d>dist[u] )continue;
        if(u==target) break;
        for(int id:graph->getAdjacentEdges(u)){
            if(banned_edges.count(id))continue;
            const Edge*edge=graph->getEdge(id);
            if(!edge || edge->is_removed) continue;
            int v=(edge->u==u)?edge->v:edge->u;
            if(edge->oneway && edge->u!=u)continue;
            if(banned_nodes.count(v))continue;
            double new_dist=d+edge->length;
            if(!dist.count(v)|| new_dist<dist[v]){
                dist[v]=new_dist;
                parent[v]=u;
                parent_edge[v]=id;
                pq.push({new_dist,v});
            }
        }
    }
    if(!dist.count(target))return result;
    result.cost=dist[target];
    int current=target;
    while(current!=start){
        result.nodes.push_back(current);
        if(parent_edge.count(current)){
            result.edges.push_back(parent_edge[current]);
        }
        current=parent[current];
    }
    result.nodes.push_back(start);
    std::reverse(result.nodes.begin(),result.nodes.end());
    std::reverse(result.edges.begin(),result.edges.end());
    return result;
}
std::vector<path> Yen::findK_paths(int start,int target,int k){
    std::vector<path>result;
    std::vector<path> cand;
    computeHeuristics(target);
    std::priority_queue<path,std::vector<path>,std::greater<path>> pq;
    path shortest_path=a_star(start,target,{},{});
    if(shortest_path.nodes.empty())return result;
    result.push_back(shortest_path);
    for(int i=0;i<k;i++){
        path & prev_path=result.back();
        for(size_t j=0;j<prev_path.nodes.size()-1;j++){
            int spur_node=prev_path.nodes[j];
            std::vector<int> root_path(prev_path.nodes.begin(),prev_path.nodes.begin()+i+1);
            std::vector<int> root_edges(prev_path.edges.begin(),prev_path.edges.begin()+i);
            double root_length=0.0;
            for(int edge_id:root_edges){
                const Edge*e=graph->getEdge(edge_id);
                if(e)root_length+=e->length;
            }
            std::set<int> banned_edges;
            std::set<int> banned_nodes;
            //ban rule1
            for(const auto&p:result){
                if (p.nodes.size() > i) {
                    bool same_root = true;
                    for (size_t k = 0; k <= i; ++k) {
                        if (p.nodes[k] != root_path[k]) {
                            same_root = false;
                            break;
                        }
                    }                
                    if(same_root) {
                        if(i<p.edges.size()){
                            banned_edges.insert(p.edges[i]);
                        }
                    }
                }
            }

            //ban2
            for(size_t j=0;j<i;++j){
                banned_nodes.insert(root_path[j]);
            }
            path spur_path=a_star(spur_node,target,banned_edges,banned_nodes);
            if(spur_path.nodes.size()!=0){
                path total_path;
                total_path.nodes=root_path;
                total_path.nodes.insert(total_path.nodes.end(),spur_path.nodes.begin()+1,spur_path.nodes.end());
                 
                total_path.edges=root_edges;
                total_path.edges.insert(total_path.edges.end(),spur_path.edges.begin(),spur_path.edges.end());
                total_path.cost=root_length+spur_path.cost;
                bool is_duplicate = false;
                for (const auto& candidate : cand) {
                    if (candidate==total_path) {
                        is_duplicate = true;
                        break;
                    }
                }
                if (!is_duplicate) {
                    cand.push_back(total_path);
                }
            }
        }
        if(cand.empty())break;
        std::sort(cand.begin(),cand.end());
        result.push_back(cand[0]);
        cand.erase(cand.begin());

    }
    return result;
}

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