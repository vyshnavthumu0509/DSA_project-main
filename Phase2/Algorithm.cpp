#include "Algorithm.hpp"
#include "Graph.hpp"
#include <climits>
#include <queue>
#include <algorithm>
#include <set>
#include <map>
#include <random>
#include <cmath>

// Static Member Definitions for Task 3
std::vector<std::vector<double>> Algorithm::landmarks_to_nodes;
std::vector<std::vector<double>> Algorithm::nodes_to_landmarks;
std::vector<int> Algorithm::landmark_indices;
bool Algorithm::is_precomputed = false;

// Local static map for O(1) ID lookups
static std::unordered_map<int, int> id_map;

using State = std::pair<double, int>; 

// ==========================================
//      TASK 1: Yen's Algorithm
// ==========================================

void Yen::buildReverseGraph() {
    reverse_adj_list.clear();
    const auto& nodes = graph->getNodes();

    for (const auto& node : nodes) {
        const auto& outgoing_edges = graph->getAdjacentEdges(node.id);
        for (int edge_id : outgoing_edges) {
            const Edge* e = graph->getEdge(edge_id);
            if (!e || e->is_removed) continue;
            
            int v = -1;
            if (e->oneway) {
                if (e->u == node.id) v = e->v;
            } else {
                v = (e->u == node.id) ? e->v : e->u;
            }
            
            if (v != -1) {
                reverse_adj_list[v].push_back(edge_id);
            }
        }
    }
}

void Yen::computeHeuristics(int target) {
    heuristic_map.clear();
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
    heuristic_map[target] = 0.0;
    pq.push({0.0, target});

    while (!pq.empty()) {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d > heuristic_map[u]) continue;

        if (reverse_adj_list.find(u) != reverse_adj_list.end()) {
            for (int edge_id : reverse_adj_list[u]) {
                const Edge* e = graph->getEdge(edge_id);
                if (!e || e->is_removed) continue;

                int src = (e->u == u) ? e->v : e->u; 
                double new_dist = d + e->length;

                if (heuristic_map.find(src) == heuristic_map.end() || new_dist < heuristic_map[src]) {
                    heuristic_map[src] = new_dist;
                    pq.push({new_dist, src});
                }
            }
        }
    }
}

path Yen::a_star(int start, int target, const std::set<int>& banned_edges, const std::set<int>& banned_nodes) {
    path result;
    if (banned_nodes.count(start) || banned_nodes.count(target)) return result;
    
    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> parent;
    std::unordered_map<int, int> parent_edge;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
    
    double start_h = (heuristic_map.count(start)) ? heuristic_map[start] : 0.0;

    dist[start] = 0.0;
    pq.push({start_h, start});
    
    while (!pq.empty()) {
        double f = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        // Lazy deletion check
        if (dist.count(u) && f > dist[u] + ((heuristic_map.count(u)) ? heuristic_map[u] : 0.0) + 1e-9) continue;
        if (u == target) break;
        
        for (int id : graph->getAdjacentEdges(u)) {
            if (banned_edges.count(id)) continue;
            const Edge* edge = graph->getEdge(id);
            if (!edge || edge->is_removed) continue;
            
            int v = (edge->u == u) ? edge->v : edge->u;
            if (edge->oneway && edge->u != u) continue;
            if (banned_nodes.count(v)) continue;
            
            double new_dist = dist[u] + edge->length;
            if (!dist.count(v) || new_dist < dist[v]) {
                dist[v] = new_dist;
                parent[v] = u;
                parent_edge[v] = id;
                double h = (heuristic_map.count(v)) ? heuristic_map[v] : 0.0;
                pq.push({new_dist + h, v});
            }
        }
    }
    
    if (!dist.count(target)) return result;
    
    result.cost = dist[target];
    int current = target;
    while (current != start) {
        result.nodes.push_back(current);
        if (parent_edge.count(current)) {
            result.edges.push_back(parent_edge[current]);
        }
        current = parent[current];
    }
    result.nodes.push_back(start);
    std::reverse(result.nodes.begin(), result.nodes.end());
    std::reverse(result.edges.begin(), result.edges.end());
    return result;
}

std::vector<path> Yen::findK_paths(int start, int target, int k) {
    std::vector<path> result;
    std::vector<path> cand;
    
    computeHeuristics(target);
    
    path shortest_path = a_star(start, target, {}, {});
    if (shortest_path.nodes.empty()) return result;
    
    result.push_back(shortest_path);
    
    for (int i = 1; i < k; i++) {
        path& prev_path = result.back();
        
        for (size_t j = 0; j < prev_path.nodes.size() - 1; j++) {
            int spur_node = prev_path.nodes[j];
            
            std::vector<int> root_path(prev_path.nodes.begin(), prev_path.nodes.begin() + j + 1);
            std::vector<int> root_edges;
            if (j > 0 && j <= prev_path.edges.size()) {
                 root_edges.assign(prev_path.edges.begin(), prev_path.edges.begin() + j);
            }

            double root_length = 0.0;
            for (int edge_id : root_edges) {
                const Edge* e = graph->getEdge(edge_id);
                if (e) root_length += e->length;
            }
            
            std::set<int> banned_edges;
            std::set<int> banned_nodes;
            
            for (const auto& p : result) {
                if (p.nodes.size() > j) {
                    bool same_root = true;
                    for (size_t n = 0; n <= j; ++n) {
                        if (p.nodes[n] != root_path[n]) {
                            same_root = false;
                            break;
                        }
                    }
                    if (same_root && j < p.edges.size()) {
                        banned_edges.insert(p.edges[j]);
                    }
                }
            }
            
            for (size_t n = 0; n < j; ++n) {
                banned_nodes.insert(root_path[n]);
            }
            
            path spur_path = a_star(spur_node, target, banned_edges, banned_nodes);
            
            if (!spur_path.nodes.empty()) {
                path total_path;
                total_path.nodes = root_path;
                if (spur_path.nodes.size() > 1) {
                    total_path.nodes.insert(total_path.nodes.end(), spur_path.nodes.begin() + 1, spur_path.nodes.end());
                }
                
                total_path.edges = root_edges;
                total_path.edges.insert(total_path.edges.end(), spur_path.edges.begin(), spur_path.edges.end());
                
                total_path.cost = root_length + spur_path.cost;
                
                bool is_duplicate = false;
                for (const auto& candidate : cand) {
                    if (candidate == total_path) {
                        is_duplicate = true;
                        break;
                    }
                }
                if (!is_duplicate) {
                    cand.push_back(total_path);
                }
            }
        }
        
        if (cand.empty()) break;
        std::sort(cand.begin(), cand.end());
        result.push_back(cand[0]);
        cand.erase(cand.begin());
    }
    return result;
}

// ==========================================
//      TASK 2: Heuristic K-Shortest Paths
// ==========================================

std::vector<path> Algorithm::kShortestPathsHeuristic(
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
    const double MAX_ITERATIONS = k * 5;

    for(int i = 0; i < MAX_ITERATIONS && results.size() < k; i++){
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

        // Update penalties
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

    if(results.size() > (size_t)k){
        results.resize(k);
    }
    return results;
}

bool Algorithm::getShortestPathWithPenalty(
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

   // Min-Priority Queue: {cost, node_id}
   std::priority_queue<State, std::vector<State>, std::greater<State>> pq;

   costn[source_id] = 0.0;
   
   // FIX: Heuristic removed (set to 0.0) to prevent scale mismatch with abstract test weights.
   // This effectively makes it Dijkstra, which ensures optimality and correctness on all graph types.
   pq.push({0.0, source_id});
   parent_node[source_id] = -1;

   while(!pq.empty()){
        double current_cost = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        // FIX: Stale node check added for performance and correctness
        if (costn.find(u) != costn.end() && current_cost > costn[u]) continue;

        if(u == target_id) break;
        
        for(int edge_id : graph.getAdjacentEdges(u)){
            const Edge* e = graph.getEdge(edge_id);
            if (!e || e->is_removed) continue;

            int v = (e->u == u) ? e->v : e->u;
            if(e->oneway && e->u != u) continue;

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
                
                // Push with new_cost (pure Dijkstra)
                pq.push({new_cost, v});
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
        if(e) result.cost += e->length; // Use REAL length, not penalized
        curr = p_node;
    }

    std::reverse(result.nodes.begin(), result.nodes.end());
    std::reverse(result.edges.begin(), result.edges.end());

    return true;
}

// ==========================================
//      TASK 3: Approximate Shortest Path
// ==========================================

void Algorithm::precomputeLandmarks(const Graph& graph) {
    if (is_precomputed) return;
    
    const auto& nodes = graph.getNodes();
    int N = nodes.size();
    
    id_map.clear();
    id_map.reserve(N);
    for(int i = 0; i < N; ++i) {
        id_map[nodes[i].id] = i;
    }

    int num_landmarks = static_cast<int>(std::sqrt(N));
    if(num_landmarks < 20) num_landmarks = 20;
    if(num_landmarks > 60) num_landmarks = 60;
    if(num_landmarks > N) num_landmarks = N;

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(0, N - 1);

    landmark_indices.clear();
    std::set<int> unique_check;
    while (landmark_indices.size() < (size_t)num_landmarks) {
        int idx = dist(rng);
        if (unique_check.find(idx) == unique_check.end()) {
            landmark_indices.push_back(idx);
            unique_check.insert(idx);
        }
    }

    landmarks_to_nodes.assign(num_landmarks, std::vector<double>(N, INF));
    nodes_to_landmarks.assign(N, std::vector<double>(num_landmarks, INF));

    std::vector<std::vector<std::pair<int, double>>> rev_adj(N);
    for (const auto& u_node : nodes) {
        int u_idx = id_map[u_node.id];
        for (int edge_id : graph.getAdjacentEdges(u_node.id)) {
            const Edge* e = graph.getEdge(edge_id);
            if(!e || e->is_removed) continue;
            
            int my_id = u_node.id;
            int other_id = (e->u == my_id) ? e->v : e->u;
            
            if(id_map.find(other_id) == id_map.end()) continue;
            int other_idx = id_map[other_id];
            
            bool can_traverse = true;
            if(e->oneway && e->u != my_id) can_traverse = false;
            
            if(can_traverse) {
                rev_adj[other_idx].push_back({u_idx, e->length});
            }
        }
    }

    for (int k = 0; k < num_landmarks; ++k) {
        int start_idx = landmark_indices[k];

        // Forward
        {
            std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
            landmarks_to_nodes[k][start_idx] = 0.0;
            pq.push({0.0, start_idx});
            
            while(!pq.empty()) {
                double d = pq.top().first;
                int u_idx = pq.top().second;
                pq.pop();

                if (d > landmarks_to_nodes[k][u_idx]) continue;

                int u_id = nodes[u_idx].id;
                for(int edge_id : graph.getAdjacentEdges(u_id)) {
                    const Edge* e = graph.getEdge(edge_id);
                    if(!e || e->is_removed) continue;
                    
                    int v_id = (e->u == u_id) ? e->v : e->u;
                    if(e->oneway && e->u != u_id) continue;
                    
                    if(id_map.find(v_id) == id_map.end()) continue;
                    int v_idx = id_map[v_id];
                    
                    double new_dist = d + e->length;
                    if(new_dist < landmarks_to_nodes[k][v_idx]) {
                        landmarks_to_nodes[k][v_idx] = new_dist;
                        pq.push({new_dist, v_idx});
                    }
                }
            }
        }

        // Backward
        {
            std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
            nodes_to_landmarks[start_idx][k] = 0.0;
            pq.push({0.0, start_idx});

            while(!pq.empty()) {
                double d = pq.top().first;
                int u_idx = pq.top().second;
                pq.pop();

                if (d > nodes_to_landmarks[u_idx][k]) continue;

                for(auto& edge : rev_adj[u_idx]) {
                    int v_idx = edge.first;
                    double weight = edge.second;
                    double new_dist = d + weight;
                    if(new_dist < nodes_to_landmarks[v_idx][k]) {
                        nodes_to_landmarks[v_idx][k] = new_dist;
                        pq.push({new_dist, v_idx});
                    }
                }
            }
        }
    }
    is_precomputed = true;
}

double Algorithm::approximateShortestPathDistance(const Graph& graph, int source_id, int target_id) {
    if(!is_precomputed) return -1.0;
    
    if(id_map.find(source_id) == id_map.end() || id_map.find(target_id) == id_map.end()) {
        return INF;
    }

    int u_idx = id_map[source_id];
    int v_idx = id_map[target_id];

    double min_dist = INF;

    int K = landmark_indices.size();
    for(int k = 0; k < K; ++k) {
        double d1 = nodes_to_landmarks[u_idx][k];
        double d2 = landmarks_to_nodes[k][v_idx];
        
        if(d1 != INF && d2 != INF) {
            double total = d1 + d2;
            if(total < min_dist) {
                min_dist = total;
            }
        }
    }

    const Node* n1 = graph.getNode(source_id);
    const Node* n2 = graph.getNode(target_id);
    if(n1 && n2) {
        double eucl = graph.euclideanDistance(n1->lat, n1->lon, n2->lat, n2->lon);
        double estimated = eucl * 1.35;
        if(estimated < min_dist) {
            min_dist = estimated;
        }
    }

    return min_dist;
}