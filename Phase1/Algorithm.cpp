#include "Algorithm.hpp"
#include <queue>
#include <climits>
#include <algorithm>
#include <unordered_map>

PathResult Algorithms:: shortestPathDistance(
        const Graph&graph,
        int src,
        int dst,
        const Constraints&constrains
    ){
        //initiate
        PathResult result;
        result.possible=false;
        result.cost=INF;
        if(constrains.forbidden_nodes.count(src)|| constrains.forbidden_nodes.count(dst) ){
            return result;
        }
        const Node* src_node=graph.getNode(src);
        const Node * dst_node=graph.getNode(dst);
        if(!src_node || !dst_node) return result;
        std::unordered_map<int,double>dist;
        std::unordered_map<int,int> parent;
        std::unordered_map<int, double> f_score;
        std::priority_queue<std::pair<double,int>,std::vector<std::pair<double,int>>,std::greater<>> pq;
         dist[src]=0.0;//push src,and make its distance 0;
         double d_start=graph.euclideanDistance(src_node->lat,src_node->lon,dst_node->lat,dst_node->lon);
        f_score[src]=d_start;
         pq.push({0.0+d_start,src});
         while(!pq.empty()){
            double d=pq.top().first;
            int u=pq.top().second;
            pq.pop();
            if(u==dst) break;
            if(f_score.count(u) && d > f_score[u]) continue;
            for(int edge_id:graph.getAdjacentEdges(u)){
                const Edge *e=graph.getEdge(edge_id);
                if(!e || e->is_removed)continue;
                if(!constrains.forbidden_road_types.empty() && constrains.forbidden_road_types.count(e->road_type))continue;
                int v=(e->u==u)?e->v:e->u;
                if(e->oneway && e->u!=u)continue;
                if(constrains.forbidden_nodes.count(v))continue;
                double new_dist=dist[u]+e->length;
                if(!dist.count(v) || new_dist<dist[v]){
                    dist[v]=new_dist;
                    parent[v]=u;
                    const Node * n_v=graph.getNode(v);
                    double h=graph.euclideanDistance(n_v->lat,n_v->lon,dst_node->lat,dst_node->lon);
                    double f=new_dist+h;
                        f_score[v]=f;
                        pq.push({f,v});
                    }

                }
        }
            if(!dist.count(dst))return result;
            if(dst!=src && !parent.count(dst))return result;
            std::vector<int> path;
            int cur=dst;
            while(cur!=src){
                if(!parent.count(cur))return result;
                path.push_back(cur);
                cur=parent[cur];
            }
            path.push_back(src);
            std::reverse(path.begin(),path.end());
            result.possible=true;
            result.cost=dist[dst];
            result.path=path;
            return result;
        }
    double Algorithms::speed(const Graph&graph){
        double m=-INFINITY;
        for(auto i:graph.edges){
            for(auto j:i.second.speed_profile){
                m=std::max(j,m);
            }
        }
        return m;
    }

PathResult Algorithms::shortestPathTime(
    const Graph& graph,
    int src,
    int dst,
    const Constraints& constrains)
{
    PathResult result;
    result.possible = false;
    result.cost = INF;

    if (constrains.forbidden_nodes.count(src) || constrains.forbidden_nodes.count(dst)) {
        return result;
    }

    const Node* src_node = graph.getNode(src);
    const Node* dst_node = graph.getNode(dst);
    if (!src_node || !dst_node) return result;

    std::unordered_map<int, double> time_map;
    std::unordered_map<int, int> parent;
    std::priority_queue<std::pair<double,int>,
                        std::vector<std::pair<double,int>>,
                        std::greater<>> pq;

    time_map[src] = 0.0;
    pq.push({0.0, src});

    while (!pq.empty()) {
        double t = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (u == dst) break;

        if (time_map.count(u) && t > time_map[u]) continue;

        for (int edge_id : graph.getAdjacentEdges(u)) {
            const Edge* e = graph.getEdge(edge_id);
            if (!e || e->is_removed) continue;

            if (!constrains.forbidden_road_types.empty() &&
                constrains.forbidden_road_types.count(e->road_type)) continue;

            int v = (e->u == u) ? e->v : e->u;
            if (e->oneway && e->u != u) continue;
            if (constrains.forbidden_nodes.count(v)) continue;

            double edge_time = graph.getEdgeTime(edge_id, time_map[u]);
            double new_time = time_map[u] + edge_time;

            if (!time_map.count(v) || new_time < time_map[v]) {
                time_map[v] = new_time;
                parent[v] = u;
                pq.push({new_time, v});
            }
        }
    }

    if (!time_map.count(dst)) return result;
    if (dst != src && !parent.count(dst)) return result;

    std::vector<int> path;
    int cur = dst;
    while (cur != src) {
        if (!parent.count(cur)) return result;
        path.push_back(cur);
        cur = parent[cur];
    }
    path.push_back(src);
    std::reverse(path.begin(), path.end());

    result.possible = true;
    result.cost = time_map[dst];
    result.path = path;
    return result;
}
int Algorithms::findNearestNode(const Graph&graph, double lat,double lon){
        int nearest=-1;
        double min_dist=INT_MAX;
        for(const Node &node:graph.getNodes()){
            double dist=graph.euclideanDistance(lat,lon,node.lat,node.lon);
            if(dist<min_dist){
                min_dist=dist;
                nearest=node.id;
            }
        }
            return nearest;

    }
std::vector<int> Algorithms::knnEuclidean(const Graph &graph,double query_lat,double query_lon,const std::string&poi_type,int k){
        std::priority_queue<std::pair<double,int>> distances;
        int cur_size=0;
        for(const Node &node:graph.getNodes()){
            for(const std::string&poi:node.pois){
                if(poi==poi_type){
                 double dist=graph.euclideanDistance(query_lat,query_lon,node.lat,node.lon);
                 if(cur_size<k){
                    distances.push({dist,node.id});
                    cur_size++;
                 }
                 else if(dist<distances.top().first){
                    distances.pop();
                    distances.push({dist,node.id});
                 }
                    break;
                }
            }
            }
            std::vector<int> result(cur_size);
            for(int i=0;i<cur_size;i++){
                result[cur_size-i-1]=distances.top().second;
                distances.pop();
            }
            return result;
    }

std::vector<int> Algorithms::knnShortestPath(const Graph &graph, double query_lat, double query_lon, const std::string &poi_type, int k) {
    std::vector<int> result;
    
    // 1. Find the starting node on the road network
    int start_node = findNearestNode(graph, query_lat, query_lon);
    if (start_node == -1) return result;

    // 2. Standard Dijkstra Initialization
    // Min-priority queue: {distance, node_id}
    std::priority_queue<std::pair<double, int>, 
                        std::vector<std::pair<double, int>>, 
                        std::greater<std::pair<double, int>>> pq;
    
    std::unordered_map<int, double> dist;
    
    dist[start_node] = 0.0;
    pq.push({0.0, start_node});

    // 3. Search Loop
    while (!pq.empty()) {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        // Stale node check
        if (dist.count(u) && d > dist[u]) continue;

        // --- CHECK FOR POI ---
        // Since Dijkstra expands in strict order of distance, 
        // the first 'k' POIs we encounter are guaranteed to be the nearest.
        const Node* u_node = graph.getNode(u);
        if (u_node) {
            bool found_poi = false;
            for (const auto& poi : u_node->pois) {
                if (poi == poi_type) {
                    found_poi = true;
                    break;
                }
            }
            if (found_poi) {
                result.push_back(u);
                // OPTIMIZATION: Early exit once we have k results
                if (result.size() == (size_t)k) return result;
            }
        }

        // --- EXPAND NEIGHBORS ---
        for (int edge_id : graph.getAdjacentEdges(u)) {
            const Edge* e = graph.getEdge(edge_id);
            
            // 1. Check if edge exists and is not removed
            if (!e || e->is_removed) continue;

            // 2. Check dynamic one-way constraint
            // If modifyEdge changed oneway status, adj_list might still have the back-link.
            // We must ensure we are at the 'u' side if it is one-way.
            if (e->oneway && e->u != u) continue;

            int v = (e->u == u) ? e->v : e->u;
            double new_dist = d + e->length;

            if (dist.find(v) == dist.end() || new_dist < dist[v]) {
                dist[v] = new_dist;
                pq.push({new_dist, v});
            }
        }
    }

    return result;
}