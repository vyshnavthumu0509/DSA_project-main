#include "Graph.hpp"

void Graph::addNode(const Node& node){
    node_id_to_index[node.id] = nodes.size();
    nodes.push_back(node);
    adj_list[node.id] = std::vector<int>();
}

void Graph::addEdge(const Edge& edge){
    edges[edge.id]=edge;
    adj_list[edge.u].push_back(edge.id);
    if(!edge.oneway){
        adj_list[edge.v].push_back(edge.id);
    }
}

void Graph::removeEdge(int edge_id){
    if(edges.find(edge_id)!=edges.end()){
        edges[edge_id].is_removed = true;
    }
}

void Graph::modifyEdge(int edge_id, const Edge&patch){
    if(edges.find(edge_id)!=edges.end()){
        Edge&e=edges[edge_id];

        if(e.is_removed) e.is_removed=false;

        if(patch.length > 0) e.length = patch.length;
        if(patch.average_time>0) e.average_time = patch.average_time;
        if(!patch.speed_profile.empty()) e.speed_profile = patch.speed_profile;
        if(!patch.road_type.empty()) e.road_type = patch.road_type;
    }
}

const Node* Graph::getNode(int node_id) const{
    auto idx = node_id_to_index.find(node_id);
    if(idx != node_id_to_index.end()){
        return & nodes[idx->second];
    }
    return nullptr;
}

const Edge* Graph::getEdge(int edge_id) const {
    auto edg= edges.find(edge_id);
    if(edg!=edges.end()&&!edg->second.is_removed){
        return &edg->second;
    }
    return nullptr;
}

const std::vector<int> & Graph::getAdjacentEdges(int node_id) const{
    static const std::vector<int> empty;
    auto ad = adj_list.find(node_id);
    if(ad!=adj_list.end()){
        return ad->second;
    }
    return empty;
}

double Graph::getEdgeTime(int edge_id, double strat_time) const{
    const Edge* e=getEdge(edge_id);
    if(!e) return INF;

    if(e->speed_profile.empty()||e->speed_profile.size()!=96){
        return e->average_time;
    }

    double total_time = 0.0;
    double remaining_distance = e->length;
    double current_time = strat_time;
    
    int safety_loop_counter = 0;
    const int MAX_LOOPS = 10000; 

    while(remaining_distance >= 1e-6 && safety_loop_counter < MAX_LOOPS){
        int slot = static_cast<int>(current_time / 900.0) % 96;
        double speed = e->speed_profile[slot];

        if(speed < 1e-6) {
    
             speed = e->length / e->average_time;
        }
        if (speed < 1e-6) return INF; 

        double time_to_slot_end = 900.0 - fmod(current_time, 900.0);
        double distance_to_slot_end = speed * time_to_slot_end;

        if(distance_to_slot_end >= remaining_distance){
            double time_needed = remaining_distance / speed;
            total_time += time_needed;
            return total_time; 
        } else {
            total_time += time_to_slot_end;
            remaining_distance -= distance_to_slot_end;
            current_time += time_to_slot_end;
        }
        safety_loop_counter++;
    }
    
    if (safety_loop_counter >= MAX_LOOPS) return INF;
    return total_time;
}

double Graph::euclideanDistance(double lat1, double lon1, double lat2, double lon2) const{
    double dx = (lon2 - lon1) * 111320.0 * cos((lat1+lat2)/2.0*M_PI/180.0);
    double dy = (lat2 - lat1) * 110540.0;
    return sqrt(dx*dx+dy*dy);
}