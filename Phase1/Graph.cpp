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

double Graph::getEdgeTime(int edge_id, double start_time) const { // Fixed typo 'strat_time'
    const Edge* e = getEdge(edge_id);
    if (!e) return INF;

    // FIX 1: Only fallback if profile is completely empty. 
    // Do not enforce size() == 96, as this breaks simplified test cases.
    if (e->speed_profile.empty()) {
        return e->average_time;
    }

    double total_time = 0.0;
    double remaining_distance = e->length;
    double current_time = start_time;
    size_t profile_size = e->speed_profile.size(); // Get actual size

    while (remaining_distance >= 1e-6) {
        // FIX 2: Use profile_size for modulo to prevent segfaults on small vectors
        int slot = static_cast<int>(current_time / 900.0) % profile_size;
        
        double speed = e->speed_profile[slot];

        // Fallback for bad data (0 speed)
        if (speed < 1e-6) {
             if (e->average_time > 1e-6) speed = e->length / e->average_time;
             else return INF; // Road is blocked (0 speed, 0 avg time)
        }

        // Time remaining until the end of the current 900-second slot
        double time_to_slot_end = 900.0 - fmod(current_time, 900.0);
        
        // Distance we can cover in the remainder of this slot
        double distance_to_slot_end = speed * time_to_slot_end;

        if (distance_to_slot_end >= remaining_distance) {
            // Case: Edge is completed within this slot
            double time_needed = remaining_distance / speed;
            total_time += time_needed;
            return total_time; 
        } else {
           // [cite_start]// Case: We reach the end of the slot and must continue in the next one [cite: 183]
            total_time += time_to_slot_end;
            remaining_distance -= distance_to_slot_end;
            current_time += time_to_slot_end;
        }
    }
    
    return total_time;
}
double Graph::euclideanDistance(double lat1, double lon1, double lat2, double lon2) const{
    double dx = (lon2 - lon1) * 111320.0 * cos((lat1+lat2)/2.0*M_PI/180.0);
    double dy = (lat2 - lat1) * 110540.0;
    return sqrt(dx*dx+dy*dy);
}
