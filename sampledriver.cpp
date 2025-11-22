#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <vector>

#include "Graph.hpp"
#include "Algorithm.hpp"

using json = nlohmann::ordered_json; // Preserves insertion order
// --- Helper function to handle individual queries ---
json process_query(Graph& graph, const json& query) {
    if (!query.is_object()) return json::object();

    std::string type;
    if (query.contains("type")) {
        type = query["type"];
    } else {
        return json::object();
    }

    json response;

    if (type == "shortest_path") {
        int id = query["id"];
        int source = query["source"];
        int target = query["target"];
        std::string mode = query["mode"];
        
        Constraints constraints;
        if (query.contains("constraints")) {
            const json& c = query["constraints"];
            if (c.contains("forbidden_nodes")) {
                for (const auto& node : c["forbidden_nodes"]) constraints.forbidden_nodes.insert(node.get<int>());
            }
            if (c.contains("forbidden_road_types")) {
                for (const auto& rt : c["forbidden_road_types"]) constraints.forbidden_road_types.insert(rt);
            }
        }
        
        PathResult result;
        if (mode == "distance") {
            result = Algorithms::shortestPathDistance(graph, source, target, constraints);
        } else if (mode == "time") {
            result = Algorithms::shortestPathTime(graph, source, target, constraints);
        }
        
        response["id"] = id;
        response["possible"] = result.possible;
        if (result.possible) {
            if (mode == "distance") response["minimum_distance"] = result.cost;
            else response["minimum_time"] = result.cost;
            response["path"] = result.path;
        }

    } else if (type == "knn") {
        int id = query["id"];
        std::string poi = query["poi"];
        double lat = query["query_point"]["lat"];
        double lon = query["query_point"]["lon"];
        int k = query["k"];
        std::string metric = query["metric"];
        
        std::vector<int> nodes;
        if (metric == "euclidean") {
            nodes = Algorithms::knnEuclidean(graph, lat, lon, poi, k);
        } else if (metric == "shortest_path") {
            nodes = Algorithms::knnShortestPath(graph, lat, lon, poi, k);
        }
        
        response["id"] = id;
        response["nodes"] = nodes;

    } else if (type == "remove_edge") {
        // UPDATE: Include ID in response [cite: 216]
        if (query.contains("id")) response["id"] = query["id"];
        
        int edge_id = query["edge_id"];
        graph.removeEdge(edge_id);
        response["done"] = true;

    } else if (type == "modify_edge") {
        // UPDATE: Include ID in response [cite: 233]
        if (query.contains("id")) response["id"] = query["id"];

        int edge_id = query["edge_id"];
        Edge patch;
        if (query.contains("patch")) {
            const json& p = query["patch"];
            if (p.contains("length")) patch.length = p["length"];
            if (p.contains("average_time")) patch.average_time = p["average_time"];
            if (p.contains("road_type")) patch.road_type = p["road_type"];
            if (p.contains("speed_profile")) {
                for (const auto& s : p["speed_profile"]) patch.speed_profile.push_back(s);
            }
        }
        graph.modifyEdge(edge_id, patch);
        response["done"] = true;
    }

    return response;
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << std::endl;
        return 1;
    }

    std::string graph_path = argv[1];
    std::string queries_path = argv[2];
    std::string output_path = argv[3];

    // 1. Read graph
    std::ifstream graph_file(graph_path);
    if (!graph_file.is_open()) {
        std::cerr << "Failed to open graph file: " << graph_path << std::endl;
        return 1;
    }
    json graph_json;
    graph_file >> graph_json;
    graph_file.close();

    Graph graph;
    if (graph_json.contains("nodes")) {
        for (const auto& node_json : graph_json["nodes"]) {
            Node node;
            node.id = node_json["id"];
            node.lat = node_json["lat"];
            node.lon = node_json["lon"];
            if (node_json.contains("pois")) {
                for (const auto& poi : node_json["pois"]) node.pois.push_back(poi);
            }
            graph.addNode(node);
        }
    }
    if (graph_json.contains("edges")) {
        for (const auto& edge_json : graph_json["edges"]) {
            Edge edge;
            edge.id = edge_json["id"];
            edge.u = edge_json["u"];
            edge.v = edge_json["v"];
            edge.length = edge_json["length"];
            edge.average_time = edge_json["average_time"];
            edge.oneway = edge_json.value("oneway", false);
            edge.road_type = edge_json.value("road_type", "");
            edge.is_removed = false;
            if (edge_json.contains("speed_profile")) {
                for (const auto& speed : edge_json["speed_profile"]) edge.speed_profile.push_back(speed);
            }
            graph.addEdge(edge);
        }
    }

    // 2. Read queries
    std::ifstream queries_file(queries_path);
    if (!queries_file.is_open()) {
        std::cerr << "Failed to open queries file: " << queries_path << std::endl;
        return 1;
    }
    json queries_json;
    queries_file >> queries_json;
    queries_file.close();

    // 3. Prepare Output Structure
    std::ofstream output_file(output_path);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open output file: " << output_path << std::endl;
        return 1;
    }

    // UPDATE: Create the root JSON object for output
    json final_output;
    // Copy meta from input to output [cite: 204]
    if (queries_json.contains("meta")) {
        final_output["meta"] = queries_json["meta"];
    }
    json results_array = json::array();

    // 4. Determine Query List
    const json* queries_list = nullptr;
    if (queries_json.contains("events")) queries_list = &queries_json["events"];
    else if (queries_json.contains("queries")) queries_list = &queries_json["queries"];
    else if (queries_json.is_array()) queries_list = &queries_json;

    if (queries_list == nullptr) {
        std::cerr << "Error: Could not find valid query array." << std::endl;
        return 1;
    }

    // 5. Process
    for (const auto& query : *queries_list) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // UPDATE: Wrap in try-catch to prevent crashes [cite: 423]
        try {
            json result = process_query(graph, query);
            
            if (!result.empty()) {
                auto end_time = std::chrono::high_resolution_clock::now();
                result["processing_time"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();
                
                // UPDATE: Append to results array instead of writing directly
                results_array.push_back(result);
            }
        } catch (const std::exception& e) {
            std::cerr << "Error processing query: " << e.what() << std::endl;
            // Optionally add an error entry to results if desired, or skip
        }
    }

    // UPDATE: Write the final single JSON object [cite: 196, 200]
    final_output["results"] = results_array;
    output_file << final_output.dump(4); // Pretty print with indentation 4

    output_file.close();
    return 0;
}