#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <string>
#include <nlohmann/json.hpp>
#include "Graph.hpp"
#include "Algorithm.hpp"

using json = nlohmann::ordered_json; // Preserves insertion order

json process_query(const Graph& graph, const json& query) {
    json result;
    
    // Ensure query has a type
    if (!query.contains("type")) return {};
    std::string type = query["type"];

    if (query.contains("id")) {
        result["id"] = query["id"];
    }

    if (type == "k_shortest_paths") {
        int source = query["source"];
        int target = query["target"];
        int k = query["k"];

        Yen yenAlgo(&graph);
        
        // Execute algorithm
        std::vector<path> paths = yenAlgo.findK_paths(source, target, k);

        // Format Output
        std::vector<json> path_results;
        for (const auto& p : paths) {
            json p_json;
            p_json["path"] = p.nodes;
            p_json["length"] = p.cost;
            path_results.push_back(p_json);
        }
        result["paths"] = path_results;
    }
    else if (type == "k_shortest_paths_heuristic") {
        int source = query["source"];
        int target = query["target"];
        int k = query["k"];
        double overlap_threshold = query.value("overlap_threshold", 0.0);

        // Execute algorithm
        std::vector<path> paths = Algorithm::kShortestPathsHeuristic(
            graph, source, target, k, overlap_threshold
        );

        // Format Output
        std::vector<json> path_results;
        for (const auto& p : paths) {
            json p_json;
            p_json["path"] = p.nodes;
            p_json["length"] = p.cost;
            path_results.push_back(p_json);
        }
        result["paths"] = path_results;
    }
    else if (type == "approx_shortest_path") {
        std::vector<json> distances;
        
        if (query.contains("queries")) {
            for (const auto& sub_q : query["queries"]) {
                int s = sub_q["source"];
                int t = sub_q["target"];

                double dist = Algorithm::approximateShortestPathDistance(graph, s, t);

                json d_entry;
                d_entry["source"] = s;
                d_entry["target"] = t;
                d_entry["approx_shortest_distance"] = dist;
                distances.push_back(d_entry);
            }
        }
        result["distances"] = distances;
    }

    return result;
}
int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << std::endl;
        return 1;
    }
    // 2. Read Graph JSON
    std::ifstream graph_file(argv[1]);
    if (!graph_file.is_open()) {
        std::cerr << "Failed to open graph file: " << argv[1] << std::endl;
        return 1;
    }
    json graph_json;
    graph_file >> graph_json;
    graph_file.close();

    // 3. Build Graph Object
    Graph graph;
    
    // Load Nodes
    if (graph_json.contains("nodes")) {
        for (const auto& n : graph_json["nodes"]) {
            Node node;
            node.id = n["id"];
            node.lat = n["lat"];
            node.lon = n["lon"];
            if (n.contains("pois")) {
                for(const auto& poi : n["pois"]) node.pois.push_back(poi);
            }
            graph.addNode(node);
        }
    }

    // Load Edges
    if (graph_json.contains("edges")) {
        for (const auto& e : graph_json["edges"]) {
            Edge edge;
            edge.id = e["id"];
            edge.u = e["u"];
            edge.v = e["v"];
            edge.length = e["length"];
            edge.average_time = e.value("average_time", 0.0);
            edge.oneway = e.value("oneway", false);
            edge.road_type = e.value("road_type", "");
            
            if (e.contains("speed_profile")) {
                for(double s : e["speed_profile"]) edge.speed_profile.push_back(s);
            }
            
            graph.addEdge(edge);
        }
    }

    // 4. Pre-processing (Task 3 Landmarks)
    // We compute landmarks immediately after loading graph
    Algorithm::precomputeLandmarks(graph);
    // 5. Read Queries JSON
    std::ifstream queries_file(argv[2]);
    if (!queries_file.is_open()) {
        std::cerr << "Failed to open queries file: " << argv[2] << std::endl;
        return 1;
    }
    json queries_json;
    queries_file >> queries_json;
    queries_file.close();
    json meta = queries_json.value("meta", json::object());
    std::vector<json> results;
    const json& events = queries_json.contains("events") ? queries_json["events"] : queries_json;

    if (events.is_array()) {
        for (const auto& query : events) {
            auto start_time = std::chrono::high_resolution_clock::now();
            
            json result;
            try {
                result = process_query(graph, query);
            } catch (const std::exception& e) {
                std::cerr << "Error processing query ID " << query.value("id", -1) << ": " << e.what() << std::endl;
                result = {{"error", e.what()}};
                if(query.contains("id")) result["id"] = query["id"];
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            if (!result.empty()) {
                result["processing_time"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();
                results.push_back(result);
            }
        }
    }

    std::ofstream output_file(argv[3]);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open output file: " << argv[3] << std::endl;
        return 1;
    }
    json output;
    if (!meta.empty()) output["meta"] = meta;
    output["results"] = results;
    output_file << output.dump(4) << std::endl;
    output_file.close();
    return 0;
}