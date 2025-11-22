#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <string>
#include <limits>
#include <nlohmann/json.hpp>

// Include Phase 3 specific headers
#include "Graph.hpp"
#include "Algorithm.hpp"

using json = nlohmann::json;

int main(int argc, char* argv[]) {
    // 1. Argument Validation
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << std::endl;
        return 1;
    }

    // -------------------------------------------------------
    // 2. Read Graph JSON
    // -------------------------------------------------------
    std::ifstream graph_file(argv[1]);
    if (!graph_file.is_open()) {
        std::cerr << "Failed to open graph file: " << argv[1] << std::endl;
        return 1;
    }
    json graph_json;
    graph_file >> graph_json;
    graph_file.close();

    // Build Graph Object
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

    // -------------------------------------------------------
    // 3. Read Queries (Orders) JSON
    // -------------------------------------------------------
    std::ifstream queries_file(argv[2]);
    if (!queries_file.is_open()) {
        std::cerr << "Failed to open queries file: " << argv[2] << std::endl;
        return 1;
    }
    json input_json;
    queries_file >> input_json;
    queries_file.close();

    // Parse Problem Configuration
    int depot_node = input_json.value("depot_node", 0);
    int num_drivers = 1;
    
    if (input_json.contains("fleet")) {
        num_drivers = input_json["fleet"].value("num_delievery_guys", 1);
    }

    // Parse Orders
    std::vector<Order> orders;
    if (input_json.contains("orders")) {
        for (const auto& o : input_json["orders"]) {
            Order order;
            order.id = o["order_id"];
            order.pickup_node = o["pickup"];
            order.dropoff_node = o["dropoff"];
            
            // --- FIX: PARSE NEW PHASE 3 FIELDS ---
            
            // 1. Deadline (Default Infinity)
            if (o.contains("deadline")) {
                order.deadline = o["deadline"];
            } else {
                order.deadline = std::numeric_limits<double>::infinity();
            }

            // 2. Prep Time / Ready Time (Crucial for Batching)
            if (o.contains("prep_time")) {
                order.prep_time = o["prep_time"];
                order.ready_time = o["prep_time"]; // Sync ready_time
            }

            // 3. Priority & Price (Crucial for Sorting/Cost)
            order.priority = o.value("priority", 0.0);
            order.price = o.value("price", 0.0);

            // 4. Gated Community Flag
            if (o.contains("requires_extended_dwell_time")) {
                order.requires_extended_dwell_time = o["requires_extended_dwell_time"];
            }
            // -------------------------------------

            orders.push_back(order);
        }
    }

    // -------------------------------------------------------
    // 4. Run Scheduling Algorithm
    // -------------------------------------------------------
    GraphAdapter adapter(graph);

    Scheduler::Config scheduler_config;
    scheduler_config.use_approximate_for_eval = true;
    
    Scheduler scheduler(adapter, depot_node, num_drivers, scheduler_config);

    std::cout << "Loading " << orders.size() << " orders..." << std::endl;
    scheduler.loadOrders(std::move(orders));

    std::cout << "Running optimization..." << std::endl;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        scheduler.run();
    } catch (const std::exception& e) {
        std::cerr << "Critical error during scheduling: " << e.what() << std::endl;
        return 1;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    double duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    std::cout << "Optimization completed in " << duration_ms << " ms." << std::endl;

    // -------------------------------------------------------
    // 5. Format Output
    // -------------------------------------------------------
    Scheduler::Output sched_result = scheduler.getResults();
    
    json final_output;
    
    json assignments_arr = json::array();
    for (const auto& assign : sched_result.assignments) {
        json a_json;
        a_json["driver_id"] = assign.driver_id;
        a_json["route"] = assign.route;
        a_json["order_ids"] = assign.order_ids;
        assignments_arr.push_back(a_json);
    }
    final_output["assignments"] = assignments_arr;

    json metrics_json;
    metrics_json["total_delivery_time_s"] = sched_result.metrics.total_delivery_time_s;
    metrics_json["max_delivery_time_s"] = sched_result.metrics.max_delivery_time_s;
    metrics_json["orders_completed"] = sched_result.metrics.orders_completed;
    metrics_json["orders_failed"] = sched_result.metrics.orders_failed;
    
    final_output["metrics"] = metrics_json;

    // -------------------------------------------------------
    // 6. Write to File
    // -------------------------------------------------------
    std::ofstream output_file(argv[3]);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open output file: " << argv[3] << std::endl;
        return 1;
    }

    output_file << final_output.dump(4) << std::endl;
    output_file.close();

    return 0;
}