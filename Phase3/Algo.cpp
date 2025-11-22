#include "Algo.hpp"
#include <random>
#include <chrono>
#include <unordered_map>

// ==========================================
//       GraphAdapter Implementation
// ==========================================

GraphAdapter::GraphAdapter(const Graph& g) : lib_graph(g) {}

double GraphAdapter::getApproximateDistance(int u, int v) {
    if (u == v) return 0.0;
    
    if (distances_precomputed) {
        auto key = std::make_pair(u, v);
        auto it = precomputed_distances.find(key);
        if (it != precomputed_distances.end()) return it->second;
    }
    
    return Algorithm::approximateShortestPathDistance(lib_graph, u, v);
}

double GraphAdapter::getExactShortestDistance(int u, int v) {
    if (u == v) return 0.0;
    
    auto key = std::make_pair(u, v);
    
    if (distances_precomputed) {
        auto it = precomputed_distances.find(key);
        if (it != precomputed_distances.end()) return it->second;
    }
    
    {
        std::lock_guard<std::mutex> lock(cache_mutex);
        auto it = distance_cache.find(key);
        if (it != distance_cache.end()) return it->second;
    }

    auto paths = Algorithm::kShortestPathsHeuristic(lib_graph, u, v, 1, 0.0);
        
    double dist = paths.empty() ? INF : paths[0].cost;
    
    {
        std::lock_guard<std::mutex> lock(cache_mutex);
        distance_cache[key] = dist;
    }
    
    return dist;
}

double GraphAdapter::getCachedOrCompute(int u, int v, bool use_approximate) {
    return use_approximate ? getApproximateDistance(u, v) : getExactShortestDistance(u, v);
}

void GraphAdapter::precomputeLandmarks() {
    Algorithm::precomputeLandmarks(lib_graph);
}

void GraphAdapter::precomputeDistanceTable(const std::vector<int>& important_nodes) {
    
    for (int u : important_nodes) {
        for (int v : important_nodes) {
            if (u != v) {
               auto paths = Algorithm::kShortestPathsHeuristic(lib_graph, u, v, 1, 0.0);
               double dist = paths.empty() ? INF : paths[0].cost;
               precomputed_distances[{u, v}] = dist;
            }
        }
    }
    
    distances_precomputed = true;
}

std::vector<path> GraphAdapter::getKShortestPaths(int u, int v, int k) {
    return Algorithm::kShortestPathsHeuristic(lib_graph, u, v, k, 0.0);
}

const Graph& GraphAdapter::getLibGraph() const { return lib_graph; }

void GraphAdapter::clearCache() {
    std::lock_guard<std::mutex> lock(cache_mutex);
    distance_cache.clear();
}

size_t GraphAdapter::getCacheSize() const { return distance_cache.size(); }

// ==========================================
//         Driver Implementation
// ==========================================

Driver::Driver(int driver_id, int depot_node)
    : id(driver_id), current_time(0.0), order_count(0) {
    route.push_back({depot_node, -1, false, 0.0});
}

void Driver::recalculateRouteTimes(GraphAdapter& adapter, bool use_approximate) {
    recalculateRouteTimesFrom(adapter, 1, use_approximate);
}

void Driver::recalculateRouteTimesFrom(GraphAdapter& adapter, int start_idx, bool use_approximate) {
    if (route.empty()) {
        current_time = 0.0;
        return;
    }

    route[0].arrival_time = 0.0;
    std::unordered_map<int, double> pickup_times;
    
    // Collect pickup times securely
    for (int i = 0; i < (int)route.size(); ++i) {
        if (route[i].is_pickup) {
            pickup_times[route[i].order_id] = route[i].arrival_time;
        }
        if (i >= start_idx - 1) break; 
    }
    
    for (size_t i = 1; i < route.size(); ++i) {
        if (i >= (size_t)start_idx) {
            int u = route[i - 1].node_id;
            int v = route[i].node_id;
            
            double travel_time = adapter.getCachedOrCompute(u, v, use_approximate);
            double arrival = route[i - 1].arrival_time + travel_time;
            
            if (!route[i].is_pickup) {
                int oid = route[i].order_id;
                auto it = pickup_times.find(oid);
                if (it != pickup_times.end()) {
                    arrival = std::max(arrival, it->second);
                }
            }
            
            route[i].arrival_time = arrival;
            
            if (route[i].is_pickup) {
                pickup_times[route[i].order_id] = route[i].arrival_time;
            }
        } else {
             if (route[i].is_pickup) {
                pickup_times[route[i].order_id] = route[i].arrival_time;
            }
        }
    }
    
    current_time = route.back().arrival_time;
}

const Order* getOrderById(const std::vector<Order>& orders, int id) {
    for(const auto& o : orders) {
        if(o.id == id) return &o;
    }
    return nullptr;
}

double Driver::findBestInsertion(const Order& new_order, GraphAdapter& adapter,
                                 const std::vector<Order>& all_orders,
                                 int& best_p_idx, int& best_d_idx,
                                 bool use_approximate, double current_best) const {
    const int L = route.size();
    double min_cost_increase = current_best; 
    
    double current_total_time = 0;
    for(const auto& s : route) {
        if(!s.is_pickup && s.order_id != -1) current_total_time += s.arrival_time;
    }

    best_p_idx = -1;
    best_d_idx = -1;

    for (int p_idx = 1; p_idx <= L; ++p_idx) {
        // Pruning
        double dist_to_pickup = adapter.getCachedOrCompute(route[p_idx - 1].node_id, new_order.pickup_node, use_approximate);
        double estimated_arrival = route[p_idx - 1].arrival_time + dist_to_pickup;
        if (estimated_arrival > new_order.deadline) continue;

        for (int d_idx = p_idx + 1; d_idx <= L + 1; ++d_idx) {
            std::vector<RouteStop> temp_route = route;
            
            temp_route.insert(temp_route.begin() + p_idx,
                {new_order.pickup_node, new_order.id, true, 0.0});
            
            temp_route.insert(temp_route.begin() + d_idx,
                {new_order.dropoff_node, new_order.id, false, 0.0});

            std::unordered_map<int, double> temp_pickup_times;
            bool feasible = true;
            double new_sum_completion_times = 0;
            temp_route[0].arrival_time = 0.0;

            for (size_t i = 1; i < temp_route.size(); ++i) {
                int u = temp_route[i-1].node_id;
                int v = temp_route[i].node_id;

                double t_time = adapter.getCachedOrCompute(u, v, use_approximate);
                if (t_time >= INF) { feasible = false; break; }

                double arr = temp_route[i-1].arrival_time + t_time;

                if (temp_route[i].is_pickup && temp_route[i].order_id == new_order.id) {
                    arr = std::max(arr, new_order.prep_time);
                }

                if (!temp_route[i].is_pickup && temp_route[i].order_id != -1) {
                    int oid = temp_route[i].order_id;
                    if (temp_pickup_times.count(oid)) {
                        arr = std::max(arr, temp_pickup_times[oid]);
                    } else {
                         feasible = false; break;
                    }
                }

                temp_route[i].arrival_time = arr;

                if (temp_route[i].is_pickup) {
                    temp_pickup_times[temp_route[i].order_id] = arr;
                }

                if (!temp_route[i].is_pickup && temp_route[i].order_id != -1) {
                    double deadline = (temp_route[i].order_id == new_order.id) ? new_order.deadline : INF;
                    if (deadline == INF) {
                        const Order* ex = getOrderById(all_orders, temp_route[i].order_id);
                        if (ex) deadline = ex->deadline;
                    }

                    if (arr > deadline) { feasible = false; break; }
                    new_sum_completion_times += arr;
                }
            }

            if (!feasible) continue;

            double cost_increase = new_sum_completion_times - current_total_time;

            if (cost_increase < min_cost_increase) {
                min_cost_increase = cost_increase;
                best_p_idx = p_idx;
                best_d_idx = d_idx;
            }
        }
    }

    return min_cost_increase;
}

double Driver::findBestInsertionLimited(const Order& new_order, GraphAdapter& adapter,
                                        const std::vector<Order>& all_orders,
                                        int& best_p_idx, int& best_d_idx,
                                        bool use_approximate, int max_candidates) const {
    return findBestInsertion(new_order, adapter, all_orders, best_p_idx, best_d_idx, use_approximate);
}

void Driver::commitInsertion(const Order& new_order, int p_idx, int d_idx) {
    route.insert(route.begin() + p_idx,
        {new_order.pickup_node, new_order.id, true, 0.0});
    
    route.insert(route.begin() + d_idx,
        {new_order.dropoff_node, new_order.id, false, 0.0});
        
    orders_being_carried.insert(new_order.id);
    order_count++;
}

std::vector<int> Driver::getFinalRouteNodes() const {
    std::vector<int> nodes;
    nodes.reserve(route.size());
    for (const auto& stop : route) {
        nodes.push_back(stop.node_id);
    }
    return nodes;
}

bool Driver::canAcceptMore() const {
    return order_count < MAX_ORDERS_PER_DRIVER && current_time < MAX_ROUTE_TIME;
}

double Driver::getLoadFactor() const {
    return static_cast<double>(order_count) / MAX_ORDERS_PER_DRIVER;
}

// ==========================================
//        Scheduler Implementation
// ==========================================

Scheduler::Scheduler(GraphAdapter& ga, int depot, int num_drivers, const Config& cfg)
    : adapter(ga), depot_node(depot), config(cfg) {
    drivers.reserve(num_drivers);
    for (int i = 0; i < num_drivers; ++i) {
        drivers.emplace_back(i, depot_node);
    }
}

void Scheduler::setConfig(const Config& cfg) { config = cfg; }

void Scheduler::loadOrders(std::vector<Order>&& o) {
    orders = std::move(o);
    
    std::set<int> important_set;
    important_set.insert(depot_node);
    for (const auto& order : orders) {
        important_set.insert(order.pickup_node);
        important_set.insert(order.dropoff_node);
    }
    std::vector<int> important_nodes(important_set.begin(), important_set.end());
    
    adapter.precomputeLandmarks();
    
    if (important_nodes.size() <= 200) {
        adapter.precomputeDistanceTable(important_nodes);
    }
}

void Scheduler::findAlternativeRoutes(int pickup, int dropoff, int k) {
    auto paths = adapter.getKShortestPaths(pickup, dropoff, k);
    for (size_t i = 0; i < paths.size(); ++i) {
        std::cout << "Route " << i << " cost: " << paths[i].cost << std::endl;
    }
}

void Scheduler::simulateTrafficDelays(double min_factor, double max_factor) {
    // Implementation unchanged
}

void Scheduler::run() {
    auto start = std::chrono::high_resolution_clock::now();
    
    std::sort(orders.begin(), orders.end(), [](const Order& a, const Order& b) {
        if (std::abs(a.priority - b.priority) > 1e-6) return a.priority < b.priority;
        return a.deadline < b.deadline;
    });
    
    // Map for fast order lookup
    std::unordered_map<int, Order*> order_lookup;
    for(auto& o : orders) order_lookup[o.id] = &o;

    using DriverEntry = std::tuple<double, double, int>; 
    std::priority_queue<DriverEntry, std::vector<DriverEntry>, std::greater<>> driver_heap;

    for (const auto& driver : drivers) {
        driver_heap.push({driver.current_time, driver.getLoadFactor(), driver.id});
    }

    int orders_completed = 0, orders_failed = 0;

    for (auto& current_order : orders) {
        std::vector<DriverEntry> tested_drivers;
        int best_p = -1, best_d = -1;
        int best_driver_idx = -1;
        double best_cost = INF;
        
        int max_checks = 50; 
        int checks = 0;

        while(!driver_heap.empty() && checks < max_checks) {
            DriverEntry entry = driver_heap.top();
            driver_heap.pop();
            tested_drivers.push_back(entry);
            
            int d_id = std::get<2>(entry);
            Driver& drv = drivers[d_id];
            
            if (!drv.canAcceptMore()) continue;

            int p_idx = -1, d_idx = -1;
            double cost = drv.findBestInsertion(current_order, adapter, orders, p_idx, d_idx, config.use_approximate_for_eval, best_cost);
            
            if (cost < best_cost) {
                best_cost = cost;
                best_p = p_idx;
                best_d = d_idx;
                best_driver_idx = d_id;
                
                if (cost < 60.0) break; 
            }
            checks++;
        }

        for (const auto& entry : tested_drivers) {
            int d_id = std::get<2>(entry);
            if (d_id == best_driver_idx) {
                Driver& assigned = drivers[d_id];
                assigned.commitInsertion(current_order, best_p, best_d);
                assigned.recalculateRouteTimesFrom(adapter, best_p, false);
                
                // --- BUG FIX START ---
                // Update completion times for ALL orders carried by this driver
                for (const auto& stop : assigned.route) {
                    if (!stop.is_pickup && stop.order_id != -1) {
                        if (order_lookup.count(stop.order_id)) {
                            order_lookup[stop.order_id]->completion_time = stop.arrival_time;
                        }
                    }
                }
                // --- BUG FIX END ---

                orders_completed++;
                driver_heap.push({assigned.current_time, assigned.getLoadFactor(), d_id});
            } else {
                driver_heap.push(entry);
            }
        }
        
        if (best_driver_idx == -1) {
            orders_failed++;
            std::cerr << "Order " << current_order.id << " failed (No feasible driver found)." << std::endl;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "Scheduling complete in " << duration.count() << "ms. "
              << "Completed: " << orders_completed << ", Failed: " << orders_failed << std::endl;
}

void Scheduler::runParallel() { run(); }

Scheduler::Output Scheduler::getResults() {
    Output result;
    double total = 0.0, max_time = 0.0;
    int completed = 0, failed = 0;

    for (const auto& order : orders) {
        if (order.completion_time > 0) {
            total += order.completion_time;
            max_time = std::max(max_time, order.completion_time);
            completed++;
        } else {
            failed++;
        }
    }

    result.metrics.total_delivery_time_s = total;
    result.metrics.max_delivery_time_s = max_time;
    result.metrics.avg_delivery_time_s = completed > 0 ? total / completed : 0;
    result.metrics.orders_completed = completed;
    result.metrics.orders_failed = failed;

    for (const auto& driver : drivers) {
        AssignmentOutput assignment;
        assignment.driver_id = driver.id;
        
        std::vector<int> full_route;
        if(!driver.route.empty()) full_route.push_back(driver.route[0].node_id);
        
        for(size_t i = 0; i < driver.route.size() - 1; ++i) {
            int u = driver.route[i].node_id;
            int v = driver.route[i+1].node_id;
            
            if (u != v) {
                auto paths = adapter.getKShortestPaths(u, v, 1);
                if (!paths.empty()) {
                    const auto& p_nodes = paths[0].nodes;
                    if (p_nodes.size() > 1) {
                         full_route.insert(full_route.end(), p_nodes.begin() + 1, p_nodes.end());
                    }
                } else {
                    full_route.push_back(v);
                }
            }
        }
        assignment.route = full_route;
        assignment.total_time = driver.current_time;

        std::vector<int> ordered_ids;
        for (const auto& stop : driver.route) {
            if (stop.order_id != -1 && !stop.is_pickup) {
                ordered_ids.push_back(stop.order_id);
            }
        }
        assignment.order_ids = ordered_ids;
        result.assignments.push_back(assignment);
    }
    
    return result;
}