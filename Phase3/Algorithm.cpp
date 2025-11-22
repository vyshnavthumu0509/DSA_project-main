#include "Algorithm.hpp"
#include <random>
#include <chrono>

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
    // Using Heuristic K-SP with k=1 as proxy for exact SP
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
    return Algorithm::kShortestPathsHeuristic(lib_graph, u, v, k, 0.5);
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
    if (route.empty()) { current_time = 0.0; return; }

    route[0].arrival_time = 0.0;
    std::unordered_map<int, double> pickup_times;
    
    for (int i = 0; i < start_idx && i < (int)route.size(); ++i) {
        if (route[i].is_pickup) {
            pickup_times[route[i].order_id] = route[i].arrival_time;
        }
    }
    
    for (size_t i = std::max(1, start_idx); i < route.size(); ++i) {
        int u = route[i - 1].node_id;
        int v = route[i].node_id;
        
        double travel_time = adapter.getCachedOrCompute(u, v, use_approximate);
        double arrival = route[i - 1].arrival_time + travel_time;
        
        if (!route[i].is_pickup) {
            int oid = route[i].order_id;
            if (pickup_times.count(oid)) {
                arrival = std::max(arrival, pickup_times.at(oid));
            }
        }
        
        route[i].arrival_time = arrival;
        if (route[i].is_pickup) {
            pickup_times[route[i].order_id] = route[i].arrival_time;
        }
    }
    current_time = route.back().arrival_time;
}

// Helper to look up order by ID
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
        for (int d_idx = p_idx + 1; d_idx <= L + 1; ++d_idx) {
            std::vector<RouteStop> temp_route = route;
            
            temp_route.insert(temp_route.begin() + p_idx, {new_order.pickup_node, new_order.id, true, 0.0});
            temp_route.insert(temp_route.begin() + d_idx, {new_order.dropoff_node, new_order.id, false, 0.0});

            std::unordered_map<int, double> pickup_times;
            for (int i = 0; i < p_idx; ++i) {
                if (route[i].is_pickup) {
                    pickup_times[route[i].order_id] = route[i].arrival_time;
                }
            }

            bool feasible = true;
            double new_sum_completion_times = 0;

            for (size_t i = p_idx; i < temp_route.size(); ++i) {
                int u = temp_route[i - 1].node_id;
                int v = temp_route[i].node_id;
                
                double travel_time = adapter.getCachedOrCompute(u, v, use_approximate);
                if (travel_time >= INF) { feasible = false; break; }
                
                double arrival = temp_route[i - 1].arrival_time + travel_time;
                
                if (temp_route[i].is_pickup && temp_route[i].order_id == new_order.id) {
                    arrival = std::max(arrival, new_order.prep_time);
                }
                
                if (!temp_route[i].is_pickup) {
                    int oid = temp_route[i].order_id;
                    if (oid != -1 && pickup_times.count(oid)) {
                        arrival = std::max(arrival, pickup_times.at(oid));
                    }
                }
                
                temp_route[i].arrival_time = arrival;
                
                if (temp_route[i].is_pickup) {
                    pickup_times[temp_route[i].order_id] = temp_route[i].arrival_time;
                }

                // Check deadlines for ALL orders involved
                if (!temp_route[i].is_pickup && temp_route[i].order_id != -1) {
                    double deadline = INF;
                    if(temp_route[i].order_id == new_order.id) {
                        deadline = new_order.deadline;
                    } else {
                        const Order* existing = getOrderById(all_orders, temp_route[i].order_id);
                        if(existing) deadline = existing->deadline;
                    }
                    
                    if (arrival > deadline) { feasible = false; break; }
                    new_sum_completion_times += arrival;
                }
            }
            
            if (!feasible) continue;

            for(int i=0; i<p_idx; ++i) {
                if(!temp_route[i].is_pickup && temp_route[i].order_id != -1) {
                    new_sum_completion_times += temp_route[i].arrival_time;
                }
            }

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
    // Fallback to full insertion for correctness
    return findBestInsertion(new_order, adapter, all_orders, best_p_idx, best_d_idx, use_approximate);
}

void Driver::commitInsertion(const Order& new_order, int p_idx, int d_idx) {
    route.insert(route.begin() + p_idx, {new_order.pickup_node, new_order.id, true, 0.0});
    // d_idx was relative to route size *before* pickup was inserted in findBestInsertion?
    // No, findBestInsertion logic calculates d_idx relative to the *temporary* route which included pickup.
    // Wait, let's check logic carefully.
    // In findBestInsertion: temp_route.insert(begin+p_idx...); temp_route.insert(begin+d_idx...);
    // If d_idx > p_idx, then indices are fine relative to the expanding vector.
    // But commitInsertion is operating on the ORIGINAL vector sequentially.
    // If we insert at p_idx, all indices >= p_idx shift by 1.
    // The d_idx from loop was relative to a vector that ALREADY had pickup inserted.
    // So we can use the exact indices if we do it in the same order (assuming d_idx is indeed > p_idx).
    
    route.insert(route.begin() + d_idx, {new_order.dropoff_node, new_order.id, false, 0.0});
    
    orders_being_carried.insert(new_order.id);
    order_count++;
}

std::vector<int> Driver::getFinalRouteNodes() const {
    std::vector<int> nodes;
    nodes.reserve(route.size());
    for (const auto& stop : route) nodes.push_back(stop.node_id);
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
    
    std::cout << "Precomputing landmarks..." << std::endl;
    adapter.precomputeLandmarks();
    if (important_nodes.size() <= 200) {
        adapter.precomputeDistanceTable(important_nodes);
    }
    std::cout << "Precomputation complete." << std::endl;
}

void Scheduler::run() {
    std::cout << "Starting Optimized Scheduler..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    
    // Sort orders: High Priority (low value) -> Early Deadline
    std::sort(orders.begin(), orders.end(), [](const Order& a, const Order& b) {
        if (std::abs(a.priority - b.priority) > 1e-6) return a.priority < b.priority;
        return a.deadline < b.deadline;
    });
    
    using DriverEntry = std::tuple<double, double, int>; // time, load, id
    std::priority_queue<DriverEntry, std::vector<DriverEntry>, std::greater<>> driver_heap;

    for (const auto& driver : drivers) {
        driver_heap.push({driver.current_time, driver.getLoadFactor(), driver.id});
    }

    int orders_completed = 0, orders_failed = 0;

    for (auto& current_order : orders) {
        DriverEntry top = driver_heap.top();
        driver_heap.pop();
        int driver_id = std::get<2>(top);
        Driver& assigned_driver = drivers[driver_id];
        
        if (!assigned_driver.canAcceptMore()) {
            orders_failed++;
            driver_heap.push({assigned_driver.current_time, assigned_driver.getLoadFactor(), driver_id});
            continue;
        }

        int best_p_idx = -1, best_d_idx = -1;
        double cost_increase = assigned_driver.findBestInsertion(
            current_order, adapter, orders, best_p_idx, best_d_idx,
            config.use_approximate_for_eval);

        if (cost_increase >= INF || best_p_idx == -1) {
            orders_failed++;
            std::cerr << "Order " << current_order.id << " failed (Priority: " << current_order.priority << " Deadline: " << current_order.deadline << ")" << std::endl;
            driver_heap.push({assigned_driver.current_time, assigned_driver.getLoadFactor(), driver_id});
            continue;
        }

        assigned_driver.commitInsertion(current_order, best_p_idx, best_d_idx);
        assigned_driver.recalculateRouteTimesFrom(adapter, best_p_idx, false);

        // Update completion time
        for (auto it = assigned_driver.route.rbegin(); it != assigned_driver.route.rend(); ++it) {
            if (it->order_id == current_order.id && !it->is_pickup) {
                current_order.completion_time = it->arrival_time;
                break;
            }
        }

        orders_completed++;
        driver_heap.push({assigned_driver.current_time, assigned_driver.getLoadFactor(), driver_id});
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Scheduling complete in " << duration.count() << "ms. "
              << "Completed: " << orders_completed << ", Failed: " << orders_failed << std::endl;
}

void Scheduler::runParallel() {
    run(); // Fallback to serial
}

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
        assignment.route = driver.getFinalRouteNodes();
        assignment.total_time = driver.current_time;

        std::set<int> unique_ids;
        for (const auto& stop : driver.route) {
            if (stop.order_id != -1) unique_ids.insert(stop.order_id);
        }
        assignment.order_ids.assign(unique_ids.begin(), unique_ids.end());
        result.assignments.push_back(assignment);
    }
    return result;
}

// Stub utilities
void Scheduler::findAlternativeRoutes(int, int, int) {}
void Scheduler::simulateTrafficDelays(double, double) {}