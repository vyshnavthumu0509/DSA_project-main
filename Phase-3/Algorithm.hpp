#ifndef ALGORITHM_HPP
#define ALGORITHM_HPP

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <set>
#include <tuple>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>

#include "../Phase-2/Algorithm.hpp"
#include "Graph.hpp"

struct PairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
};
namespace Config {
    constexpr double BATCH_THRESHOLD = 900.0;     // 15 mins
    constexpr double GATED_DELAY = 300.0;         // 5 mins
    constexpr double PRICE_WEIGHT = 30.0;         // $1 = 30s time
    constexpr double MAX_NEG_REWARD = 7200.0;     // Cap profit
    constexpr double MAX_SHIFT_SECONDS = 28800.0; // 8 hours
    
    // Node Dwell Times
    constexpr double DWELL_MALL = 180.0;
    constexpr double DWELL_APT = 60.0;
    constexpr double DWELL_HOTEL = 120.0;
}
struct RouteStop {
    int node_id;
    int order_id;  // -1 for depot
    bool is_pickup;
    double arrival_time;
 bool requires_extended_dwell = false; 
    double node_dwell_time = 0.0;
    double ready_time = 0.0;
};

struct Order {
    int id;
    int pickup_node;
    int dropoff_node;
    double completion_time = 0.0;
    double priority = 0.0;        // Lower value = Higher priority
    double prep_time = 0.0;       
    double deadline = std::numeric_limits<double>::infinity(); 
 double ready_time = 0.0;  // Absolute time order is ready
    double price = 0.0;           
    bool requires_extended_dwell_time = false;
};

class GraphAdapter {
private:
    const Graph& lib_graph;
    std::unordered_map<std::pair<int, int>, double, PairHash> distance_cache;
    std::mutex cache_mutex;
    
    std::unordered_map<std::pair<int, int>, double, PairHash> precomputed_distances;
    bool distances_precomputed = false;

public:
    explicit GraphAdapter(const Graph& g);

    double getApproximateDistance(int u, int v);
    double getExactShortestDistance(int u, int v);
    double getCachedOrCompute(int u, int v, bool use_approximate);
    
    void precomputeLandmarks();
    void precomputeDistanceTable(const std::vector<int>& important_nodes);
    std::vector<path> getKShortestPaths(int u, int v, int k);
    const Graph& getLibGraph() const;
    void clearCache();
    size_t getCacheSize() const;
};
class Driver {
public:
    int id;
    std::vector<RouteStop> route;
    double current_time;
    std::set<int> orders_being_carried;
    int order_count = 0; 
    
    static constexpr int MAX_ORDERS_PER_DRIVER = 10;
    static constexpr double MAX_ROUTE_TIME = 28800.0; 

    Driver(int driver_id, int depot_node);

    void recalculateRouteTimes(GraphAdapter& adapter, bool use_approximate);
    void recalculateRouteTimesFrom(GraphAdapter& adapter, int start_idx, bool use_approximate);

    double findBestInsertion(const Order& new_order, GraphAdapter& adapter,
                             const std::vector<Order>& all_orders,
                             int& best_p_idx, int& best_d_idx,
                             bool use_approximate, double current_best = std::numeric_limits<double>::infinity()) const;

    double findBestInsertionLimited(const Order& new_order, GraphAdapter& adapter,
                                    const std::vector<Order>& all_orders,
                                    int& best_p_idx, int& best_d_idx,
                                    bool use_approximate, int max_candidates = 5) const;

    void commitInsertion(const Order& new_order, int p_idx, int d_idx,const GraphAdapter& adapter);
    std::vector<int> getFinalRouteNodes() const;
    bool canAcceptMore() const;
    double getLoadFactor() const;
};

class Scheduler {
public:
    struct Metrics {
        double total_delivery_time_s;
        double max_delivery_time_s;
        double avg_delivery_time_s;
        int orders_completed;
        int orders_failed;
    };

    struct AssignmentOutput {
        int driver_id;
        std::vector<int> route;
        std::vector<int> order_ids;
        double total_time;
    };

    struct Output {
        Metrics metrics;
        std::vector<AssignmentOutput> assignments;
    };

    struct Config {
        bool use_approximate_for_eval = true;
        bool use_parallel_eval = false;
        bool use_limited_candidates = true;
        int max_insertion_candidates = 5;
        double max_delivery_time_threshold = 3600.0;
        double traffic_factor = 1.0;
        bool balance_driver_loads = true;
    };

private:
    GraphAdapter& adapter;
    int depot_node;
    Config config;
    std::vector<Driver> drivers;
    std::vector<Order> orders;
    std::mutex result_mutex;
    

public:
    Scheduler(GraphAdapter& ga, int depot, int num_drivers, const Config& cfg);

    void loadOrders(std::vector<Order>&& o);
    void setConfig(const Config& cfg);
    void run();
    void runParallel();
    Output getResults();
    void findAlternativeRoutes(int pickup, int dropoff, int k = 3);
    void simulateTrafficDelays(double min_factor = 0.9, double max_factor = 1.5);
};

#endif 