
"""Test case generator for phase 1
Generates synthetic graph and query files for testing"""

import json
import random
import math


def generate_graph(num_nodes = 100, num_edges = 300, filename = "test_graph.json"):
    """Generate a random graph with realistic properties"""

    # Mumbai approx bounds
    lat_min, lat_max = 19.0, 19.3
    lon_min, lon_max = 72.8, 73.0

    pois_types = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]
    road_types = ["primary", "secondary", "tertiary", "local", "expressway"]

    nodes = []

    for i in range(num_nodes):
        node = {
            "id" : i,
            "lat" : random.uniform(lat_min, lat_max),
            "lon" : random.uniform(lon_min, lon_max),
            "pois" : random.sample(pois_types, k = random.randint(0, 2))
        }

        nodes.append(node)

    edges = []
    edge_id = 1000
    used_pairs = set()

    # Create a spanning tree
    visited = {0}
    unvisited = set(range(1, num_nodes))

    while unvisited:
        u = random.choice(list(visited))
        v = random.choice(list(unvisited))

        visited.add(v)
        unvisited.remove(v)

        # Calculus distance
        dx = (nodes[v]["lon"] - nodes[u]["lon"]) * 111320 * math.cos(math.radians((nodes[u]["lat"] + nodes[v]["lat"]) / 2))
        dy = (nodes[v]["lat"] - nodes[u]["lat"]) * 110540
        length = math.sqrt(dx * dx + dy * dy)

        avg_speed = random.randint(20, 60)  # m/s
        avg_time = length / avg_speed

        # Generate speed profile (96 slots)
        speed_profile = []
        for slot in range(96):
            # Simulate traffic patterns
            hour = (slot * 15) / 60
            if 7 <= hour < 10 or 17 <= hour < 20:
                speed = avg_speed * random.uniform(0.5, 0.8)
            
            else:
                speed = avg_speed * random.uniform(0.9, 1.1)
            speed_profile.append(round(speed, 2))

        edge = {
            "id" : edge_id,
            "u" : u,
            "v" : v,
            "length" : round(length, 2),
            "average_time" : round(avg_time, 2),
            "road_type" : random.choice(road_types),
            "speed_profile" : speed_profile,
            "oneway" : random.random() < 0.2
        }

        edges.append(edge)
        used_pairs.add((min(u, v), max(u, v)))
        edge_id += 1

    # Add random edges
    while len(edges) < num_edges:
        u = random.randint(0, num_nodes - 1)
        v = random.randint(0, num_nodes - 1)

        if u == v or (min(u, v), max(u, v)) in used_pairs:
            continue
        
        used_pairs.add((min(u, v), max(u, v)))

        # Calculus distance
        dx = (nodes[v]["lon"] - nodes[u]["lon"]) * 111320 * math.cos(math.radians((nodes[u]["lat"] + nodes[v]["lat"]) / 2))
        dy = (nodes[v]["lat"] - nodes[u]["lat"]) * 110540
        length = math.sqrt(dx * dx + dy * dy)

        avg_speed = random.randint(20, 60)  
        avg_time = length / avg_speed

        # Generate speed profile (96 slots)
        speed_profile = []
        for slot in range(96):
            # Simulate traffic patterns
            hour = (slot * 15) / 60
            if 7 <= hour < 10 or 17 <= hour < 20:
                speed = avg_speed * random.uniform(0.5, 0.8)
            else:
                speed = avg_speed * random.uniform(0.9, 1.1)
            speed_profile.append(round(speed, 2))

        edge = {
            "id" : edge_id,
            "u" : u,
            "v" : v,
            "length" : round(length, 2),
            "average_time" : round(avg_time, 2),
            "road_type" : random.choice(road_types),
            "speed_profile" : speed_profile,
            "oneway" : random.random() < 0.2
        }

        edges.append(edge)
        edge_id += 1

    graph = {
        "meta" : {
            "id" : "test_case_1",
            "nodes" : num_nodes,
            "description" : "Auto-generated test case"
        },
        "nodes" : nodes,
        "edges" : edges
    }

    with open(filename, "w") as f:
        json.dump(graph, f, indent = 2)

    print(f"Generated graph with {num_nodes} nodes and {num_edges} edges")
    return graph

def generate_queries(graph, num_queries = 50, filename = "test_queries.json") :
    """Generate random queries for the given graph"""

    num_nodes = len(graph["nodes"])
    edge_ids = [e["id"] for e in graph["edges"]]
    pois = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]

    events = []
    query_id = 1

    # Shortest path queries
    for _ in range(num_queries // 2):
        source = random.randint(0, num_nodes - 1)
        target = random.randint(0, num_nodes - 1)
        mode = random.choice(["distance", "time"])

        query = {
            "id" : query_id,
            "type" : "shortest_path",
            "source" : source,
            "target" : target,
            "mode" : mode
        }

        # Sometimes add constraints
        if random.random() < 0.3:
            query["constraints"] = {}
            if random.random() < 0.5:
                query["constraints"]["forbidden_nodes"] = random.sample(range(num_nodes), k = random.randint(1, 5))

            if random.random() < 0.5:
                query["constraints"]["forbidden_road_types"] = random.sample(["primary", "secondary", "expressway"], k = random.randint(1, 2))

        events.append(query)
        query_id += 1

        # KNN queries
        for _ in range(num_queries // 3):
            node = graph["nodes"][random.randint(0, num_nodes - 1)]

            query = {
                "type" : "knn",
                "id" : query_id,
                "poi" : random.choice(pois),
                "query_point" : {
                    "lat" : node["lat"] + random.uniform(-0.01, 0.01),
                    "lon" : node["lon"] + random.uniform(-0.01, 0.01)
                },
                "k" : random.randint(3, 10),
                "metric" : random.choice(["euclidean", "shortest_path"])
            }

            events.append(query)
            query_id += 1
        
        # Dynamic updates
        for _ in range(num_queries // 6):
            events.append({
                "type" : "modify_edge",
                "edge_id" : random.choice(edge_ids),
                "patch" : {
                    "length" : random.uniform(100, 1000),
                }
            })

        queries = {
            "meta" : {"id" : "test_queries_1"},
            "events" : events
        }

        with open(filename, "w") as f:
            json.dump(queries, f, indent = 2)

        print(f"Generated {len(events)} queries")

if __name__ == "__main__":
    graph = generate_graph(num_nodes = 50, num_edges = 150, filename = "test_graph.json")
    generate_queries(graph, num_queries = 30, filename = "test_queries.json")

    print("\nTest files generated successfully.")
    print("Run with: ./phase1 test_graph.json test_queries.json output.json")
=======
"""Test case generator for phase 1
Generates synthetic graph and query files for testing"""

import json
import random
import math


def generate_graph(num_nodes = 100, num_edges = 300, filename = "test_graph.json"):
    """Generate a random graph with realistic properties"""

    # Mumbai approx bounds
    lat_min, lat_max = 19.0, 19.3
    lon_min, lon_max = 72.8, 73.0

    pois_types = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]
    road_types = ["primary", "secondary", "tertiary", "local", "expressway"]

    nodes = []

    for i in range(num_nodes):
        node = {
            "id" : i,
            "lat" : random.uniform(lat_min, lat_max),
            "lon" : random.uniform(lon_min, lon_max),
            "pois" : random.sample(pois_types, k = random.randint(0, 2))
        }

        nodes.append(node)

    edges = []
    edge_id = 1000
    used_pairs = set()

    # Create a spanning tree
    visited = {0}
    unvisited = set(range(1, num_nodes))

    while unvisited:
        u = random.choice(list(visited))
        v = random.choice(list(unvisited))

        visited.add(v)
        unvisited.remove(v)

        # Calculus distance
        dx = (nodes[v]["lon"] - nodes[u]["lon"]) * 111320 * math.cos(math.radians((nodes[u]["lat"] + nodes[v]["lat"]) / 2))
        dy = (nodes[v]["lat"] - nodes[u]["lat"]) * 110540
        length = math.sqrt(dx * dx + dy * dy)

        avg_speed = random.randint(20, 60)  # m/s
        avg_time = length / avg_speed

        # Generate speed profile (96 slots)
        speed_profile = []
        for slot in range(96):
            # Simulate traffic patterns
            hour = (slot * 15) / 60
            if 7 <= hour < 10 or 17 <= hour < 20:
                speed = avg_speed * random.uniform(0.5, 0.8)
            
            else:
                speed = avg_speed * random.uniform(0.9, 1.1)
            speed_profile.append(round(speed, 2))

        edge = {
            "id" : edge_id,
            "u" : u,
            "v" : v,
            "length" : round(length, 2),
            "average_time" : round(avg_time, 2),
            "road_type" : random.choice(road_types),
            "speed_profile" : speed_profile,
            "oneway" : random.random() < 0.2
        }

        edges.append(edge)
        used_pairs.add((min(u, v), max(u, v)))
        edge_id += 1

    # Add random edges
    while len(edges) < num_edges:
        u = random.randint(0, num_nodes - 1)
        v = random.randint(0, num_nodes - 1)

        if u == v or (min(u, v), max(u, v)) in used_pairs:
            continue
        
        used_pairs.add((min(u, v), max(u, v)))

        # Calculus distance
        dx = (nodes[v]["lon"] - nodes[u]["lon"]) * 111320 * math.cos(math.radians((nodes[u]["lat"] + nodes[v]["lat"]) / 2))
        dy = (nodes[v]["lat"] - nodes[u]["lat"]) * 110540
        length = math.sqrt(dx * dx + dy * dy)

        avg_speed = random.randint(20, 60)  
        avg_time = length / avg_speed

        # Generate speed profile (96 slots)
        speed_profile = []
        for slot in range(96):
            # Simulate traffic patterns
            hour = (slot * 15) / 60
            if 7 <= hour < 10 or 17 <= hour < 20:
                speed = avg_speed * random.uniform(0.5, 0.8)
            else:
                speed = avg_speed * random.uniform(0.9, 1.1)
            speed_profile.append(round(speed, 2))

        edge = {
            "id" : edge_id,
            "u" : u,
            "v" : v,
            "length" : round(length, 2),
            "average_time" : round(avg_time, 2),
            "road_type" : random.choice(road_types),
            "speed_profile" : speed_profile,
            "oneway" : random.random() < 0.2
        }

        edges.append(edge)
        edge_id += 1

    graph = {
        "meta" : {
            "id" : "test_case_1",
            "nodes" : num_nodes,
            "description" : "Auto-generated test case"
        },
        "nodes" : nodes,
        "edges" : edges
    }

    with open(filename, "w") as f:
        json.dump(graph, f, indent = 2)

    print(f"Generated graph with {num_nodes} nodes and {num_edges} edges")
    return graph

def generate_queries(graph, num_queries = 50, filename = "test_queries.json") :
    """Generate random queries for the given graph"""

    num_nodes = len(graph["nodes"])
    edge_ids = [e["id"] for e in graph["edges"]]
    pois = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]

    events = []
    query_id = 1

    # Shortest path queries
    for _ in range(num_queries // 2):
        source = random.randint(0, num_nodes - 1)
        target = random.randint(0, num_nodes - 1)
        mode = random.choice(["distance", "time"])

        query = {
            "id" : query_id,
            "type" : "shortest_path",
            "source" : source,
            "target" : target,
            "mode" : mode
        }

        # Sometimes add constraints
        if random.random() < 0.3:
            query["constraints"] = {}
            if random.random() < 0.5:
                query["constraints"]["forbidden_nodes"] = random.sample(range(num_nodes), k = random.randint(1, 5))

            if random.random() < 0.5:
                query["constraints"]["forbidden_road_types"] = random.sample(["primary", "secondary", "expressway"], k = random.randint(1, 2))

        events.append(query)
        query_id += 1

        # KNN queries
        for _ in range(num_queries // 3):
            node = graph["nodes"][random.randint(0, num_nodes - 1)]

            query = {
                "type" : "knn",
                "id" : query_id,
                "poi" : random.choice(pois),
                "query_point" : {
                    "lat" : node["lat"] + random.uniform(-0.01, 0.01),
                    "lon" : node["lon"] + random.uniform(-0.01, 0.01)
                },
                "k" : random.randint(3, 10),
                "metric" : random.choice(["euclidean", "shortest_path"])
            }

            events.append(query)
            query_id += 1
        
        # Dynamic updates
        for _ in range(num_queries // 6):
            events.append({
                "type" : "modify_edge",
                "edge_id" : random.choice(edge_ids),
                "patch" : {
                    "length" : random.uniform(100, 1000),
                }
            })

        queries = {
            "meta" : {"id" : "test_queries_1"},
            "events" : events
        }

        with open(filename, "w") as f:
            json.dump(queries, f, indent = 2)

        print(f"Generated {len(events)} queries")

if __name__ == "__main__":
    graph = generate_graph(num_nodes = 50, num_edges = 150, filename = "test_graph.json")
    generate_queries(graph, num_queries = 30, filename = "test_queries.json")

    print("\nTest files generated successfully.")
    print("Run with: ./phase1 test_graph.json test_queries.json output.json")
>>>>>>> cab6b08fb4f3157ce7f2e451d180ed291cacdd4f
