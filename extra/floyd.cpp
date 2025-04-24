#include <iostream>
#include <vector>
#include <queue>
#include <random>
#include <chrono>
#include <thread>
#include <iomanip>
#include <string>
#include <fstream>
#include <sstream>
#include <limits>
#include <algorithm>

using namespace std;

// Constants
const int INF = numeric_limits<int>::max();
const int MAX_NODES = 100;
const int MAX_VEHICLES = 200;
const double TIME_STEP = 1.0; // seconds
const int SIMULATION_STEPS = 100;

// Random number generator
random_device rd;
mt19937 gen(rd());

// Structure to represent a road
struct Road {
    int from, to;
    int distance; // in meters
    int speed_limit; // in km/h
    int lanes;
    double congestion_factor; // 0 to 1, where 1 is heavily congested
    Road(int f, int t, int d, int s, int l)
        : from(f), to(t), distance(d), speed_limit(s), lanes(l), congestion_factor(0.0) {}
};

// Structure to represent a vehicle
struct Vehicle {
    int id;
    int current_node;
    int destination;
    vector<int> path; // Path from current to destination
    double progress; // Distance traveled on current road (meters)
    double speed; // Current speed (m/s)
    bool is_moving;
    Vehicle(int i, int start, int dest)
        : id(i), current_node(start), destination(dest), progress(0.0), speed(0.0), is_moving(true) {}
};

// Structure to represent a traffic light
struct TrafficLight {
    int node;
    bool is_green; // true for green, false for red
    double green_duration; // seconds
    double red_duration; // seconds
    double time_since_change; // seconds
    TrafficLight(int n, double g, double r)
        : node(n), is_green(true), green_duration(g), red_duration(r), time_since_change(0.0) {}
};

// Class to manage the traffic simulation
class TrafficSimulation {
private:
    int num_nodes;
    vector<vector<int> > adj_matrix; // Adjacency matrix for distances
    vector<vector<int> > next_node; // For path reconstruction
    vector<Road> roads;
    vector<Vehicle> vehicles;
    vector<TrafficLight> traffic_lights;
    double simulation_time;

public:
    TrafficSimulation(int nodes) : num_nodes(nodes), simulation_time(0.0) {
        // Initialize adjacency matrix
        adj_matrix.assign(num_nodes, vector<int>(num_nodes, INF));
        next_node.assign(num_nodes, vector<int>(num_nodes, -1));
        for (int i = 0; i < num_nodes; ++i) {
            adj_matrix[i][i] = 0;
            next_node[i][i] = i;
        }
    }

    // Add a road to the network
    void addRoad(int from, int to, int distance, int speed_limit, int lanes) {
        if (from >= num_nodes || to >= num_nodes || from < 0 || to < 0) {
            cout << "Invalid nodes for road.\n";
            return;
        }
        roads.emplace_back(from, to, distance, speed_limit, lanes);
        adj_matrix[from][to] = distance;
        next_node[from][to] = to;
    }

    // Add a vehicle to the simulation
    void addVehicle(int id, int start, int dest) {
        if (start >= num_nodes || dest >= num_nodes || start < 0 || dest < 0) {
            cout << "Invalid nodes for vehicle.\n";
            return;
        }
        vehicles.emplace_back(id, start, dest);
        computePath(vehicles.back());
    }

    // Add a traffic light
    void addTrafficLight(int node, double green_duration, double red_duration) {
        if (node >= num_nodes || node < 0) {
            cout << "Invalid node for traffic light.\n";
            return;
        }
        traffic_lights.emplace_back(node, green_duration, red_duration);
    }

    // Floyd-Warshall algorithm to compute shortest paths
    void floydWarshall() {
        // Initialize distances
        vector<vector<int> > dist = adj_matrix;

        // Compute shortest paths
        for (int k = 0; k < num_nodes; ++k) {
            for (int i = 0; i < num_nodes; ++i) {
                for (int j = 0; j < num_nodes; ++j) {
                    if (dist[i][k] != INF && dist[k][j] != INF &&
                        dist[i][j] > dist[i][k] + dist[k][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                        next_node[i][j] = next_node[i][k];
                    }
                }
            }
        }

        // Update adjacency matrix
        adj_matrix = dist;
    }

    // Compute path for a vehicle using next_node matrix
    void computePath(Vehicle& vehicle) {
        int start = vehicle.current_node;
        int dest = vehicle.destination;
        if (adj_matrix[start][dest] == INF) {
            cout << "No path exists for vehicle " << vehicle.id << " from " << start << " to " << dest << ".\n";
            vehicle.is_moving = false;
            return;
        }

        vehicle.path.clear();
        int current = start;
        while (current != dest) {
            vehicle.path.push_back(current);
            current = next_node[current][dest];
        }
        vehicle.path.push_back(dest);
    }

    // Update congestion on roads
    void updateCongestion() {
        vector<int> road_usage(roads.size(), 0);
        for (const auto& vehicle : vehicles) {
            if (!vehicle.is_moving || vehicle.path.size() < 2) continue;
            int current = vehicle.path[0];
            int next = vehicle.path[1];
            for (size_t i = 0; i < roads.size(); ++i) {
                if (roads[i].from == current && roads[i].to == next) {
                    road_usage[i]++;
                    break;
                }
            }
        }

        for (size_t i = 0; i < roads.size(); ++i) {
            double usage_ratio = static_cast<double>(road_usage[i]) / (roads[i].lanes * 10);
            roads[i].congestion_factor = min(1.0, max(0.0, usage_ratio));
        }
    }

    // Update traffic lights
    void updateTrafficLights(double dt) {
        for (auto& light : traffic_lights) {
            light.time_since_change += dt;
            if (light.time_since_change < 0) light.time_since_change = 0; // Ensure non-negative
            double cycle_time = light.green_duration + light.red_duration;
            double time_in_cycle = fmod(light.time_since_change, cycle_time);
            if (time_in_cycle < light.green_duration) {
                light.is_green = true;
            } else {
                light.is_green = false;
            }
        }
    }

    // Check if a vehicle can move based on traffic light
    bool canVehicleMove(const Vehicle& vehicle) {
        if (vehicle.path.size() < 2) return false;
        int next_node = vehicle.path[1];
        for (const auto& light : traffic_lights) {
            if (light.node == next_node && !light.is_green) {
                return false;
            }
        }
        return true;
    }

    // Update vehicle positions
    void updateVehicles(double dt) {
        for (auto& vehicle : vehicles) {
            if (!vehicle.is_moving || vehicle.path.size() < 2) continue;

            // Check traffic light
            if (!canVehicleMove(vehicle)) {
                vehicle.speed = 0.0;
                continue;
            }

            // Find current road
            int current = vehicle.path[0];
            int next = vehicle.path[1];
            Road* current_road = nullptr;
            for (auto& road : roads) {
                if (road.from == current && road.to == next) {
                    current_road = &road;
                    break;
                }
            }

            if (!current_road) continue;

            // Calculate speed considering congestion
            double max_speed = (current_road->speed_limit * 1000.0) / 3600.0; // Convert km/h to m/s
            double effective_speed = max_speed * (1.0 - current_road->congestion_factor);
            vehicle.speed = effective_speed;

            // Update progress
            vehicle.progress += vehicle.speed * dt;

            // Check if vehicle reached the next node
            if (vehicle.progress >= current_road->distance) {
                vehicle.progress = 0.0;
                vehicle.current_node = next;
                vehicle.path.erase(vehicle.path.begin());
                if (vehicle.path.size() == 1) {
                    // Vehicle reached destination
                    vehicle.is_moving = false;
                    cout << "Vehicle " << vehicle.id << " reached destination " << vehicle.destination << ".\n";
                }
            }
        }
    }

    // Run simulation for one time step
    void step(double dt) {
        simulation_time += dt;
        updateCongestion();
        updateTrafficLights(dt);
        updateVehicles(dt);
    }

    // Run the full simulation
    void runSimulation() {
        floydWarshall();
        for (int step_count = 0; step_count < SIMULATION_STEPS; ++step_count) {
            cout << "Time: " << fixed << setprecision(1) << simulation_time << "s\n";
            this->step(TIME_STEP); // Explicitly call the class method
            printStatus();
            this_thread::sleep_for(chrono::milliseconds(100)); // Simulate real-time
        }
    }

    // Print current status
    void printStatus() {
        cout << "Vehicles:\n";
        for (const auto& vehicle : vehicles) {
            if (vehicle.is_moving) {
                cout << "Vehicle " << vehicle.id << ": Node " << vehicle.current_node
                     << ", Progress: " << fixed << setprecision(1) << vehicle.progress
                     << "m, Speed: " << vehicle.speed * 3.6 << "km/h\n";
            }
        }
        cout << "Traffic Lights:\n";
        for (const auto& light : traffic_lights) {
            cout << "Node " << light.node << ": " << (light.is_green ? "Green" : "Red") << "\n";
        }
        cout << "Roads Congestion:\n";
        for (const auto& road : roads) {
            cout << "Road " << road.from << " -> " << road.to << ": Congestion "
                 << fixed << setprecision(2) << road.congestion_factor * 100 << "%\n";
        }
        cout << "------------------------\n";
    }

    // Load network from a file
    bool loadNetwork(const string& filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            cout << "Failed to open file: " << filename << "\n";
            return false;
        }

        string line;
        while (getline(file, line)) {
            istringstream iss(line);
            string type;
            if (!(iss >> type)) {
                cout << "Invalid line in file: " << line << "\n";
                continue;
            }
            if (type == "ROAD") {
                int from, to, distance, speed_limit, lanes;
                if (iss >> from >> to >> distance >> speed_limit >> lanes) {
                    addRoad(from, to, distance, speed_limit, lanes);
                } else {
                    cout << "Invalid ROAD entry: " << line << "\n";
                }
            } else if (type == "VEHICLE") {
                int id, start, dest;
                if (iss >> id >> start >> dest) {
                    addVehicle(id, start, dest);
                } else {
                    cout << "Invalid VEHICLE entry: " << line << "\n";
                }
            } else if (type == "TRAFFIC_LIGHT") {
                int node;
                double green, red;
                if (iss >> node >> green >> red) {
                    addTrafficLight(node, green, red);
                } else {
                    cout << "Invalid TRAFFIC_LIGHT entry: " << line << "\n";
                }
            } else {
                cout << "Unknown entry type: " << type << "\n";
            }
        }
        file.close();
        return true;
    }
};

// Main function to set up and run the simulation
int main() {
    // Initialize simulation with 10 nodes
    TrafficSimulation sim(10);

    // Option to load from file or define programmatically
    bool use_file = false;
    if (use_file) {
        sim.loadNetwork("network.txt");
    } else {
        // Define a sample network
        // Roads: (from, to, distance (m), speed_limit (km/h), lanes)
        sim.addRoad(0, 1, 500, 60, 2);
        sim.addRoad(1, 2, 600, 60, 2);
        sim.addRoad(2, 3, 400, 50, 1);
        sim.addRoad(0, 4, 700, 70, 3);
        sim.addRoad(4, 5, 800, 70, 3);
        sim.addRoad(5, 3, 300, 50, 1);
        sim.addRoad(1, 4, 200, 40, 1);
        sim.addRoad(2, 5, 250, 40, 1);

        // Vehicles: (id, start_node, destination_node)
        sim.addVehicle(1, 0, 3);
        sim.addVehicle(2, 1, 5);
        sim.addVehicle(3, 4, 2);

        // Traffic lights: (node, green_duration (s), red_duration (s))
        sim.addTrafficLight(1, 30.0, 30.0);
        sim.addTrafficLight(4, 20.0, 40.0);
    }

    // Run the simulation
    cout << "Starting Traffic Simulation...\n";
    sim.runSimulation();
    cout << "Simulation Completed.\n";

    return 0;
}