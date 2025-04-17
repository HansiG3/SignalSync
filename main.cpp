#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <ctime>
#include <cstdlib>
#include <string>
#include <thread>
#include <chrono>

using namespace std;

// Enum for traffic light states with durations
enum LightState { RED = 4, GREEN = 5, YELLOW = 2 };

// Logger class for event logging
class Logger {
public:
    // Static method to log messages to a file with timestamp
    static void log(const string& message) {
        ofstream file("traffic_log.txt", ios::app);
        if (file.is_open()) {
            time_t now = time(0);
            string dt = ctime(&now);
            dt.pop_back(); // Remove newline
            file << "[" << dt << "] " << message << endl;
            file.close();
        }
    }
};

// TrafficLight class managing light states and timers
class TrafficLight {
public:
    LightState state;      // Current state (RED, GREEN, YELLOW)
    int timer;             // Time until state change
    int extendedGreenTime; // Extended green time for busy intersections
    int minGreenTime;      // Minimum green time
    int maxGreenTime;      // Maximum green time
    LightState prevState;  // Previous state for change detection

    // Constructor initializing light
    TrafficLight() : state(RED), timer(RED), extendedGreenTime(7), 
                     minGreenTime(3), maxGreenTime(10), prevState(RED) {}

    // Update light state based on queue size and traffic density
    bool update(int queueSize, double trafficDensity) {
        // Store current state for change detection
        prevState = state;

        // Adjust green time dynamically based on queue size and density
        int currentGreenTime = GREEN;
        if (queueSize > 3 || trafficDensity > 0.5) {
            currentGreenTime = min(extendedGreenTime + queueSize / 2, maxGreenTime);
        } else {
            currentGreenTime = max(static_cast<int>(GREEN), minGreenTime);
        }

        timer--;
        if (timer <= 0) {
            if (state == RED) {
                state = GREEN;
                timer = currentGreenTime;
            } else if (state == GREEN) {
                state = YELLOW;
                timer = YELLOW;
            } else {
                state = RED;
                timer = RED;
            }
        }

        // Return true if state changed
        return state != prevState;
    }

    // Convert enum state to string for output
    string getStateString() const {
        switch (state) {
            case RED: return "RED";
            case GREEN: return "GREEN";
            case YELLOW: return "YELLOW";
            default: return "UNKNOWN";
        }
    }

    // Log light state change
    void logStateChange(int intersectionId) {
        Logger::log("Intersection " + to_string(intersectionId) + " light changed to " + getStateString());
    }
};

// Road class representing a connection between intersections
class Road {
public:
    int start;     // Starting intersection ID
    int end;       // Ending intersection ID
    int weight;    // Distance or travel time
    bool blocked;  // True if road is blocked (e.g., by accident)

    // Constructor for a road
    Road(int s, int e, int w) : start(s), end(e), weight(w), blocked(false) {}

    // Display road details for debugging
    void display() const {
        cout << "Road from " << start << " to " << end << ", Weight: " << weight
             << ", Blocked: " << (blocked ? "Yes" : "No") << endl;
    }
};

// Intersection class representing a node with a light and queue
class Intersection {
public:
    int id;                // Unique ID
    TrafficLight light;    // Traffic light
    vector<int> queue;     // Vehicle IDs waiting
    int maxQueueSize;      // Maximum queue capacity
    int totalWaitTime;     // Sum of wait times for congestion analysis
    int vehiclesProcessed;  // Number of vehicles passed through

    // Constructor
    Intersection(int i) : id(i), maxQueueSize(5), totalWaitTime(0), vehiclesProcessed(0) {}

    // Add vehicle to queue
    bool addToQueue(int vehicleId) {
        if (queue.size() < maxQueueSize) {
            queue.push_back(vehicleId);
            return true;
        }
        return false;
    }

    // Remove first vehicle from queue
    int releaseVehicle() {
        if (queue.empty()) return -1;
        int vehicleId = queue[0];
        queue.erase(queue.begin());
        vehiclesProcessed++;
        return vehicleId;
    }

    // Update wait time for congestion analysis
    void updateWaitTime() {
        totalWaitTime += queue.size();
    }

    // Get average wait time
    double getAverageWaitTime() const {
        return vehiclesProcessed > 0 ? (double)totalWaitTime / vehiclesProcessed : 0.0;
    }
};

// Network class managing intersections and roads
class Network {
public:
    vector<Road> roads;          // List of all roads
    vector<Intersection> intersections; // List of all intersections

    // Add a new intersection
    void addIntersection(int id) {
        intersections.push_back(Intersection(id));
    }

    // Add a two-way road
    void addRoad(int start, int end, int weight) {
        roads.push_back(Road(start, end, weight));
    }

    // Block a road due to an accident
    void blockRoad(int start, int end) {
        for (auto& road : roads) {
            if ((road.start == start && road.end == end) || (road.start == end && road.end == start)) {
                road.blocked = true;
            }
        }
    }

    // Find shortest path using Dijkstra’s algorithm
    vector<int> findPath(int start, int end) {
        vector<int> dist(intersections.size() + 1, 999999);
        vector<int> prev(intersections.size() + 1, -1);
        vector<bool> visited(intersections.size() + 1, false);
        dist[start] = 0;

        for (size_t i = 0; i < intersections.size(); ++i) {
            int minDist = 999999, minNode = -1;
            for (size_t j = 1; j <= intersections.size(); ++j) {
                if (!visited[j] && dist[j] < minDist) {
                    minDist = dist[j];
                    minNode = j;
                }
            }
            if (minNode == -1) break;

            visited[minNode] = true;
            if (minNode == end) break;

            for (const auto& road : roads) {
                if (road.blocked) continue;
                if (road.start == minNode) {
                    int next = road.end;
                    if (dist[minNode] + road.weight < dist[next]) {
                        dist[next] = dist[minNode] + road.weight;
                        prev[next] = minNode;
                    }
                } else if (road.end == minNode) {
                    int next = road.start;
                    if (dist[minNode] + road.weight < dist[next]) {
                        dist[next] = dist[minNode] + road.weight;
                        prev[next] = minNode;
                    }
                }
            }
        }

        vector<int> path;
        int current = end;
        while (current != -1) {
            path.push_back(current);
            current = prev[current];
        }
        reverse(path.begin(), path.end());
        if (path[0] != start) return {};
        return path;
    }

    // Calculate traffic density (average queue length)
    double calculateTrafficDensity() const {
        double totalQueueSize = 0;
        for (const auto& inter : intersections) {
            totalQueueSize += inter.queue.size();
        }
        return intersections.empty() ? 0.0 : totalQueueSize / intersections.size();
    }

    // Display network status for debugging
    void displayNetwork() const {
        cout << "Network Status:\n";
        for (const auto& road : roads) {
            road.display();
        }
        for (const auto& inter : intersections) {
            cout << "Intersection " << inter.id << ": Queue Size = " << inter.queue.size() << endl;
        }
    }
};

// Base Vehicle class
class Vehicle {
public:
    int id;           // Unique vehicle ID
    int current;      // Current intersection
    int destination;  // Target intersection
    vector<int> path; // Planned path
    bool arrived;     // True if at destination
    bool inQueue;     // True if in a queue
    int waitTime;     // Time spent waiting

    // Constructor
    Vehicle(int i, int start, int dest, Network& network)
        : id(i), current(start), destination(dest), arrived(false), inQueue(false), waitTime(0) {
        path = network.findPath(start, dest);
        if (path.empty()) arrived = true;
    }

    // Move vehicle to next intersection
    virtual void move(Network& network) {
        if (arrived || path.empty()) return;
        int nextIndex = -1;
        for (size_t i = 0; i < path.size(); ++i) {
            if (path[i] == current) {
                nextIndex = i + 1;
                break;
            }
        }
        if (nextIndex >= (int)path.size()) {
            arrived = true;
            return;
        }
        current = path[nextIndex];
        inQueue = false;
        waitTime = 0;
        cout << "Vehicle " << id << " moved to intersection " << current << endl;
        Logger::log("Vehicle " + to_string(id) + " moved to " + to_string(current));
        if (current == destination) {
            arrived = true;
            cout << "Vehicle " << id << " reached destination!" << endl;
            Logger::log("Vehicle " + to_string(id) + " reached destination");
        }
    }

    // Reroute vehicle if path is blocked
    virtual void reroute(Network& network) {
        path = network.findPath(current, destination);
        if (path.empty()) arrived = true;
        cout << "Vehicle " << id << " rerouted.\n";
        Logger::log("Vehicle " + to_string(id) + " rerouted");
    }

    // Update wait time
    void incrementWaitTime() {
        if (inQueue) waitTime++;
    }

    // Check if vehicle is emergency (for polymorphism)
    virtual bool isEmergency() const { return false; }

    // Virtual destructor for polymorphism
    virtual ~Vehicle() {}
};

// EmergencyVehicle class inheriting from Vehicle
class EmergencyVehicle : public Vehicle {
public:
    // Constructor
    EmergencyVehicle(int i, int start, int dest, Network& network)
        : Vehicle(i, start, dest, network) {}

    // Override move to ignore light state
    void move(Network& network) override {
        if (arrived || path.empty()) return;
        int nextIndex = -1;
        for (size_t i = 0; i < path.size(); ++i) {
            if (path[i] == current) {
                nextIndex = i + 1;
                break;
            }
        }
        if (nextIndex >= (int)path.size()) {
            arrived = true;
            return;
        }
        current = path[nextIndex];
        inQueue = false;
        waitTime = 0;
        cout << "Emergency Vehicle " << id << " moved to intersection " << current << endl;
        Logger::log("Emergency Vehicle " + to_string(id) + " moved to " + to_string(current));
        if (current == destination) {
            arrived = true;
            cout << "Emergency Vehicle " << id << " reached destination!" << endl;
            Logger::log("Emergency Vehicle " + to_string(id) + " reached destination");
        }
    }

    // Emergency vehicles have priority
    bool isEmergency() const override { return true; }
};

// Simulation class managing the entire system
class Simulation {
public:
    Network network;           // Road network
    vector<Vehicle*> vehicles; // Polymorphic vector for Vehicle and EmergencyVehicle
    int time;                  // Current time step
    int totalVehicles;         // Total vehicles created

    // Constructor
    Simulation() : time(0), totalVehicles(0) {}

    // Setup the expanded road network and vehicles
    void setup() {
        // Add 10 intersections
        for (int i = 1; i <= 10; ++i) {
            network.addIntersection(i);
        }

        // Add 20 roads (grid-like with extra connections)
        network.addRoad(1, 2, 5);
        network.addRoad(2, 3, 3);
        network.addRoad(3, 4, 4);
        network.addRoad(5, 6, 6);
        network.addRoad(6, 7, 3);
        network.addRoad(7, 8, 5);
        network.addRoad(9, 10, 4);
        network.addRoad(1, 5, 7);
        network.addRoad(2, 6, 5);
        network.addRoad(3, 7, 6);
        network.addRoad(4, 8, 4);
        network.addRoad(5, 9, 5);
        network.addRoad(6, 10, 6);
        network.addRoad(1, 6, 8);
        network.addRoad(2, 7, 7);
        network.addRoad(3, 8, 5);
        network.addRoad(5, 10, 6);
        network.addRoad(6, 9, 4);
        network.addRoad(4, 7, 5);
        network.addRoad(2, 5, 6);

        // Add 10 vehicles (7 regular, 3 emergency)
        vehicles.push_back(new Vehicle(totalVehicles++, 1, 4, network));
        vehicles.push_back(new Vehicle(totalVehicles++, 2, 8, network));
        vehicles.push_back(new Vehicle(totalVehicles++, 3, 10, network));
        vehicles.push_back(new Vehicle(totalVehicles++, 5, 7, network));
        vehicles.push_back(new Vehicle(totalVehicles++, 6, 9, network));
        vehicles.push_back(new Vehicle(totalVehicles++, 7, 1, network));
        vehicles.push_back(new Vehicle(totalVehicles++, 8, 2, network));
        vehicles.push_back(new EmergencyVehicle(totalVehicles++, 9, 3, network));
        vehicles.push_back(new EmergencyVehicle(totalVehicles++, 10, 5, network));
        vehicles.push_back(new EmergencyVehicle(totalVehicles++, 4, 6, network));
    }

    // Simulate a random accident
    void simulateAccident() {
        int start = rand() % 10 + 1;
        int end = rand() % 10 + 1;
        while (end == start) end = rand() % 10 + 1;
        network.blockRoad(start, end);
        cout << "Accident blocked road between " << start << " and " << end << endl;
        Logger::log("Accident between " + to_string(start) + " and " + to_string(end));
        for (auto& vehicle : vehicles) {
            if (!vehicle->arrived) vehicle->reroute(network);
        }
    }

    // Add a new vehicle based on user input
    void addUserVehicle(int start, int dest) {
        if (start >= 1 && start <= 10 && dest >= 1 && dest <= 10) {
            vehicles.push_back(new Vehicle(totalVehicles++, start, dest, network));
            cout << "Added Vehicle " << totalVehicles - 1 << " from " << start << " to " << dest << endl;
            Logger::log("Added Vehicle " + to_string(totalVehicles - 1) + " from " + to_string(start) + " to " + to_string(dest));
        } else {
            cout << "Invalid start or destination.\n";
        }
    }

    // Process user input for interactive features
    void processUserInput() {
        cout << "Enter command (accident/add_vehicle/exit): ";
        string command;
        cin.clear();
        getline(cin, command); // Use getline to handle input robustly
        if (command == "accident") {
            simulateAccident();
        } else if (command == "add_vehicle") {
            int start, dest;
            cout << "Enter start intersection (1-10): ";
            cin >> start;
            cout << "Enter destination intersection (1-10): ";
            cin >> dest;
            cin.ignore(); // Clear newline
            addUserVehicle(start, dest);
        } else if (command == "exit") {
            time = 100; // Force simulation to end
        }
    }

    // Update traffic lights
    void updateTrafficLights() {
        double trafficDensity = network.calculateTrafficDensity();
        for (auto& inter : network.intersections) {
            if (inter.light.update(inter.queue.size(), trafficDensity)) {
                inter.light.logStateChange(inter.id);
            }
            inter.updateWaitTime();
        }
    }

    // Process vehicle queues
    void processQueues() {
        for (auto& inter : network.intersections) {
            // Skip if queue is empty
            if (inter.queue.empty()) continue;

            // Prioritize emergency vehicles
            for (size_t i = 0; i < inter.queue.size(); ++i) {
                for (auto& vehicle : vehicles) {
                    if (vehicle->id == inter.queue[i] && vehicle->isEmergency()) {
                        int vehicleId = inter.queue[i];
                        inter.queue.erase(inter.queue.begin() + i);
                        vehicle->move(network);
                        Logger::log("Emergency Vehicle " + to_string(vehicleId) + " moved to " + to_string(vehicle->current));
                        break;
                    }
                }
            }

            // Regular vehicles move on GREEN
            if (inter.light.state == GREEN && !inter.queue.empty()) {
                int vehicleId = inter.releaseVehicle();
                if (vehicleId != -1) {
                    for (auto& vehicle : vehicles) {
                        if (vehicle->id == vehicleId && !vehicle->arrived) {
                            vehicle->move(network);
                            break;
                        }
                    }
                }
            }
        }
    }

    // Add vehicles to queues
    void addVehiclesToQueues() {
        for (auto& vehicle : vehicles) {
            if (!vehicle->arrived && !vehicle->inQueue) {
                for (auto& inter : network.intersections) {
                    if (inter.id == vehicle->current && inter.addToQueue(vehicle->id)) {
                        vehicle->inQueue = true;
                        cout << "Vehicle " << vehicle->id << " joined queue at intersection " << inter.id << endl;
                        Logger::log("Vehicle " + to_string(vehicle->id) + " joined queue at " + to_string(inter.id));
                        break;
                    }
                }
            }
            vehicle->incrementWaitTime();
        }
    }

    // Report congestion and traffic stats
    void reportStats() {
        cout << "\nTraffic Report:\n";
        int arrived = 0, waiting = 0;
        double totalWaitTime = 0;
        for (const auto& vehicle : vehicles) {
            if (vehicle->arrived) arrived++;
            if (vehicle->inQueue) waiting++;
            totalWaitTime += vehicle->waitTime;
        }
        cout << "Total Vehicles: " << vehicles.size() << endl;
        cout << "Arrived: " << arrived << endl;
        cout << "Waiting in Queues: " << waiting << endl;
        cout << "Average Wait Time: " << (vehicles.empty() ? 0 : totalWaitTime / vehicles.size()) << " steps\n";
        cout << "Traffic Density: " << network.calculateTrafficDensity() << " vehicles/intersection\n";

        // Congestion analysis per intersection
        cout << "Intersection Congestion:\n";
        for (const auto& inter : network.intersections) {
            cout << "Intersection " << inter.id << ": Avg Wait Time = " << inter.getAverageWaitTime()
                 << ", Queue Size = " << inter.queue.size() << endl;
        }
    }

    // Main simulation loop
    void run(int maxSteps) {
        setup();
        while (time < maxSteps) {
            cout << "\nTime Step " << time << ":\n";

            // Update traffic lights
            updateTrafficLights();

            // Display intersection status
            for (auto& inter : network.intersections) {
                cout << "Intersection " << inter.id << ": Light = " << inter.light.getStateString()
                     << ", Timer = " << inter.light.timer << ", Queue = " << inter.queue.size() << endl;
            }

            // Process queues and move vehicles
            processQueues();
            addVehiclesToQueues();

            // Random accident with 10% chance per step
            if (rand() % 100 < 10) {
                simulateAccident();
            }

            // Display vehicle status
            cout << "Vehicle Status:\n";
            for (const auto& vehicle : vehicles) {
                cout << "Vehicle " << vehicle->id << (vehicle->isEmergency() ? " (Emergency)" : "")
                     << ": At " << vehicle->current << (vehicle->arrived ? " (Arrived)" : "")
                     << ", Wait Time = " << vehicle->waitTime << endl;
            }

            // Report stats every 5 steps
            if (time % 5 == 0) {
                reportStats();
            }

            // Process user input
            processUserInput();

            // Pause for readability
            this_thread::sleep_for(chrono::milliseconds(1000));
            time++;
        }

        // Clean up dynamically allocated vehicles
        for (auto vehicle : vehicles) {
            delete vehicle;
        }
        vehicles.clear();

        cout << "Simulation ended.\n";
    }

    // Destructor to ensure cleanup
    ~Simulation() {
        for (auto vehicle : vehicles) {
            delete vehicle;
        }
        vehicles.clear();
    }
};

// Main function
int main() {
    srand(time(0));
    Simulation sim;
    sim.run(20); // Run for 20 steps
    return 0;
}