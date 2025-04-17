#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <limits>
#include <set>
#include <cstdlib>
#include <ctime>
#include <thread>
#include <chrono>
#include <algorithm>
#include <fstream>

using namespace std;

const int INF = numeric_limits<int>::max();

enum LightState { RED, GREEN, YELLOW };

class TrafficLight {
    LightState state;
    int timer;
    int redDuration;
    int greenDuration;
    int yellowDuration;

public:
    TrafficLight(int red = 5, int green = 5, int yellow = 2)
        : redDuration(red), greenDuration(green), yellowDuration(yellow) {
        state = RED;
        timer = redDuration;
    }

    void update() {
        timer--;
        if (timer <= 0) {
            switch (state) {
                case RED:
                    state = GREEN;
                    timer = greenDuration;
                    break;
                case GREEN:
                    state = YELLOW;
                    timer = yellowDuration;
                    break;
                case YELLOW:
                    state = RED;
                    timer = redDuration;
                    break;
            }
        }
    }

    LightState getState() const { return state; }

    void display() const {
        string status = (state == RED ? "RED" : (state == GREEN ? "GREEN" : "YELLOW"));
        cout << "[Traffic Light] State: " << status << ", Timer: " << timer << endl;
    }
};

struct Edge {
    int to;
    int weight;
    bool blocked;

    Edge(int t, int w, bool b = false) : to(t), weight(w), blocked(b) {}
};

class Graph {
    unordered_map<int, vector<Edge>> adjList;

public:
    void addEdge(int u, int v, int w) {
        adjList[u].emplace_back(v, w);
        adjList[v].emplace_back(u, w);
    }

    void blockEdge(int u, int v) {
        for (auto& e : adjList[u]) {
            if (e.to == v) e.blocked = true;
        }
        for (auto& e : adjList[v]) {
            if (e.to == u) e.blocked = true;
        }
    }

    vector<int> dijkstra(int src, int dest) {
        unordered_map<int, int> dist;
        unordered_map<int, int> prev;
        set<pair<int, int>> pq;

        for (auto& node : adjList) {
            dist[node.first] = INF;
        }
        dist[src] = 0;
        pq.insert({0, src});

        while (!pq.empty()) {
            int u = pq.begin()->second;
            pq.erase(pq.begin());
            if (u == dest) break;

            for (auto& edge : adjList[u]) {
                if (edge.blocked) continue;
                int v = edge.to;
                int weight = edge.weight;
                if (dist[u] + weight < dist[v]) {
                    pq.erase({dist[v], v});
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                    pq.insert({dist[v], v});
                }
            }
        }

        vector<int> path;
        for (int at = dest; prev.find(at) != prev.end(); at = prev[at]) {
            path.push_back(at);
        }
        if (!path.empty() || src == dest) path.push_back(src);
        reverse(path.begin(), path.end());
        return path;
    }
};

class Logger {
public:
    static void log(const string& message) {
        ofstream file("traffic_log.txt", ios::app);
        if (file.is_open()) {
            time_t now = time(0);
            string dt = ctime(&now);
            dt.pop_back();
            file << "[" << dt << "] " << message << endl;
            file.close();
        }
    }
};

class Vehicle {
public:
    int id;
    int currentPos;
    int destination;
    int speed;
    vector<int> path;
    size_t currentStep;
    bool arrived;
    bool isWaiting;
    bool inQueue;

    Vehicle(int i, int src, int dest, Graph& graph)
        : id(i), currentPos(src), destination(dest), speed(1),
          currentStep(0), arrived(false), isWaiting(false), inQueue(false) {
        path = graph.dijkstra(src, dest);
        if (path.empty() && src != dest) {
            cout << "No path from " << src << " to " << dest << endl;
            arrived = true;
        }
    }

    void move(Graph& graph) {
        if (arrived || currentStep >= path.size() - 1) {
            arrived = true;
            return;
        }
        currentPos = path[currentStep + 1];
        currentStep++;
        isWaiting = false;
        inQueue = false;
        Logger::log("Vehicle " + to_string(id) + " moved to node " + to_string(currentPos));
        cout << "Vehicle " << id << " moved to node " << currentPos << endl;
        if (currentPos == destination) {
            arrived = true;
            cout << "Vehicle " << id << " has reached its destination.\n";
        }
    }

    void reroute(Graph& graph) {
        path = graph.dijkstra(currentPos, destination);
        currentStep = 0;
        cout << "Vehicle " << id << " rerouted.\n";
    }
};

class LaneQueue {
    queue<int> vehicleQueue;
    int maxLength;

public:
    LaneQueue(int maxL = 10) : maxLength(maxL) {}

    bool canEnter() const { return vehicleQueue.size() < maxLength; }

    void addVehicle(int vehicleID) {
        if (canEnter()) vehicleQueue.push(vehicleID);
    }

    void releaseVehicle() {
        if (!vehicleQueue.empty()) vehicleQueue.pop();
    }

    int queueSize() const { return vehicleQueue.size(); }

    int frontVehicle() const {
        if (!vehicleQueue.empty()) return vehicleQueue.front();
        return -1;
    }
};

class Intersection {
public:
    int id;
    TrafficLight light;
    LaneQueue queue;

    Intersection(int _id) : id(_id), light(), queue() {}

    void updateLight() { light.update(); }

    LightState getLightState() const { return light.getState(); }

    void displayLight() {
        cout << "[Intersection " << id << "] ";
        light.display();
    }
};

class SimulationManager {
    Graph graph;
    unordered_map<int, Intersection> intersections;
    vector<Vehicle> vehicles;
    int timeStep;
    int totalVehicles;
    bool simulationRunning;

public:
    SimulationManager() : timeStep(0), totalVehicles(0), simulationRunning(true) {
        srand(time(0));
    }

    void setupRoadNetwork() {
        for (int i = 1; i <= 6; ++i) {
            intersections.emplace(i, Intersection(i));
        }
        graph.addEdge(1, 2, 5);
        graph.addEdge(2, 3, 3);
        graph.addEdge(3, 4, 2);
        graph.addEdge(4, 5, 4);
        graph.addEdge(5, 6, 6);
        graph.addEdge(1, 6, 10);
        graph.addEdge(2, 5, 8);
    }

    void addVehicle(int src, int dest) {
        if (intersections.find(src) == intersections.end() ||
            intersections.find(dest) == intersections.end()) {
            cout << "Invalid source or destination.\n";
            return;
        }
        Vehicle v(totalVehicles++, src, dest, graph);
        vehicles.push_back(v);
    }

    void simulateAccident() {
        int u = rand() % 6 + 1;
        int v = rand() % 6 + 1;
        while (v == u) v = rand() % 6 + 1;
        cout << "\n🚨 Accident occurred between " << u << " and " << v << "!\n";
        graph.blockEdge(u, v);
        Logger::log("Accident simulated between " + to_string(u) + " and " + to_string(v));
        for (auto& v : vehicles) {
            if (!v.arrived) v.reroute(graph);
        }
    }

    void updateTrafficLights() {
        for (auto& pair : intersections) {
            pair.second.updateLight();
        }
    }

    void moveVehicles() {
        for (auto& pair : intersections) {
            Intersection& inter = pair.second;
            if (inter.getLightState() == GREEN && inter.queue.queueSize() > 0) {
                int vehicleID = inter.queue.frontVehicle();
                inter.queue.releaseVehicle();
                auto it = find_if(vehicles.begin(), vehicles.end(),
                                 [vehicleID](const Vehicle& v) { return v.id == vehicleID; });
                if (it != vehicles.end()) {
                    it->move(graph);
                }
            }
        }
        for (auto& v : vehicles) {
            if (!v.arrived && !v.inQueue) {
                try {
                    Intersection& inter = intersections.at(v.currentPos);
                    if (inter.queue.canEnter()) {
                        inter.queue.addVehicle(v.id);
                        v.inQueue = true;
                        cout << "Vehicle " << v.id << " entered queue at node " << v.currentPos << endl;
                    } else {
                        v.isWaiting = true;
                        cout << "Vehicle " << v.id << " waiting to enter queue at node " << v.currentPos << endl;
                    }
                } catch (const std::out_of_range& e) {
                    cout << "Error: Intersection " << v.currentPos << " not found for vehicle " << v.id << endl;
                    v.arrived = true; // Mark vehicle as arrived to prevent further processing
                }
            }
        }
    }

    void displayStatus() {
        cout << "\n=== Simulation Time Step: " << timeStep << " ===\n";
        for (auto& v : vehicles) {
            cout << "Vehicle " << v.id << " at node " << v.currentPos;
            if (v.arrived) cout << " (Arrived)";
            cout << endl;
        }
        for (auto& p : intersections) {
            p.second.displayLight();
        }
        cout << "===============================\n";
    }

    void reportStats() {
        int totalArrived = 0;
        int totalWaiting = 0;
        for (auto& v : vehicles) {
            if (v.arrived) totalArrived++;
            if (v.isWaiting) totalWaiting++;
        }
        cout << "\n📈 Traffic Report:\n";
        cout << "Total Vehicles: " << vehicles.size() << endl;
        cout << "Arrived: " << totalArrived << endl;
        cout << "Still Waiting: " << totalWaiting << endl;
    }

    void runSimulation(int maxSteps = 30) {
        setupRoadNetwork();
        addVehicle(1, 4);
        addVehicle(2, 5);
        addVehicle(3, 6);

        while (simulationRunning && timeStep < maxSteps) {
            updateTrafficLights();
            moveVehicles();
            if (timeStep == 10 || timeStep == 20) {
                simulateAccident();
            }
            displayStatus();
            if (timeStep % 5 == 0) {
                reportStats();
            }
            this_thread::sleep_for(chrono::milliseconds(1000));
            timeStep++;
        }
        cout << "\nSimulation complete.\n";
    }
};

int main() {
    SimulationManager sim;
    sim.runSimulation();
    return 0;
}