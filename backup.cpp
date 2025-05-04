// Terminal-Based Traffic Simulation System
// Main components:
// - Graph representation of road network
// - Traffic light system`
// - Vehicle movement and path finding
// - Accident simulation
// - Terminal visualization

#include <iostream>
#include <vector>
#include <queue>
#include <list>
#include <limits>
#include <algorithm>
#include <random>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>
#include <map>

// Forward declarations
class Vehicle;
class TrafficLight;
class Node;
class Edge;
class Graph;
class Simulator;
class TerminalDisplay;

// Constants
const int INFINITY_VALUE = std::numeric_limits<int>::max();
const int RED_LIGHT_DURATION = 15;    // seconds
const int GREEN_LIGHT_DURATION = 20;  // seconds
const int YELLOW_LIGHT_DURATION = 5;  // seconds
const int ACCIDENT_DURATION = 10;     // seconds
const int TOTAL_SIMULATION_TIME = 60; // seconds (limited to 60)

// Enums
enum class TrafficLightState
{
    RED,
    GREEN,
    YELLOW
};

enum class VehicleState
{
    MOVING,
    WAITING_AT_LIGHT,
    REACHED_DESTINATION,
    INVOLVED_IN_ACCIDENT
};

// Node class represents an intersection in the road network
class Node
{
private:
    int id;
    float x, y; // Coordinates for visualization
    TrafficLight *trafficLight;
    std::vector<Vehicle *> waitingVehicles;

public:
    Node(int nodeId, float posX, float posY);
    ~Node();

    int getId() const { return id; }
    float getX() const { return x; }
    float getY() const { return y; }

    TrafficLight *getTrafficLight() const { return trafficLight; }
    void addWaitingVehicle(Vehicle *vehicle);
    void removeWaitingVehicle(Vehicle *vehicle);
    int getWaitingVehiclesCount() const { return waitingVehicles.size(); }

    std::vector<Vehicle *> getWaitingVehicles() const { return waitingVehicles; }
    std::string getStatusString() const;
};

// Edge class represents a road between two intersections
class Edge
{
private:
    int id;
    Node *source;
    Node *destination;
    int weight;           // Weight represents congestion/time to traverse
    bool blocked;         // Represents an accident or blockage
    int blockageTimeLeft; // Time until the blockage is cleared
    std::vector<Vehicle *> vehiclesOnRoad;

public:
    Edge(int edgeId, Node *src, Node *dest, int w);

    int getId() const { return id; }
    Node *getSource() const { return source; }
    Node *getDestination() const { return destination; }
    int getWeight() const { return weight; }
    bool isBlocked() const { return blocked; }

    void addVehicle(Vehicle *vehicle);
    void removeVehicle(Vehicle *vehicle);
    int getVehicleCount() const { return vehiclesOnRoad.size(); }
    std::vector<Vehicle *> getVehicles() const { return vehiclesOnRoad; }

    void createAccident(int duration);
    void updateBlockageTime(int timeElapsed);
    void updateCongestion();

    std::string getStatusString() const;
};

// TrafficLight class for managing intersection traffic
class TrafficLight
{
private:
    TrafficLightState currentState;
    int timeInCurrentState;
    Node *parentNode;

    // Direction mappings for traffic flow control
    std::vector<std::pair<Edge *, Edge *>> flowDirections;

public:
    TrafficLight(Node *node);

    TrafficLightState getState() const { return currentState; }
    void setState(TrafficLightState state);
    void update(int timeElapsed);
    bool canPass(Edge *fromEdge, Edge *toEdge) const;

    std::string getStateString() const;
};

// Vehicle class for representing cars in the simulation
class Vehicle
{
private:
    int id;
    Node *source;
    Node *destination;
    Node *currentNode;
    Edge *currentEdge;
    float progress; // Progress along the current edge (0.0 to 1.0)
    float speed;    // Speed factor affecting how fast the vehicle moves
    VehicleState state;
    std::vector<Edge *> path; // Calculated path from source to destination
    float x, y;               // Current position for visualization

public:
    Vehicle(int vehicleId, Node *src, Node *dest, Graph *graph);

    int getId() const { return id; }
    Node *getSource() const { return source; }
    Node *getDestination() const { return destination; }
    Node *getCurrentNode() const { return currentNode; }
    Edge *getCurrentEdge() const { return currentEdge; }
    VehicleState getState() const { return state; }
    float getProgress() const { return progress; }

    void setState(VehicleState newState) { state = newState; }
    void setCurrentNode(Node *node) { currentNode = node; }
    void setCurrentEdge(Edge *edge) { currentEdge = edge; }

    void calculatePath(Graph *graph);
    void update(int timeElapsed, Graph *graph);
    void move(int timeElapsed);
    void replanPath(Graph *graph);

    std::string getStatusString() const;
};

// Graph class represents the entire road network
class Graph
{
private:
    std::vector<Node *> nodes;
    std::vector<Edge *> edges;
    std::vector<std::vector<Edge *>> adjacencyList;

public:
    Graph();
    ~Graph();

    void addNode(Node *node);
    void addEdge(Edge *edge);

    Node *getNodeById(int id) const;
    Edge *getEdge(Node *src, Node *dest) const;
    std::vector<Edge *> getOutgoingEdges(Node *node) const;

    int getNodeCount() const { return nodes.size(); }
    int getEdgeCount() const { return edges.size(); }
    std::vector<Node *> getNodes() const { return nodes; }
    std::vector<Edge *> getEdges() const { return edges; }

    std::vector<Edge *> findShortestPath(Node *source, Node *destination);

    std::string getNetworkStatus() const;
};

// Terminal display class to handle console output
class TerminalDisplay
{
private:
    Simulator *simulator;

    // Helper functions to display simulation status
    void displayGraph();
    void displayVehicles();
    void displayStatus(int timeStep);

public:
    TerminalDisplay(Simulator *sim);

    void initialize();
    void render(int timeStep);
    void summary();
};

// Simulator class controls the entire simulation
class Simulator
{
private:
    Graph *roadNetwork;
    std::vector<Vehicle *> vehicles;
    int time;
    int accidentProbability; // Chance of an accident happening (0-100)
    std::mt19937 rng;        // Random number generator
    int lastAccidentTime;    // Track when the last accident occurred

    void checkForAccidents();
    void generateVehicles(int count);

public:
    Simulator();
    ~Simulator();

    void initialize(int nodeCount, int edgeCount);
    void step(int timeElapsed);
    void addVehicle(Node *source, Node *destination);

    Graph *getRoadNetwork() const { return roadNetwork; }
    std::vector<Vehicle *> getVehicles() const { return vehicles; }
    int getTime() const { return time; }

    void run(int simulationTime);
};

// Helper function to convert TrafficLightState to string
std::string trafficLightStateToString(TrafficLightState state)
{
    switch (state)
    {
    case TrafficLightState::RED:
        return "RED";
    case TrafficLightState::GREEN:
        return "GREEN";
    case TrafficLightState::YELLOW:
        return "YELLOW";
    default:
        return "UNKNOWN";
    }
}

// Helper function to convert VehicleState to string
std::string vehicleStateToString(VehicleState state)
{
    switch (state)
    {
    case VehicleState::MOVING:
        return "MOVING";
    case VehicleState::WAITING_AT_LIGHT:
        return "WAITING";
    case VehicleState::REACHED_DESTINATION:
        return "REACHED";
    case VehicleState::INVOLVED_IN_ACCIDENT:
        return "ACCIDENT";
    default:
        return "UNKNOWN";
    }
}

// Implementation of Node class
Node::Node(int nodeId, float posX, float posY)
    : id(nodeId), x(posX), y(posY)
{
    trafficLight = new TrafficLight(this);
}

Node::~Node()
{
    delete trafficLight;
}

void Node::addWaitingVehicle(Vehicle *vehicle)
{
    waitingVehicles.push_back(vehicle);
}

void Node::removeWaitingVehicle(Vehicle *vehicle)
{
    waitingVehicles.erase(
        std::remove(waitingVehicles.begin(), waitingVehicles.end(), vehicle),
        waitingVehicles.end());
}

std::string Node::getStatusString() const
{
    std::stringstream ss;
    ss << "Node " << id << " - Traffic Light: " << trafficLight->getStateString()
       << ", Waiting Vehicles: " << waitingVehicles.size();

    return ss.str();
}

// Implementation of Edge class
Edge::Edge(int edgeId, Node *src, Node *dest, int w)
    : id(edgeId), source(src), destination(dest), weight(w), blocked(false), blockageTimeLeft(0)
{
}

void Edge::addVehicle(Vehicle *vehicle)
{
    vehiclesOnRoad.push_back(vehicle);
}

void Edge::removeVehicle(Vehicle *vehicle)
{
    vehiclesOnRoad.erase(
        std::remove(vehiclesOnRoad.begin(), vehiclesOnRoad.end(), vehicle),
        vehiclesOnRoad.end());
}

void Edge::createAccident(int duration)
{
    if (!blocked)
    {
        blocked = true;
        blockageTimeLeft = duration;
        std::cout << "ACCIDENT: Road from Node " << source->getId()
                  << " to Node " << destination->getId() << " is now blocked." << std::endl;
    }
}

void Edge::updateBlockageTime(int timeElapsed)
{
    if (blocked)
    {
        blockageTimeLeft -= timeElapsed;
        if (blockageTimeLeft <= 0)
        {
            blocked = false;
            std::cout << "CLEARED: Road from Node " << source->getId() << " to Node "
                      << destination->getId() << " is now clear." << std::endl;
        }
    }
}

void Edge::updateCongestion()
{
    // Update weight based on number of vehicles on the road
    int baseWeight = 10;                             // Base travel time
    weight = baseWeight + vehiclesOnRoad.size() * 2; // Each vehicle adds 2 to travel time

    // If blocked, make weight very high
    if (blocked)
    {
        weight = INFINITY_VALUE / 2; // High but not infinite to allow for recalculation
    }
}

std::string Edge::getStatusString() const
{
    std::stringstream ss;
    ss << "Road " << id << " (Node " << source->getId() << " → Node " << destination->getId() << ") - "
       << "Vehicles: " << vehiclesOnRoad.size() << ", "
       << "Weight: " << weight << ", "
       << "Status: " << (blocked ? "BLOCKED (" + std::to_string(blockageTimeLeft) + "s left)" : "OPEN");

    return ss.str();
}

// Implementation of TrafficLight class
TrafficLight::TrafficLight(Node *node)
    : currentState(TrafficLightState::RED), timeInCurrentState(0), parentNode(node)
{
}

void TrafficLight::setState(TrafficLightState state)
{
    currentState = state;
    timeInCurrentState = 0;
}

void TrafficLight::update(int timeElapsed)
{
    timeInCurrentState += timeElapsed;

    // Traffic light cycle with longer durations
    switch (currentState)
    {
    case TrafficLightState::RED:
        if (timeInCurrentState >= RED_LIGHT_DURATION)
        {
            setState(TrafficLightState::GREEN);
        }
        break;

    case TrafficLightState::GREEN:
        if (timeInCurrentState >= GREEN_LIGHT_DURATION)
        {
            setState(TrafficLightState::YELLOW);
        }
        break;

    case TrafficLightState::YELLOW:
        if (timeInCurrentState >= YELLOW_LIGHT_DURATION)
        {
            setState(TrafficLightState::RED);
        }
        break;
    }
}

bool TrafficLight::canPass(Edge *fromEdge, Edge *toEdge) const
{
    // For simplicity, all vehicles can pass if the light is green
    return currentState == TrafficLightState::GREEN;
}

std::string TrafficLight::getStateString() const
{
    return trafficLightStateToString(currentState);
}

// Implementation of Vehicle class
Vehicle::Vehicle(int vehicleId, Node *src, Node *dest, Graph *graph)
    : id(vehicleId), source(src), destination(dest), currentNode(src), currentEdge(nullptr),
      progress(0.0f), speed(1.0f), state(VehicleState::WAITING_AT_LIGHT)
{
    // Calculate initial path
    calculatePath(graph);

    // Initialize position
    x = source->getX();
    y = source->getY();
}

void Vehicle::calculatePath(Graph *graph)
{
    // Reset path
    path.clear();

    // Use Dijkstra's algorithm to find the shortest path
    std::vector<Edge *> shortestPath = graph->findShortestPath(currentNode, destination);
    path = shortestPath;
}

void Vehicle::update(int timeElapsed, Graph *graph)
{
    switch (state)
    {
    case VehicleState::WAITING_AT_LIGHT:
        // If there's no path or path is empty, recalculate
        if (path.empty())
        {
            calculatePath(graph);
            if (path.empty())
            {
                // No path available
                return;
            }
        }

        // Check if we can proceed to the next edge
        if (!path.empty() && currentNode->getTrafficLight()->canPass(nullptr, path[0]))
        {
            currentEdge = path[0];
            currentNode->removeWaitingVehicle(this);
            currentEdge->addVehicle(this);
            state = VehicleState::MOVING;
            progress = 0.0f;
        }
        break;

    case VehicleState::MOVING:
        // Update position along the edge
        move(timeElapsed);

        // Check if we've reached the end of the current edge
        if (progress >= 1.0f)
        {
            // Remove from current edge
            currentEdge->removeVehicle(this);

            // Update current node
            currentNode = currentEdge->getDestination();

            // Remove the edge we just traversed from our path
            path.erase(path.begin());

            // Check if we've reached the destination
            if (currentNode == destination)
            {
                state = VehicleState::REACHED_DESTINATION;
                return;
            }

            // Check for path to continue
            if (path.empty())
            {
                // Need to recalculate path
                calculatePath(graph);
                if (path.empty())
                {
                    // No path available
                    state = VehicleState::WAITING_AT_LIGHT;
                    currentNode->addWaitingVehicle(this);
                    return;
                }
            }

            // Wait at the traffic light
            state = VehicleState::WAITING_AT_LIGHT;
            currentNode->addWaitingVehicle(this);
        }
        break;

    case VehicleState::REACHED_DESTINATION:
        // Vehicle has reached its destination, nothing to update
        break;

    case VehicleState::INVOLVED_IN_ACCIDENT:
        // Vehicle is involved in an accident, wait for cleanup
        break;
    }
}

void Vehicle::move(int timeElapsed)
{
    if (state != VehicleState::MOVING || !currentEdge)
    {
        return;
    }

    // Calculate how much to move based on speed, time elapsed, and edge weight
    float moveAmount = (speed * timeElapsed) / (currentEdge->getWeight() * 1000.0f);
    progress += moveAmount;

    // Clamp progress to [0, 1]
    progress = std::min(1.0f, progress);

    // Update position for visualization - linear interpolation
    float startX = currentEdge->getSource()->getX();
    float startY = currentEdge->getSource()->getY();
    float endX = currentEdge->getDestination()->getX();
    float endY = currentEdge->getDestination()->getY();

    x = startX + progress * (endX - startX);
    y = startY + progress * (endY - startY);
}

void Vehicle::replanPath(Graph *graph)
{
    // Save current edge if any
    Edge *oldCurrentEdge = currentEdge;

    // Recalculate path from current position
    calculatePath(graph);

    // If we were moving, continue movement on the new path
    if (state == VehicleState::MOVING && oldCurrentEdge)
    {
        // If old edge is now blocked, go back to waiting
        if (oldCurrentEdge->isBlocked())
        {
            state = VehicleState::WAITING_AT_LIGHT;
            currentNode->addWaitingVehicle(this);
            oldCurrentEdge->removeVehicle(this);
            currentEdge = nullptr;
        }
    }
}

std::string Vehicle::getStatusString() const
{
    std::stringstream ss;
    ss << "Vehicle " << id << " - "
       << "From: Node " << source->getId()
       << ", To: Node " << destination->getId()
       << ", Status: " << vehicleStateToString(state);

    if (state == VehicleState::MOVING && currentEdge)
    {
        ss << " on Road " << currentEdge->getId()
           << " (Node " << currentEdge->getSource()->getId()
           << " → Node " << currentEdge->getDestination()->getId()
           << ", Progress: " << std::fixed << std::setprecision(2) << progress * 100 << "%)";
    }
    else if (state == VehicleState::WAITING_AT_LIGHT && currentNode)
    {
        ss << " at Node " << currentNode->getId();
    }

    return ss.str();
}

// Implementation of Graph class
Graph::Graph()
{
}

Graph::~Graph()
{
    // Clean up nodes and edges
    for (Node *node : nodes)
    {
        delete node;
    }

    for (Edge *edge : edges)
    {
        delete edge;
    }
}

void Graph::addNode(Node *node)
{
    nodes.push_back(node);

    // Resize adjacency list if needed
    if (adjacencyList.size() <= static_cast<size_t>(node->getId()))
    {
        adjacencyList.resize(node->getId() + 1);
    }
}

void Graph::addEdge(Edge *edge)
{
    edges.push_back(edge);

    // Add to adjacency list
    int srcId = edge->getSource()->getId();
    if (adjacencyList.size() <= static_cast<size_t>(srcId))
    {
        adjacencyList.resize(srcId + 1);
    }
    adjacencyList[srcId].push_back(edge);
}

Node *Graph::getNodeById(int id) const
{
    for (Node *node : nodes)
    {
        if (node->getId() == id)
        {
            return node;
        }
    }
    return nullptr;
}

Edge *Graph::getEdge(Node *src, Node *dest) const
{
    for (Edge *edge : edges)
    {
        if (edge->getSource() == src && edge->getDestination() == dest)
        {
            return edge;
        }
    }
    return nullptr;
}

std::vector<Edge *> Graph::getOutgoingEdges(Node *node) const
{
    if (static_cast<size_t>(node->getId()) < adjacencyList.size())
    {
        return adjacencyList[node->getId()];
    }
    return std::vector<Edge *>();
}

std::vector<Edge *> Graph::findShortestPath(Node *source, Node *destination)
{
    // Implementation of Dijkstra's algorithm
    std::vector<int> dist(nodes.size(), INFINITY_VALUE);
    std::vector<Edge *> prev(nodes.size(), nullptr);
    std::vector<bool> visited(nodes.size(), false);

    // Priority queue for Dijkstra's algorithm
    std::priority_queue<std::pair<int, Node *>,
                        std::vector<std::pair<int, Node *>>,
                        std::greater<std::pair<int, Node *>>>
        pq;

    // Initialize
    dist[source->getId()] = 0;
    pq.push(std::make_pair(0, source));

    while (!pq.empty())
    {
        Node *current = pq.top().second;
        pq.pop();

        if (current == destination)
        {
            break;
        }

        if (visited[current->getId()])
        {
            continue;
        }

        visited[current->getId()] = true;

        // Check all neighbors
        for (Edge *edge : getOutgoingEdges(current))
        {
            Node *neighbor = edge->getDestination();
            int weight = edge->getWeight();

            // Skip blocked edges
            if (edge->isBlocked())
            {
                continue;
            }

            int newDist = dist[current->getId()] + weight;
            if (newDist < dist[neighbor->getId()])
            {
                dist[neighbor->getId()] = newDist;
                prev[neighbor->getId()] = edge;
                pq.push(std::make_pair(newDist, neighbor));
            }
        }
    }

    // Reconstruct path
    std::vector<Edge *> path;

    if (dist[destination->getId()] == INFINITY_VALUE)
    {
        // No path found
        return path;
    }

    // Start from destination and work backward
    Node *current = destination;
    while (current != source)
    {
        Edge *edge = prev[current->getId()];
        if (!edge)
        {
            // Something went wrong
            path.clear();
            return path;
        }

        path.push_back(edge);
        current = edge->getSource();
    }

    // Reverse to get source to destination order
    std::reverse(path.begin(), path.end());
    return path;
}

std::string Graph::getNetworkStatus() const
{
    std::stringstream ss;

    // Print all nodes and their traffic lights
    ss << "=== NODES AND TRAFFIC LIGHTS ===\n";
    for (Node *node : nodes)
    {
        ss << node->getStatusString() << "\n";
    }

    // Print all edges and their status
    ss << "\n=== ROADS ===\n";
    for (Edge *edge : edges)
    {
        ss << edge->getStatusString() << "\n";
    }

    return ss.str();
}

// Implementation of TerminalDisplay class
TerminalDisplay::TerminalDisplay(Simulator *sim)
    : simulator(sim)
{
}

void TerminalDisplay::initialize()
{
    std::cout << "=== TRAFFIC SIMULATION INITIALIZED ===\n";
    std::cout << "Nodes: " << simulator->getRoadNetwork()->getNodeCount() << "\n";
    std::cout << "Roads: " << simulator->getRoadNetwork()->getEdgeCount() << "\n";
    std::cout << "Initial Vehicles: " << simulator->getVehicles().size() << "\n";
    std::cout << "Press Enter to start the simulation...\n";
    std::cin.get();
}

void TerminalDisplay::render(int timeStep)
{
    // Clear console (this is a bit of a hack, works on most terminals)
    std::cout << "\033[2J\033[1;1H";

    // Display status for this time step
    displayStatus(timeStep);
}

void TerminalDisplay::displayStatus(int timeStep)
{
    std::cout << "====================================\n";
    std::cout << "TIME STEP: " << timeStep << " seconds\n";
    std::cout << "====================================\n\n";

    // Display network status
    std::cout << simulator->getRoadNetwork()->getNetworkStatus() << "\n";

    // Display all vehicles and their status
    std::cout << "=== VEHICLES ===\n";
    const std::vector<Vehicle *> &vehicles = simulator->getVehicles();
    if (vehicles.empty())
    {
        std::cout << "No vehicles in the simulation.\n";
    }
    else
    {
        for (Vehicle *vehicle : vehicles)
        {
            std::cout << vehicle->getStatusString() << "\n";
        }
    }

    std::cout << "\n";
}

void TerminalDisplay::summary()
{
    std::cout << "====================================\n";
    std::cout << "SIMULATION COMPLETED\n";
    std::cout << "====================================\n";
    std::cout << "Final Time: " << simulator->getTime() << " seconds\n";
    std::cout << "Remaining Vehicles: " << simulator->getVehicles().size() << "\n";

    // Count vehicles by state
    std::map<VehicleState, int> vehiclesByState;
    for (Vehicle *vehicle : simulator->getVehicles())
    {
        vehiclesByState[vehicle->getState()]++;
    }

    std::cout << "Vehicle States:\n";
    std::cout << "  - Moving: " << vehiclesByState[VehicleState::MOVING] << "\n";
    std::cout << "  - Waiting at light: " << vehiclesByState[VehicleState::WAITING_AT_LIGHT] << "\n";
    std::cout << "  - Reached destination: " << vehiclesByState[VehicleState::REACHED_DESTINATION] << "\n";
    std::cout << "  - Involved in accident: " << vehiclesByState[VehicleState::INVOLVED_IN_ACCIDENT] << "\n";
}

// Implementation of Simulator class
Simulator::Simulator()
    : roadNetwork(new Graph()), time(0), accidentProbability(2), lastAccidentTime(-30)
{
    // Initialize random number generator
    std::random_device rd;
    rng = std::mt19937(rd());
}

Simulator::~Simulator()
{
    delete roadNetwork;

    for (Vehicle *vehicle : vehicles)
    {
        delete vehicle;
    }
}

void Simulator::initialize(int nodeCount, int edgeCount)
{
    // Generate random road network
    std::uniform_real_distribution<float> xDist(50, 1150);
    std::uniform_real_distribution<float> yDist(50, 750);
    std::uniform_int_distribution<int> weightDist(5, 20);

    // Create nodes
    for (int i = 0; i < nodeCount; i++)
    {
        Node *node = new Node(i, xDist(rng), yDist(rng));
        roadNetwork->addNode(node);
    }

    // Create edges to ensure connected graph
    // First, ensure every node is connected to the graph
    for (int i = 1; i < nodeCount; i++)
    {
        int j = i - 1; // Connect to previous node
        Node *src = roadNetwork->getNodeById(j);
        Node *dest = roadNetwork->getNodeById(i);

        Edge *edge = new Edge(roadNetwork->getEdgeCount(), src, dest, weightDist(rng));
        roadNetwork->addEdge(edge);
    }

    // Add remaining random edges
    int remainingEdges = edgeCount - (nodeCount - 1);
    if (remainingEdges > 0)
    {
        std::uniform_int_distribution<int> nodeDist(0, nodeCount - 1);

        for (int i = 0; i < remainingEdges; i++)
        {
            int srcId = nodeDist(rng);
            int destId = nodeDist(rng);

            // Avoid self-loops and duplicate edges
            while (srcId == destId || roadNetwork->getEdge(roadNetwork->getNodeById(srcId),
                                                           roadNetwork->getNodeById(destId)) != nullptr)
            {
                srcId = nodeDist(rng);
                destId = nodeDist(rng);
            }

            Node *src = roadNetwork->getNodeById(srcId);
            Node *dest = roadNetwork->getNodeById(destId);

            Edge *edge = new Edge(roadNetwork->getEdgeCount(), src, dest, weightDist(rng));
            roadNetwork->addEdge(edge);
        }
    }

    // Generate some initial vehicles
    generateVehicles(nodeCount / 2);
}

void Simulator::step(int timeElapsed)
{
    time += timeElapsed;

    // Update traffic lights
    for (Node *node : roadNetwork->getNodes())
    {
        node->getTrafficLight()->update(timeElapsed);
    }

    // Update edges (check for accident clearance)
    for (Edge *edge : roadNetwork->getEdges())
    {
        edge->updateBlockageTime(timeElapsed);
        edge->updateCongestion();
    }

    // Update vehicles
    for (auto it = vehicles.begin(); it != vehicles.end();)
    {
        Vehicle *vehicle = *it;
        vehicle->update(timeElapsed, roadNetwork);

        // Remove vehicles that have reached their destination
        if (vehicle->getState() == VehicleState::REACHED_DESTINATION)
        {
            delete vehicle;
            it = vehicles.erase(it);
        }
        else
        {
            ++it;
        }
    }

    // Check for potential accidents
    checkForAccidents();

    // Generate new vehicles occasionally
    if (time % 10 == 0) // Every 10 seconds
    {
        generateVehicles(roadNetwork->getNodeCount() / 5);
    }
}

void Simulator::checkForAccidents()
{
    // Only check every 5 steps and if enough time has passed since last accident
    if (time % 5 != 0 || time - lastAccidentTime < 30)
    {
        return;
    }

    std::uniform_int_distribution<int> chanceDist(1, 100);
    if (chanceDist(rng) <= accidentProbability)
    {
        // Create an accident on a random edge with vehicles
        std::vector<Edge *> edgesWithVehicles;
        for (Edge *edge : roadNetwork->getEdges())
        {
            if (edge->getVehicleCount() > 0 && !edge->isBlocked())
            {
                edgesWithVehicles.push_back(edge);
            }
        }

        if (!edgesWithVehicles.empty())
        {
            std::uniform_int_distribution<int> edgeDist(0, edgesWithVehicles.size() - 1);
            Edge *accidentEdge = edgesWithVehicles[edgeDist(rng)];

            // Create the accident
            accidentEdge->createAccident(ACCIDENT_DURATION);
            lastAccidentTime = time;

            // Mark a random vehicle as involved in accident
            const std::vector<Vehicle *> &vehiclesOnRoad = accidentEdge->getVehicles();
            if (!vehiclesOnRoad.empty())
            {
                std::uniform_int_distribution<int> vehicleDist(0, vehiclesOnRoad.size() - 1);
                Vehicle *accidentVehicle = vehiclesOnRoad[vehicleDist(rng)];
                accidentVehicle->setState(VehicleState::INVOLVED_IN_ACCIDENT);
            }

            // Make all vehicles on the network replan their routes
            for (Vehicle *vehicle : vehicles)
            {
                if (vehicle->getState() == VehicleState::MOVING)
                {
                    vehicle->replanPath(roadNetwork);
                }
            }
        }
    }
}

void Simulator::generateVehicles(int count)
{
    if (roadNetwork->getNodeCount() < 2)
    {
        return;
    }

    std::uniform_int_distribution<int> nodeDist(0, roadNetwork->getNodeCount() - 1);

    for (int i = 0; i < count; i++)
    {
        int sourceId = nodeDist(rng);
        int destId = nodeDist(rng);

        // Make sure source and destination are different
        while (sourceId == destId)
        {
            destId = nodeDist(rng);
        }

        Node *source = roadNetwork->getNodeById(sourceId);
        Node *destination = roadNetwork->getNodeById(destId);

        addVehicle(source, destination);
    }
}

void Simulator::addVehicle(Node *source, Node *destination)
{
    if (source == destination)
    {
        return;
    }

    // Check if there's a path between source and destination
    std::vector<Edge *> path = roadNetwork->findShortestPath(source, destination);
    if (path.empty())
    {
        return; // No path available
    }

    // Create and add the vehicle
    Vehicle *vehicle = new Vehicle(vehicles.size(), source, destination, roadNetwork);
    vehicles.push_back(vehicle);

    // Add to the source node's waiting vehicles
    source->addWaitingVehicle(vehicle);
}

void Simulator::run(int simulationTime)
{
    TerminalDisplay display(this);
    display.initialize();

    int stepSize = 1; // Update every second

    for (int t = 0; t < simulationTime; t += stepSize)
    {
        step(stepSize);
        display.render(t);

        // Slow down simulation for visualization
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    display.summary();
}

// Main function
int main()
{
    Simulator simulator;

    // Initialize with 10 nodes and 20 edges
    simulator.initialize(10, 20);

    // Run for 60 seconds
    simulator.run(TOTAL_SIMULATION_TIME);

    return 0;
}