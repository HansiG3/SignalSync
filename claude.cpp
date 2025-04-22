// Traffic Simulation System with Traffic Lights
// Main components: 
// - Graph representation of road network
// - Traffic light system
// - Vehicle movement and path finding
// - Accident simulation
// - Visualization with SFML

#include <iostream>
#include <vector>
#include <queue>
#include <list>
#include <limits>
#include <algorithm>
#include <random>
#include <chrono>
#include <thread>
#include <SFML/Graphics.hpp>

// Forward declarations
class Vehicle;
class TrafficLight;
class Node;
class Edge;
class Graph;
class Simulator;
class Visualization;

// Constants
const int INFINITY_VALUE = std::numeric_limits<int>::max();
const int RED_LIGHT_DURATION = 5;      // seconds
const int GREEN_LIGHT_DURATION = 7;    // seconds
const int YELLOW_LIGHT_DURATION = 2;   // seconds
const int ACCIDENT_DURATION = 10;      // seconds
const int SCREEN_WIDTH = 1200;
const int SCREEN_HEIGHT = 800;

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
    float x, y;  // Coordinates for visualization
    TrafficLight* trafficLight;
    std::vector<Vehicle*> waitingVehicles;

public:
    Node(int nodeId, float posX, float posY);
    ~Node();

    int getId() const { return id; }
    float getX() const { return x; }
    float getY() const { return y; }

    TrafficLight* getTrafficLight() const { return trafficLight; }
    void addWaitingVehicle(Vehicle* vehicle);
    void removeWaitingVehicle(Vehicle* vehicle);
    int getWaitingVehiclesCount() const { return waitingVehicles.size(); }
    
    void draw(sf::RenderWindow& window);
};

// Edge class represents a road between two intersections
class Edge
{
private:
    int id;
    Node* source;
    Node* destination;
    int weight;              // Weight represents congestion/time to traverse
    bool blocked;            // Represents an accident or blockage
    int blockageTimeLeft;    // Time until the blockage is cleared
    std::vector<Vehicle*> vehiclesOnRoad;

public:
    Edge(int edgeId, Node* src, Node* dest, int w);

    int getId() const { return id; }
    Node* getSource() const { return source; }
    Node* getDestination() const { return destination; }
    int getWeight() const { return weight; }
    bool isBlocked() const { return blocked; }
    
    void addVehicle(Vehicle* vehicle);
    void removeVehicle(Vehicle* vehicle);
    int getVehicleCount() const { return vehiclesOnRoad.size(); }
    
    void createAccident(int duration);
    void updateBlockageTime(int timeElapsed);
    void updateCongestion();
    
    void draw(sf::RenderWindow& window);
};

// TrafficLight class for managing intersection traffic
class TrafficLight
{
private:
    TrafficLightState currentState;
    int timeInCurrentState;
    Node* parentNode;
    
    // Direction mappings for traffic flow control
    std::vector<std::pair<Edge*, Edge*>> flowDirections;

public:
    TrafficLight(Node* node);

    TrafficLightState getState() const { return currentState; }
    void setState(TrafficLightState state);
    void update(int timeElapsed);
    bool canPass(Edge* fromEdge, Edge* toEdge) const;
    
    void draw(sf::RenderWindow& window);
};

// Vehicle class for representing cars in the simulation
class Vehicle
{
private:
    int id;
    Node* source;
    Node* destination;
    Node* currentNode;
    Edge* currentEdge;
    float progress;          // Progress along the current edge (0.0 to 1.0)
    float speed;             // Speed factor affecting how fast the vehicle moves
    VehicleState state;
    std::vector<Edge*> path; // Calculated path from source to destination
    float x, y;              // Current position for visualization
    sf::Color color;         // Vehicle color for visualization

public:
    Vehicle(int vehicleId, Node* src, Node* dest, Graph* graph);
    
    int getId() const { return id; }
    Node* getSource() const { return source; }
    Node* getDestination() const { return destination; }
    Node* getCurrentNode() const { return currentNode; }
    Edge* getCurrentEdge() const { return currentEdge; }
    VehicleState getState() const { return state; }
    
    void setState(VehicleState newState) { state = newState; }
    void setCurrentNode(Node* node) { currentNode = node; }
    void setCurrentEdge(Edge* edge) { currentEdge = edge; }
    
    void calculatePath(Graph* graph);
    void update(int timeElapsed, Graph* graph);
    void move(int timeElapsed);
    void replanPath(Graph* graph);
    
    void draw(sf::RenderWindow& window);
};

// Graph class represents the entire road network
class Graph
{
private:
    std::vector<Node*> nodes;
    std::vector<Edge*> edges;
    std::vector<std::vector<Edge*>> adjacencyList;

public:
    Graph();
    ~Graph();
    
    void addNode(Node* node);
    void addEdge(Edge* edge);
    
    Node* getNodeById(int id) const;
    Edge* getEdge(Node* src, Node* dest) const;
    std::vector<Edge*> getOutgoingEdges(Node* node) const;
    
    int getNodeCount() const { return nodes.size(); }
    int getEdgeCount() const { return edges.size(); }
    std::vector<Node*> getNodes() const { return nodes; }
    std::vector<Edge*> getEdges() const { return edges; }
    
    std::vector<Edge*> findShortestPath(Node* source, Node* destination);
    
    void draw(sf::RenderWindow& window);
};

// Simulator class controls the entire simulation
class Simulator
{
private:
    Graph* roadNetwork;
    std::vector<Vehicle*> vehicles;
    int time;
    int accidentProbability;  // Chance of an accident happening (0-100)
    std::mt19937 rng;         // Random number generator
    
    void checkForAccidents();
    void generateVehicles(int count);

public:
    Simulator();
    ~Simulator();
    
    void initialize(int nodeCount, int edgeCount);
    void step(int timeElapsed);
    void addVehicle(Node* source, Node* destination);
    
    Graph* getRoadNetwork() const { return roadNetwork; }
    std::vector<Vehicle*> getVehicles() const { return vehicles; }
    
    void run(int simulationTime);
};

// Visualization class for SFML rendering
class Visualization
{
private:
    sf::RenderWindow window;
    Simulator* simulator;
    sf::Font font;
    
    void drawGraph();
    void drawVehicles();
    void drawUI();

public:
    Visualization(Simulator* sim);
    
    void initialize();
    void render();
    bool isOpen() const { return window.isOpen(); }
    void handleEvents();
};

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

void Node::addWaitingVehicle(Vehicle* vehicle)
{
    waitingVehicles.push_back(vehicle);
}

void Node::removeWaitingVehicle(Vehicle* vehicle)
{
    waitingVehicles.erase(
        std::remove(waitingVehicles.begin(), waitingVehicles.end(), vehicle),
        waitingVehicles.end());
}

void Node::draw(sf::RenderWindow& window)
{
    // Draw node as a circle
    sf::CircleShape nodeShape(20);
    nodeShape.setPosition(x - 20, y - 20);
    nodeShape.setFillColor(sf::Color(100, 100, 100));
    nodeShape.setOutlineThickness(2);
    nodeShape.setOutlineColor(sf::Color::Black);
    window.draw(nodeShape);
    
    // Draw node ID
    sf::Text idText;
    sf::Font font;
    // if (font.loadFromFile("arial.ttf")) {
    //     idText.setFont(font);
    //     idText.setString(std::to_string(id));
    //     idText.setCharacterSize(18);
    //     idText.setFillColor(sf::Color::White);
    //     idText.setPosition(x - 5, y - 12);
    //     window.draw(idText);
    // }
    
    // Draw traffic light
    trafficLight->draw(window);
}

// Implementation of Edge class
Edge::Edge(int edgeId, Node* src, Node* dest, int w)
    : id(edgeId), source(src), destination(dest), weight(w), blocked(false), blockageTimeLeft(0)
{
}

void Edge::addVehicle(Vehicle* vehicle)
{
    vehiclesOnRoad.push_back(vehicle);
}

void Edge::removeVehicle(Vehicle* vehicle)
{
    vehiclesOnRoad.erase(
        std::remove(vehiclesOnRoad.begin(), vehiclesOnRoad.end(), vehicle),
        vehiclesOnRoad.end());
}

void Edge::createAccident(int duration)
{
    if (!blocked) {
        blocked = true;
        blockageTimeLeft = duration;
        std::cout << "Accident occurred on road from Node " << source->getId() 
                  << " to Node " << destination->getId() << std::endl;
    }
}

void Edge::updateBlockageTime(int timeElapsed)
{
    if (blocked) {
        blockageTimeLeft -= timeElapsed;
        if (blockageTimeLeft <= 0) {
            blocked = false;
            std::cout << "Road from Node " << source->getId() << " to Node " 
                      << destination->getId() << " is now clear." << std::endl;
        }
    }
}

void Edge::updateCongestion()
{
    // Update weight based on number of vehicles on the road
    int baseWeight = 10; // Base travel time
    weight = baseWeight + vehiclesOnRoad.size() * 2; // Each vehicle adds 2 to travel time
    
    // If blocked, make weight very high
    if (blocked) {
        weight = INFINITY_VALUE / 2; // High but not infinite to allow for recalculation
    }
}

void Edge::draw(sf::RenderWindow& window)
{
    // Draw a line representing the road
    sf::Vertex line[] = {
        sf::Vertex(sf::Vector2f(source->getX(), source->getY())),
        sf::Vertex(sf::Vector2f(destination->getX(), destination->getY()))
    };
    
    // Set color based on congestion and blockage
    if (blocked) {
        line[0].color = sf::Color::Red;
        line[1].color = sf::Color::Red;
    } else {
        // Color gradient from green (no congestion) to yellow to red (high congestion)
        int congestionLevel = std::min(255, static_cast<int>(50 + vehiclesOnRoad.size() * 30));
        line[0].color = sf::Color(congestionLevel, 255 - congestionLevel, 0);
        line[1].color = sf::Color(congestionLevel, 255 - congestionLevel, 0);
    }
    
    window.draw(line, 2, sf::Lines);
    
    // Draw arrow to indicate direction
    float dx = destination->getX() - source->getX();
    float dy = destination->getY() - source->getY();
    float length = std::sqrt(dx * dx + dy * dy);
    
    if (length > 0) {
        dx /= length;
        dy /= length;
        
        float arrowPosX = source->getX() + dx * length * 0.7;
        float arrowPosY = source->getY() + dy * length * 0.7;
        
        sf::ConvexShape arrow;
        arrow.setPointCount(3);
        arrow.setPoint(0, sf::Vector2f(arrowPosX + dy * 5, arrowPosY - dx * 5));
        arrow.setPoint(1, sf::Vector2f(arrowPosX + dx * 10, arrowPosY + dy * 10));
        arrow.setPoint(2, sf::Vector2f(arrowPosX - dy * 5, arrowPosY + dx * 5));
        
        if (blocked) {
            arrow.setFillColor(sf::Color::Red);
        } else {
            int congestionLevel = std::min(255, static_cast<int>(50 + vehiclesOnRoad.size() * 30));
            arrow.setFillColor(sf::Color(congestionLevel, 255 - congestionLevel, 0));
        }
        
        window.draw(arrow);
    }
    
    // Draw weight text
    sf::Text weightText;
    sf::Font font;
    // if (font.loadFromFile("arial.ttf")) {
    //     weightText.setFont(font);
    //     weightText.setString(std::to_string(weight));
    //     weightText.setCharacterSize(15);
    //     weightText.setFillColor(sf::Color::White);
    //     weightText.setPosition(
    //         (source->getX() + destination->getX()) / 2,
    //         (source->getY() + destination->getY()) / 2
    //     );
    //     window.draw(weightText);
    // }
}

// Implementation of TrafficLight class
TrafficLight::TrafficLight(Node* node)
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
    
    // Simple traffic light cycle
    switch (currentState)
    {
        case TrafficLightState::RED:
            if (timeInCurrentState >= RED_LIGHT_DURATION) {
                setState(TrafficLightState::GREEN);
            }
            break;
            
        case TrafficLightState::GREEN:
            if (timeInCurrentState >= GREEN_LIGHT_DURATION) {
                setState(TrafficLightState::YELLOW);
            }
            break;
            
        case TrafficLightState::YELLOW:
            if (timeInCurrentState >= YELLOW_LIGHT_DURATION) {
                setState(TrafficLightState::RED);
            }
            break;
    }
}

bool TrafficLight::canPass(Edge* fromEdge, Edge* toEdge) const
{
    // For simplicity, all vehicles can pass if the light is green
    return currentState == TrafficLightState::GREEN;
}

void TrafficLight::draw(sf::RenderWindow& window)
{
    // Draw traffic light as a colored circle above the node
    sf::CircleShape lightShape(10);
    
    // Position above the node
    lightShape.setPosition(parentNode->getX() - 10, parentNode->getY() - 45);
    
    // Set color based on state
    switch (currentState)
    {
        case TrafficLightState::RED:
            lightShape.setFillColor(sf::Color::Red);
            break;
        case TrafficLightState::GREEN:
            lightShape.setFillColor(sf::Color::Green);
            break;
        case TrafficLightState::YELLOW:
            lightShape.setFillColor(sf::Color::Yellow);
            break;
    }
    
    window.draw(lightShape);
}

// Implementation of Vehicle class
Vehicle::Vehicle(int vehicleId, Node* src, Node* dest, Graph* graph)
    : id(vehicleId), source(src), destination(dest), currentNode(src), currentEdge(nullptr),
      progress(0.0f), speed(1.0f), state(VehicleState::WAITING_AT_LIGHT)
{
    // Calculate initial path
    calculatePath(graph);
    
    // Assign random color for visualization
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> colorDist(50, 200);
    
    color = sf::Color(colorDist(gen), colorDist(gen), colorDist(gen));
    
    // Initialize position
    x = source->getX();
    y = source->getY();
}

void Vehicle::calculatePath(Graph* graph)
{
    // Reset path
    path.clear();
    
    // Use Dijkstra's algorithm to find the shortest path
    std::vector<Edge*> shortestPath = graph->findShortestPath(currentNode, destination);
    path = shortestPath;
}

void Vehicle::update(int timeElapsed, Graph* graph)
{
    switch (state)
    {
        case VehicleState::WAITING_AT_LIGHT:
            // If there's no path or path is empty, recalculate
            if (path.empty()) {
                calculatePath(graph);
                if (path.empty()) {
                    // No path available
                    return;
                }
            }
            
            // Check if we can proceed to the next edge
            if (!path.empty() && currentNode->getTrafficLight()->canPass(nullptr, path[0])) {
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
            if (progress >= 1.0f) {
                // Remove from current edge
                currentEdge->removeVehicle(this);
                
                // Update current node
                currentNode = currentEdge->getDestination();
                
                // Remove the edge we just traversed from our path
                path.erase(path.begin());
                
                // Check if we've reached the destination
                if (currentNode == destination) {
                    state = VehicleState::REACHED_DESTINATION;
                    return;
                }
                
                // Check for path to continue
                if (path.empty()) {
                    // Need to recalculate path
                    calculatePath(graph);
                    if (path.empty()) {
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
    if (state != VehicleState::MOVING || !currentEdge) {
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

void Vehicle::replanPath(Graph* graph)
{
    // Save current edge if any
    Edge* oldCurrentEdge = currentEdge;
    
    // Recalculate path from current position
    calculatePath(graph);
    
    // If we were moving, continue movement on the new path
    if (state == VehicleState::MOVING && oldCurrentEdge) {
        // If old edge is now blocked, go back to waiting
        if (oldCurrentEdge->isBlocked()) {
            state = VehicleState::WAITING_AT_LIGHT;
            currentNode->addWaitingVehicle(this);
            oldCurrentEdge->removeVehicle(this);
            currentEdge = nullptr;
        }
    }
}

void Vehicle::draw(sf::RenderWindow& window)
{
    // Draw vehicle as a small circle
    sf::CircleShape vehicleShape(5);
    vehicleShape.setPosition(x - 5, y - 5);
    vehicleShape.setFillColor(color);
    window.draw(vehicleShape);
    
    // Draw ID
    sf::Text idText;
    sf::Font font;
    // if (font.loadFromFile("arial.ttf")) {
    //     idText.setFont(font);
    //     idText.setString(std::to_string(id));
    //     idText.setCharacterSize(10);
    //     idText.setFillColor(sf::Color::White);
    //     idText.setPosition(x - 3, y - 3);
    //     window.draw(idText);
    // }
}

// Implementation of Graph class
Graph::Graph()
{
}

Graph::~Graph()
{
    // Clean up nodes and edges
    for (Node* node : nodes) {
        delete node;
    }
    
    for (Edge* edge : edges) {
        delete edge;
    }
}

void Graph::addNode(Node* node)
{
    nodes.push_back(node);
    
    // Resize adjacency list if needed
    if (adjacencyList.size() <= static_cast<size_t>(node->getId())) {
        adjacencyList.resize(node->getId() + 1);
    }
}

void Graph::addEdge(Edge* edge)
{
    edges.push_back(edge);
    
    // Add to adjacency list
    int srcId = edge->getSource()->getId();
    if (adjacencyList.size() <= static_cast<size_t>(srcId)) {
        adjacencyList.resize(srcId + 1);
    }
    adjacencyList[srcId].push_back(edge);
}

Node* Graph::getNodeById(int id) const
{
    for (Node* node : nodes) {
        if (node->getId() == id) {
            return node;
        }
    }
    return nullptr;
}

Edge* Graph::getEdge(Node* src, Node* dest) const
{
    for (Edge* edge : edges) {
        if (edge->getSource() == src && edge->getDestination() == dest) {
            return edge;
        }
    }
    return nullptr;
}

std::vector<Edge*> Graph::getOutgoingEdges(Node* node) const
{
    if (static_cast<size_t>(node->getId()) < adjacencyList.size()) {
        return adjacencyList[node->getId()];
    }
    return std::vector<Edge*>();
}

std::vector<Edge*> Graph::findShortestPath(Node* source, Node* destination)
{
    // Implementation of Dijkstra's algorithm
    std::vector<int> dist(nodes.size(), INFINITY_VALUE);
    std::vector<Edge*> prev(nodes.size(), nullptr);
    std::vector<bool> visited(nodes.size(), false);
    
    // Priority queue for Dijkstra's algorithm
    std::priority_queue<std::pair<int, Node*>, 
                        std::vector<std::pair<int, Node*>>, 
                        std::greater<std::pair<int, Node*>>> pq;
    
    // Initialize
    dist[source->getId()] = 0;
    pq.push(std::make_pair(0, source));
    
    while (!pq.empty()) {
        Node* current = pq.top().second;
        pq.pop();
        
        if (current == destination) {
            break;
        }
        
        if (visited[current->getId()]) {
            continue;
        }
        
        visited[current->getId()] = true;
        
        // Check all neighbors
        for (Edge* edge : getOutgoingEdges(current)) {
            Node* neighbor = edge->getDestination();
            int weight = edge->getWeight();
            
            // Skip blocked edges
            if (edge->isBlocked()) {
                continue;
            }
            
            int newDist = dist[current->getId()] + weight;
            if (newDist < dist[neighbor->getId()]) {
                dist[neighbor->getId()] = newDist;
                prev[neighbor->getId()] = edge;
                pq.push(std::make_pair(newDist, neighbor));
            }
        }
    }
    
    // Reconstruct path
    std::vector<Edge*> path;
    
    if (dist[destination->getId()] == INFINITY_VALUE) {
        // No path found
        return path;
    }
    
    // Start from destination and work backward
    Node* current = destination;
    while (current != source) {
        Edge* edge = prev[current->getId()];
        if (!edge) {
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

void Graph::draw(sf::RenderWindow& window)
{
    // Draw all edges first
    for (Edge* edge : edges) {
        edge->draw(window);
    }
    
    // Then draw all nodes (so they appear on top)
    for (Node* node : nodes) {
        node->draw(window);
    }
}

// Implementation of Simulator class
Simulator::Simulator() 
    : roadNetwork(new Graph()), time(0), accidentProbability(5)
{
    // Initialize random number generator
    std::random_device rd;
    rng = std::mt19937(rd());
}

Simulator::~Simulator()
{
    delete roadNetwork;
    
    for (Vehicle* vehicle : vehicles) {
        delete vehicle;
    }
}

void Simulator::initialize(int nodeCount, int edgeCount)
{
    // Generate random road network
    std::uniform_real_distribution<float> xDist(50, SCREEN_WIDTH - 50);
    std::uniform_real_distribution<float> yDist(50, SCREEN_HEIGHT - 50);
    std::uniform_int_distribution<int> weightDist(5, 20);
    
    // Create nodes
    for (int i = 0; i < nodeCount; i++) {
        Node* node = new Node(i, xDist(rng), yDist(rng));
        roadNetwork->addNode(node);
    }
    
    // Create edges to ensure connected graph
    // First, ensure every node is connected to the graph
    for (int i = 1; i < nodeCount; i++) {
        int j = i - 1;  // Connect to previous node
        Node* src = roadNetwork->getNodeById(j);
        Node* dest = roadNetwork->getNodeById(i);
        
        Edge* edge = new Edge(roadNetwork->getEdgeCount(), src, dest, weightDist(rng));
        roadNetwork->addEdge(edge);
    }
    
    // Add remaining random edges
    int remainingEdges = edgeCount - (nodeCount - 1);
    if (remainingEdges > 0) {
        std::uniform_int_distribution<int> nodeDist(0, nodeCount - 1);
        
        for (int i = 0; i < remainingEdges; i++) {
            int srcId = nodeDist(rng);
            int destId = nodeDist(rng);
            
            // Avoid self-loops and duplicate edges
            while (srcId == destId || roadNetwork->getEdge(roadNetwork->getNodeById(srcId), 
                                                        roadNetwork->getNodeById(destId)) != nullptr) {
                srcId = nodeDist(rng);
                destId = nodeDist(rng);
            }
            
            Node* src = roadNetwork->getNodeById(srcId);
            Node* dest = roadNetwork->getNodeById(destId);
            
            Edge* edge = new Edge(roadNetwork->getEdgeCount(), src, dest, weightDist(rng));
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
    for (Node* node : roadNetwork->getNodes()) {
        node->getTrafficLight()->update(timeElapsed);
    }
    
    // Update edges (check for accident clearance)
    for (Edge* edge : roadNetwork->getEdges()) {
        edge->updateBlockageTime(timeElapsed);
        edge->updateCongestion();
    }
    
    // Update vehicles
    for (auto it = vehicles.begin(); it != vehicles.end();) {
        Vehicle* vehicle = *it;
        vehicle->update(timeElapsed, roadNetwork);
        
        // Remove vehicles that have reached their destination
        if (vehicle->getState() == VehicleState::REACHED_DESTINATION) {
            delete vehicle;
            it = vehicles.erase(it);
        } else {
            ++it;
        }
    }
    
    // Check for potential accidents
    checkForAccidents();
    
    // Occasionally generate new vehicles
    if (time % 10 == 0) {
        generateVehicles(1);
    }
}

void Simulator::checkForAccidents()
{
    // Simple accident model: random chance of accident on edges with multiple vehicles
    std::uniform_int_distribution<int> chanceDist(1, 100);
    
    for (Edge* edge : roadNetwork->getEdges()) {
        if (edge->getVehicleCount() >= 2 && !edge->isBlocked()) {
            // Higher chance with more vehicles
            int chance = chanceDist(rng);
            if (chance <= accidentProbability) {
                edge->createAccident(ACCIDENT_DURATION);
                
                // Replan paths for all affected vehicles
                for (Vehicle* vehicle : vehicles) {
                    if (vehicle->getCurrentEdge() == edge) {
                        vehicle->setState(VehicleState::INVOLVED_IN_ACCIDENT);
                    } else {
                        vehicle->replanPath(roadNetwork);
                    }
                }
            }
        }
    }
}

void Simulator::generateVehicles(int count)
{
    if (roadNetwork->getNodeCount() < 2) {
        return;
    }
    
    std::uniform_int_distribution<int> nodeDist(0, roadNetwork->getNodeCount() - 1);
    
    for (int i = 0; i < count; i++) {
        // Generate random source and destination
        int srcId = nodeDist(rng);
        int destId = nodeDist(rng);
        
        // Ensure source and destination are different
        while (srcId == destId) {
            destId = nodeDist(rng);
        }
        
        Node* src = roadNetwork->getNodeById(srcId);
        Node* dest = roadNetwork->getNodeById(destId);
        
        // Create and add the vehicle
        addVehicle(src, dest);
    }
}

void Simulator::addVehicle(Node* source, Node* destination)
{
    // Create new vehicle with unique ID
    int vehicleId = vehicles.size();
    Vehicle* vehicle = new Vehicle(vehicleId, source, destination, roadNetwork);
    
    // Add to vehicles list and to waiting vehicles at source node
    vehicles.push_back(vehicle);
    source->addWaitingVehicle(vehicle);
}

void Simulator::run(int simulationTime)
{
    int elapsedTime = 0;
    while (elapsedTime < simulationTime) {
        step(1);  // Step 1 time unit at a time
        elapsedTime++;
        
        // Sleep to slow down simulation for visual purposes
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Implementation of Visualization class
Visualization::Visualization(Simulator* sim)
    : window(sf::VideoMode(SCREEN_WIDTH, SCREEN_HEIGHT), "Traffic Simulation System"), 
      simulator(sim)
{
}

void Visualization::initialize()
{
    // Load font
    // bool fontLoaded = font.loadFromFile("arial.ttf");
	// if (!fontLoaded) {
	// 	std::cout << "Font not found. Text display disabled." << std::endl;
	// }
}

void Visualization::render()
{
    window.clear(sf::Color(30, 30, 30));  // Dark background
    
    // Draw graph (road network)
    drawGraph();
    
    // Draw vehicles
    drawVehicles();
    
    // Draw UI elements
    drawUI();
    
    window.display();
}

void Visualization::drawGraph()
{
    simulator->getRoadNetwork()->draw(window);
}

void Visualization::drawVehicles()
{
    for (Vehicle* vehicle : simulator->getVehicles()) {
        vehicle->draw(window);
    }
}

void Visualization::drawUI()
{
    // if (!font.loadFromFile("arial.ttf")) {
    //     return;  // Can't draw text without font
    // }
    
    // Draw vehicle count
    sf::Text vehicleCountText;
    vehicleCountText.setFont(font);
    vehicleCountText.setString("Vehicles: " + std::to_string(simulator->getVehicles().size()));
    vehicleCountText.setCharacterSize(20);
    vehicleCountText.setFillColor(sf::Color::White);
    vehicleCountText.setPosition(20, 20);
    window.draw(vehicleCountText);
}

void Visualization::handleEvents()
{
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window.close();
        }
        
        // Add other event handling as needed
        // For example, mouse clicks to add new vehicles or create accidents
    }
}

// Main function
int main()
{
    // Get user input for number of nodes and edges
    int nodeCount, edgeCount;
    
    std::cout << "Enter number of nodes (minimum 15 recommended): ";
    std::cin >> nodeCount;
    // nodeCount = std::max(15, nodeCount);  // Ensure at least 15 nodes
    
    std::cout << "Enter number of edges (minimum " << (nodeCount - 1) << " required): ";
    std::cin >> edgeCount;
    edgeCount = std::max(nodeCount - 1, edgeCount);  // Ensure minimum edges for connectivity
    
    // Initialize simulator
    Simulator simulator;
    simulator.initialize(nodeCount, edgeCount);
    
    // Initialize visualization
    Visualization vis(&simulator);
    vis.initialize();
    
    // Main simulation loop
    sf::Clock clock;
    while (vis.isOpen()) {
        // Handle window events
        vis.handleEvents();
        
        // Update simulation
        float dt = clock.restart().asSeconds();
        simulator.step(static_cast<int>(dt * 1000));  // Convert to milliseconds
        
        // Render
        vis.render();

        // Limit frame rate
        std::this_thread::sleep_for(std::chrono::milliseconds(16));  // ~60 FPS
    }
    return 0;
}

// Additional functionality that could be added:
// - User interaction to manually add vehicles, create accidents, etc.
// - Statistics tracking (average travel time, congestion levels, etc.)
// - Advanced traffic light algorithms based on traffic density
// - Multiple vehicle types with different speeds
// - One-way/two-way roads
// - Multiple lanes per road
// - Pedestrian crossings
// - Different road types (highways, residential roads)
// - Real-time visualization improvements
// - User controls to speed up/slow down simulation

/*
To compile this program with SFML:

1. Install SFML:
   - On Linux: sudo apt-get install libsfml-dev
   - On Windows: Download from https://www.sfml-dev.org/ and link properly
   - On Mac: brew install sfml

2. Compile with:
   g++ -std=c++17 traffic_simulation.cpp -o traffic_sim -lsfml-graphics -lsfml-window -lsfml-system

3. Run the program:
   ./traffic_sim
*/