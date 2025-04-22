#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <random>
#include <chrono>
#include <thread>
#include <string>
#include <algorithm>
#include <iomanip>
#include <ctime>
#include <limits>
#include <cctype>
#include <mutex>
#include <atomic>
#include <condition_variable>
#ifdef _WIN32
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#endif

// Optional SFML for visualization (disabled by default)
#define USE_SFML_VISUALIZATION 0
#if USE_SFML_VISUALIZATION
#include <SFML/Graphics.hpp>
#endif

using namespace std;

// Simulation configuration
struct SimulationConfig {
    int numIntersections = 20;
    int numRoads = 50;
    int numVehicles = 150;
    int numPedestrians = 50;
    double simulationStep = 0.05;
    int maxSimulationSteps = 1000; // Reduced for faster testing
    double accidentProbability = 0.003;
    double constructionProbability = 0.002;
    double emergencyVehicleProbability = 0.001;
    double weatherImpact = 0.0;
    double minRoadLength = 100.0;
    double maxRoadLength = 1200.0;
    double minVehicleSpeed = 8.0;
    double maxVehicleSpeed = 22.0;
    double minPedestrianSpeed = 1.0;
    double maxPedestrianSpeed = 2.0;
    double greenDuration = 25.0;
    double yellowDuration = 4.0;
    double redDuration = 25.0;
    string logFile = "traffic_simulation_log.txt";
    string errorLogFile = "traffic_simulation_errors.txt";
    string analyticsDir = "analytics/";
    int logFrequency = 100; // Increased to reduce output
    double maxSimulationTime = 300.0; // 5 minutes
    string scenarioName = "default";
};

// Forward declarations
class Intersection;
class Road;
class TrafficLight;
class Vehicle;
class Pedestrian;
class Event;
class TrafficSimulation;

// Random number generator
random_device rd;
mt19937 gen(rd());

// Utility functions
double getRandom(double min, double max) {
    uniform_real_distribution<> dis(min, max);
    return dis(gen);
}

string getCurrentTime() {
    time_t now = time(nullptr);
    stringstream ss;
    ss << put_time(localtime(&now), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

bool isKeyPressed() {
#ifdef _WIN32
    return _kbhit();
#else
    struct termios oldt, newt;
    int ch, oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF) {
        ungetc(ch, stdin);
        return true;
    }
    return false;
#endif
}

// Configuration parser
class ConfigParser {
public:
    static SimulationConfig parse(const string& filename) {
        SimulationConfig config;
        ifstream file(filename);
        if (!file.is_open()) {
            cout << "Error: Could not open config file " << filename << ". Using defaults.\n";
            return config;
        }

        string line;
        while (getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            stringstream ss(line);
            string key, valueStr;
            if (getline(ss, key, '=') && getline(ss, valueStr)) {
                key.erase(remove_if(key.begin(), key.end(), ::isspace), key.end());
                valueStr.erase(remove_if(valueStr.begin(), valueStr.end(), ::isspace), valueStr.end());
                try {
                    if (key == "numIntersections") config.numIntersections = max(1, min(100, stoi(valueStr)));
                    else if (key == "numRoads") config.numRoads = max(1, min(200, stoi(valueStr)));
                    else if (key == "numVehicles") config.numVehicles = max(0, min(1000, stoi(valueStr)));
                    else if (key == "numPedestrians") config.numPedestrians = max(0, min(500, stoi(valueStr)));
                    else if (key == "simulationStep") config.simulationStep = max(0.01, min(1.0, stod(valueStr)));
                    else if (key == "maxSimulationSteps") config.maxSimulationSteps = max(100, min(10000, stoi(valueStr)));
                    else if (key == "accidentProbability") config.accidentProbability = max(0.0, min(0.1, stod(valueStr)));
                    else if (key == "constructionProbability") config.constructionProbability = max(0.0, min(0.1, stod(valueStr)));
                    else if (key == "emergencyVehicleProbability") config.emergencyVehicleProbability = max(0.0, min(0.1, stod(valueStr)));
                    else if (key == "weatherImpact") config.weatherImpact = max(0.0, min(1.0, stod(valueStr)));
                    else if (key == "minRoadLength") config.minRoadLength = max(10.0, stod(valueStr));
                    else if (key == "maxRoadLength") config.maxRoadLength = max(config.minRoadLength, stod(valueStr));
                    else if (key == "minVehicleSpeed") config.minVehicleSpeed = max(1.0, stod(valueStr));
                    else if (key == "maxVehicleSpeed") config.maxVehicleSpeed = max(config.minVehicleSpeed, stod(valueStr));
                    else if (key == "minPedestrianSpeed") config.minPedestrianSpeed = max(0.1, stod(valueStr));
                    else if (key == "maxPedestrianSpeed") config.maxPedestrianSpeed = max(config.minPedestrianSpeed, stod(valueStr));
                    else if (key == "greenDuration") config.greenDuration = max(5.0, stod(valueStr));
                    else if (key == "yellowDuration") config.yellowDuration = max(1.0, stod(valueStr));
                    else if (key == "redDuration") config.redDuration = max(5.0, stod(valueStr));
                    else if (key == "logFile") config.logFile = valueStr;
                    else if (key == "errorLogFile") config.errorLogFile = valueStr;
                    else if (key == "analyticsDir") config.analyticsDir = valueStr;
                    else if (key == "logFrequency") config.logFrequency = max(10, stoi(valueStr));
                    else if (key == "maxSimulationTime") config.maxSimulationTime = max(60.0, min(3600.0, stod(valueStr)));
                    else if (key == "scenarioName") config.scenarioName = valueStr;
                } catch (const exception& e) {
                    cout << "Warning: Invalid config value for " << key << ": " << valueStr << "\n";
                }
            }
        }
        file.close();
        return config;
    }

    static void writeDefaultConfig(const string& filename) {
        ofstream file(filename);
        if (!file.is_open()) {
            cout << "Error: Could not create config file " << filename << ".\n";
            return;
        }
        file << "# Traffic Simulation Configuration\n";
        file << "numIntersections=20\n";
        file << "numRoads=50\n";
        file << "numVehicles=150\n";
        file << "numPedestrians=50\n";
        file << "simulationStep=0.05\n";
        file << "maxSimulationSteps=1000\n";
        file << "accidentProbability=0.003\n";
        file << "constructionProbability=0.002\n";
        file << "emergencyVehicleProbability=0.001\n";
        file << "weatherImpact=0.0\n";
        file << "minRoadLength=100.0\n";
        file << "maxRoadLength=1200.0\n";
        file << "minVehicleSpeed=8.0\n";
        file << "maxVehicleSpeed=22.0\n";
        file << "minPedestrianSpeed=1.0\n";
        file << "maxPedestrianSpeed=2.0\n";
        file << "greenDuration=25.0\n";
        file << "yellowDuration=4.0\n";
        file << "redDuration=25.0\n";
        file << "logFile=traffic_simulation_log.txt\n";
        file << "errorLogFile=traffic_simulation_errors.txt\n";
        file << "analyticsDir=analytics/\n";
        file << "logFrequency=100\n";
        file << "maxSimulationTime=300.0\n";
        file << "scenarioName=default\n";
        file.close();
        cout << "Created default config file: " << filename << "\n";
    }
};

// Intersection class
class Intersection {
public:
    int id;
    vector<Road*> outgoingRoads;
    vector<Road*> incomingRoads;
    TrafficLight* trafficLight;
    double x, y;
    bool hasCrosswalk;

    Intersection(int id, double x = 0.0, double y = 0.0, bool crosswalk = false)
        : id(id), trafficLight(nullptr), x(x), y(y), hasCrosswalk(crosswalk) {}

    void addOutgoingRoad(Road* road) { outgoingRoads.push_back(road); }
    void addIncomingRoad(Road* road) { incomingRoads.push_back(road); }
    void setTrafficLight(TrafficLight* light) { trafficLight = light; }
};

// Road class
class Road {
public:
    int id;
    Intersection* start;
    Intersection* end;
    double length;
    double maxSpeed;
    double congestion;
    bool isBlocked;
    int numLanes;
    vector<Vehicle*> vehiclesOnRoad;
    vector<Pedestrian*> pedestriansOnRoad;

    Road(int id, Intersection* start, Intersection* end, double length, double maxSpeed, int numLanes = 2)
        : id(id), start(start), end(end), length(length), maxSpeed(maxSpeed),
          congestion(0.0), isBlocked(false), numLanes(numLanes) {}

    double getTravelTime() const {
        return length / (maxSpeed * (1.0 - congestion));
    }

    void updateCongestion() {
        congestion = min(1.0, max(0.0, static_cast<double>(vehiclesOnRoad.size()) / (numLanes * 15.0)));
    }

    void addVehicle(Vehicle* vehicle) { vehiclesOnRoad.push_back(vehicle); }
    void removeVehicle(Vehicle* vehicle) {
        vehiclesOnRoad.erase(remove(vehiclesOnRoad.begin(), vehiclesOnRoad.end(), vehicle), vehiclesOnRoad.end());
    }
    void addPedestrian(Pedestrian* pedestrian) { pedestriansOnRoad.push_back(pedestrian); }
    void removePedestrian(Pedestrian* pedestrian) {
        pedestriansOnRoad.erase(remove(pedestriansOnRoad.begin(), pedestriansOnRoad.end(), pedestrian), pedestriansOnRoad.end());
    }
};

// TrafficLight class
class TrafficLight {
public:
    enum State { RED, GREEN, YELLOW };
    State state;
    double timeInState;
    double greenDuration;
    double yellowDuration;
    double redDuration;
    Intersection* intersection;
    int waitingVehicles;
    int waitingPedestrians;
    double congestionFactor;

    TrafficLight(Intersection* intersection, double green, double yellow, double red)
        : state(GREEN), timeInState(0.0), greenDuration(green), yellowDuration(yellow),
          redDuration(red), intersection(intersection), waitingVehicles(0), waitingPedestrians(0),
          congestionFactor(1.0) {}

    void optimizeTiming(double congestion) {
        congestionFactor = 1.0 + congestion * 0.5;
    }

    void update(double deltaTime, int vehicles, int pedestrians) {
        waitingVehicles = vehicles;
        waitingPedestrians = pedestrians;
        timeInState += deltaTime;

        double adjustedGreen = greenDuration * congestionFactor * (1.0 + (waitingVehicles + waitingPedestrians) / 12.0);
        double adjustedRed = redDuration / congestionFactor * (1.0 - (waitingVehicles + waitingPedestrians) / 20.0);

        if (state == GREEN && timeInState >= adjustedGreen) {
            state = YELLOW;
            timeInState = 0.0;
        } else if (state == YELLOW && timeInState >= yellowDuration) {
            state = RED;
            timeInState = 0.0;
        } else if (state == RED && timeInState >= adjustedRed) {
            state = GREEN;
            timeInState = 0.0;
        }
    }

    bool canPassVehicles() const { return state == GREEN; }
    bool canPassPedestrians() const { return state == RED && intersection->hasCrosswalk; }
    string getStateString() const {
        switch (state) {
            case GREEN: return "Green";
            case YELLOW: return "Yellow";
            case RED: return "Red";
            default: return "Unknown";
        }
    }
};

// Vehicle class
class Vehicle {
public:
    int id;
    Road* currentRoad;
    double position;
    double speed;
    double maxSpeed;
    Intersection* destination;
    vector<Road*> path;
    bool isStopped;
    int currentLane;
    double acceleration;
    bool hasCollided;
    bool isEmergency;
    double timeWaiting;
    double timeTraveling;
    double distanceTraveled;

    Vehicle(int id, Road* road, double maxSpeed, Intersection* dest, int lane = 0, bool emergency = false)
        : id(id), currentRoad(road), position(0.0), speed(0.0), maxSpeed(maxSpeed),
          destination(dest), isStopped(false), currentLane(lane), acceleration(emergency ? 4.0 : 2.5),
          hasCollided(false), isEmergency(emergency), timeWaiting(0.0), timeTraveling(0.0),
          distanceTraveled(0.0) {
        road->addVehicle(this);
    }

    void move(double deltaTime, double weatherImpact) {
        if (isStopped || currentRoad->isBlocked || hasCollided) {
            if (isStopped) timeWaiting += deltaTime;
            return;
        }

        double weatherFactor = 1.0 - weatherImpact * (isEmergency ? 0.2 : 0.5);
        double targetSpeed = maxSpeed * (1.0 - currentRoad->congestion) * (1.0 - 0.1 * currentLane) * weatherFactor;
        speed = min(speed + acceleration * deltaTime, targetSpeed);
        double distance = speed * deltaTime;

        if (position + distance >= currentRoad->length) {
            position = currentRoad->length;
            isStopped = true;
        } else {
            position += distance;
            timeTraveling += deltaTime;
            distanceTraveled += distance;
        }
    }

    bool atIntersection() const { return position >= currentRoad->length; }
    void setPath(const vector<Road*>& newPath) { path = newPath; }
    void changeLane(int newLane) {
        if (newLane >= 0 && newLane < currentRoad->numLanes && newLane != currentLane) {
            currentLane = newLane;
            speed *= 0.9;
        }
    }
    void handleCollision() {
        hasCollided = true;
        speed = 0.0;
        isStopped = true;
    }
};

// Pedestrian class
class Pedestrian {
public:
    int id;
    Road* currentRoad;
    double position;
    double speed;
    double maxSpeed;
    Intersection* destination;
    vector<Road*> path;
    bool isWaiting;
    double timeWaiting;
    double distanceTraveled;

    Pedestrian(int id, Road* road, double maxSpeed, Intersection* dest)
        : id(id), currentRoad(road), position(0.0), speed(maxSpeed), maxSpeed(maxSpeed),
          destination(dest), isWaiting(false), timeWaiting(0.0), distanceTraveled(0.0) {
        road->addPedestrian(this);
    }

    void move(double deltaTime, TrafficLight* light) {
        if (isWaiting || !light->canPassPedestrians()) {
            isWaiting = true;
            timeWaiting += deltaTime;
            return;
        }

        double distance = speed * deltaTime;
        if (position + distance >= currentRoad->length) {
            position = currentRoad->length;
            isWaiting = true;
        } else {
            position += distance;
            distanceTraveled += distance;
        }
    }

    bool atIntersection() const { return position >= currentRoad->length; }
    void setPath(const vector<Road*>& newPath) { path = newPath; }
};

// Event class
class Event {
public:
    enum Type { ACCIDENT, CONSTRUCTION, EMERGENCY_VEHICLE, TRAFFIC_JAM };
    Type type;
    Road* road;
    double startTime;
    double duration;
    bool active;

    Event(Type type, Road* road, double startTime, double duration)
        : type(type), road(road), startTime(startTime), duration(duration), active(true) {}

    void apply(TrafficSimulation& sim);
    void end(TrafficSimulation& sim);
};

// Graph class
class Graph {
public:
    vector<Intersection*> intersections;
    vector<Road*> roads;

    void addIntersection(Intersection* intersection) { intersections.push_back(intersection); }
    void addRoad(Road* road) {
        roads.push_back(road);
        road->start->addOutgoingRoad(road);
        road->end->addIncomingRoad(road);
    }

    vector<Road*> dijkstra(Intersection* start, Intersection* end, bool isEmergency = false) {
        map<Intersection*, double> dist;
        map<Intersection*, Road*> prev;
        set<Intersection*> visited;
        priority_queue<pair<double, Intersection*>, vector<pair<double, Intersection*>>, greater<>> pq;

        for (auto* inter : intersections) {
            dist[inter] = numeric_limits<double>::infinity();
        }
        dist[start] = 0.0;
        pq.push({0.0, start});

        while (!pq.empty()) {
            auto [d, curr] = pq.top();
            pq.pop();

            if (curr == end) break;
            if (visited.count(curr)) continue;
            visited.insert(curr);

            for (auto* road : curr->outgoingRoads) {
                if (road->isBlocked && !isEmergency) continue;
                double newDist = dist[curr] + road->getTravelTime();
                if (isEmergency) newDist *= 0.5; // Emergency vehicles prioritize speed
                if (newDist < dist[road->end]) {
                    dist[road->end] = newDist;
                    prev[road->end] = road;
                    pq.push({newDist, road->end});
                }
            }
        }

        vector<Road*> path;
        Intersection* curr = end;
        while (prev.find(curr) != prev.end()) {
            path.push_back(prev[curr]);
            curr = prev[curr]->start;
        }
        reverse(path.begin(), path.end());
        return path;
    }
};

// Logger class
class Logger {
private:
    ofstream logFile;
    ofstream errorFile;
    mutex logMutex;

public:
    Logger(const string& logFilename, const string& errorFilename) {
        logFile.open(logFilename, ios::app);
        errorFile.open(errorFilename, ios::app);
        if (!logFile.is_open()) cout << "Error: Could not open log file " << logFilename << ".\n";
        if (!errorFile.is_open()) cout << "Error: Could not open error log file " << errorFilename << ".\n";
        log("Simulation started at " + getCurrentTime());
    }

    ~Logger() {
        log("Simulation ended at " + getCurrentTime());
        if (logFile.is_open()) logFile.close();
        if (errorFile.is_open()) errorFile.close();
    }

    void log(const string& message) {
        lock_guard<mutex> lock(logMutex);
        string msg = "[" + getCurrentTime() + "] " + message;
        if (logFile.is_open()) logFile << msg << "\n";
        cout << msg << "\n";
    }

    void error(const string& message) {
        lock_guard<mutex> lock(logMutex);
        string msg = "[" + getCurrentTime() + "] ERROR: " + message;
        if (errorFile.is_open()) errorFile << msg << "\n";
        cerr << msg << "\n";
    }
};

// TrafficSimulation class
class TrafficSimulation {
private:
    SimulationConfig config;
    Graph graph;
    vector<Vehicle*> vehicles;
    vector<Pedestrian*> pedestrians;
    vector<TrafficLight*> trafficLights;
    vector<Event*> events;
    int stepCount;
    Logger logger;
    double totalVehicleTravelTime;
    double totalVehicleWaitTime;
    double totalPedestrianWaitTime;
    int totalCollisions;
    double simulationStartTime;
    atomic<bool> shouldStop;
    atomic<bool> isPaused;
    mutex simulationMutex;
    condition_variable simulationCV;
    thread inputThread;

    #if USE_SFML_VISUALIZATION
    sf::RenderWindow window;
    #endif

    void handleInput() {
        while (!shouldStop) {
            if (isKeyPressed()) {
                char ch = getchar();
                if (ch == 'p' || ch == 'P') {
                    isPaused = !isPaused;
                    logger.log(isPaused ? "Simulation paused" : "Simulation resumed");
                } else if (ch == 'q' || ch == 'Q') {
                    shouldStop = true;
                    logger.log("Simulation stopped by user");
                }
            }
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }

public:
    TrafficSimulation(const string& configFile = "traffic_sim_config.txt")
        : stepCount(0), logger(config.logFile, config.errorLogFile), totalVehicleTravelTime(0.0),
          totalVehicleWaitTime(0.0), totalPedestrianWaitTime(0.0), totalCollisions(0),
          shouldStop(false), isPaused(false) {
        config = ConfigParser::parse(configFile);
        if (!ifstream(configFile).good()) {
            ConfigParser::writeDefaultConfig(configFile);
        }

        simulationStartTime = chrono::duration_cast<chrono::seconds>(
            chrono::system_clock::now().time_since_epoch()
        ).count();

        #if USE_SFML_VISUALIZATION
        window.create(sf::VideoMode(1000, 800), "Traffic Simulation - " + config.scenarioName);
        window.setFramerateLimit(60);
        #endif

        initializeRoadNetwork();
        initializeVehicles();
        initializePedestrians();
        logger.log("Initialized scenario '" + config.scenarioName + "': " +
                   to_string(config.numIntersections) + " intersections, " +
                   to_string(config.numRoads) + " roads, " +
                   to_string(config.numVehicles) + " vehicles, " +
                   to_string(config.numPedestrians) + " pedestrians");

        inputThread = thread(&TrafficSimulation::handleInput, this);
    }

    ~TrafficSimulation() {
        shouldStop = true;
        if (inputThread.joinable()) inputThread.join();
        for (auto* inter : graph.intersections) delete inter;
        for (auto* road : graph.roads) delete road;
        for (auto* vehicle : vehicles) delete vehicle;
        for (auto* pedestrian : pedestrians) delete pedestrian;
        for (auto* light : trafficLights) delete light;
        for (auto* event : events) delete event;
    }

    void initializeRoadNetwork() {
        for (int i = 0; i < config.numIntersections; ++i) {
            double x = getRandom(50.0, 950.0);
            double y = getRandom(50.0, 750.0);
            bool hasCrosswalk = getRandom(0.0, 1.0) < 0.5;
            graph.addIntersection(new Intersection(i, x, y, hasCrosswalk));
        }

        for (int i = 0; i < config.numRoads; ++i) {
            int startId = getRandom(0, config.numIntersections - 1);
            int endId = getRandom(0, config.numIntersections - 1);
            while (startId == endId) endId = getRandom(0, config.numIntersections - 1);
            double length = getRandom(config.minRoadLength, config.maxRoadLength);
            double maxSpeed = getRandom(config.minVehicleSpeed, config.maxVehicleSpeed);
            int numLanes = getRandom(1, 4);
            graph.addRoad(new Road(i, graph.intersections[startId], graph.intersections[endId], length, maxSpeed, numLanes));
        }

        for (int i = 0; i < config.numIntersections * 3 / 4; ++i) {
            auto* light = new TrafficLight(graph.intersections[i], config.greenDuration,
                                         config.yellowDuration, config.redDuration);
            graph.intersections[i]->setTrafficLight(light);
            trafficLights.push_back(light);
        }
    }

    void initializeVehicles() {
        for (int i = 0; i < config.numVehicles; ++i) {
            Road* road = graph.roads[getRandom(0, graph.roads.size() - 1)];
            Intersection* dest = graph.intersections[getRandom(0, graph.intersections.size() - 1)];
            int lane = getRandom(0, road->numLanes - 1);
            bool isEmergency = getRandom(0.0, 1.0) < config.emergencyVehicleProbability;
            auto* vehicle = new Vehicle(i, road, getRandom(config.minVehicleSpeed, config.maxVehicleSpeed), dest, lane, isEmergency);
            auto path = graph.dijkstra(road->start, dest, isEmergency);
            if (path.empty()) {
                logger.error("No path found for vehicle " + to_string(i));
                delete vehicle;
                continue;
            }
            vehicle->setPath(path);
            vehicles.push_back(vehicle);
        }
    }

    void initializePedestrians() {
        for (int i = 0; i < config.numPedestrians; ++i) {
            Road* road = graph.roads[getRandom(0, graph.roads.size() - 1)];
            Intersection* dest = graph.intersections[getRandom(0, graph.intersections.size() - 1)];
            auto* pedestrian = new Pedestrian(i, road, getRandom(config.minPedestrianSpeed, config.maxPedestrianSpeed), dest);
            auto path = graph.dijkstra(road->start, dest);
            if (path.empty()) {
                logger.error("No path found for pedestrian " + to_string(i));
                delete pedestrian;
                continue;
            }
            pedestrian->setPath(path);
            pedestrians.push_back(pedestrian);
        }
    }

    void simulateEvents() {
        // New events
        if (getRandom(0.0, 1.0) < config.accidentProbability) {
            Road* road = graph.roads[getRandom(0, graph.roads.size() - 1)];
            if (!road->isBlocked) {
                events.push_back(new Event(Event::ACCIDENT, road, stepCount * config.simulationStep, getRandom(30.0, 120.0)));
            }
        }
        if (getRandom(0.0, 1.0) < config.constructionProbability) {
            Road* road = graph.roads[getRandom(0, graph.roads.size() - 1)];
            if (!road->isBlocked) {
                events.push_back(new Event(Event::CONSTRUCTION, road, stepCount * config.simulationStep, getRandom(60.0, 300.0)));
            }
        }
        if (getRandom(0.0, 1.0) < config.emergencyVehicleProbability) {
            Road* road = graph.roads[getRandom(0, graph.roads.size() - 1)];
            Intersection* dest = graph.intersections[getRandom(0, graph.intersections.size() - 1)];
            int lane = getRandom(0, road->numLanes - 1);
            auto* vehicle = new Vehicle(vehicles.size(), road, config.maxVehicleSpeed * 1.5, dest, lane, true);
            auto path = graph.dijkstra(road->start, dest, true);
            if (!path.empty()) {
                vehicle->setPath(path);
                vehicles.push_back(vehicle);
                events.push_back(new Event(Event::EMERGENCY_VEHICLE, road, stepCount * config.simulationStep, getRandom(20.0, 60.0)));
            } else {
                delete vehicle;
            }
        }

        // Update events
        for (auto it = events.begin(); it != events.end();) {
            Event* event = *it;
            if (event->active && (stepCount * config.simulationStep) >= event->startTime + event->duration) {
                event->end(*this);
                delete event;
                it = events.erase(it);
            } else {
                if (event->active) event->apply(*this);
                ++it;
            }
        }
    }

    void Event::apply(TrafficSimulation& sim) {
        switch (type) {
            case ACCIDENT:
                road->isBlocked = true;
                sim.logger.log("Accident on road " + to_string(road->id));
                break;
            case CONSTRUCTION:
                road->isBlocked = true;
                sim.logger.log("Construction on road " + to_string(road->id));
                break;
            case EMERGENCY_VEHICLE:
                // Handled by vehicle creation
                break;
            case TRAFFIC_JAM:
                road->congestion = min(1.0, road->congestion + 0.3);
                sim.logger.log("Traffic jam on road " + to_string(road->id));
                break;
        }
    }

    void Event::end(TrafficSimulation& sim) {
        active = false;
        switch (type) {
            case ACCIDENT:
            case CONSTRUCTION:
                road->isBlocked = false;
                sim.logger.log("Event ended on road " + to_string(road->id));
                break;
            case EMERGENCY_VEHICLE:
            case TRAFFIC_JAM:
                break;
        }
    }

    void simulateCollisions() {
        for (auto* road : graph.roads) {
            for (size_t i = 0; i < road->vehiclesOnRoad.size(); ++i) {
                for (size_t j = i + 1; j < road->vehiclesOnRoad.size(); ++j) {
                    Vehicle* v1 = road->vehiclesOnRoad[i];
                    Vehicle* v2 = road->vehiclesOnRoad[j];
                    if (v1->currentLane == v2->currentLane && 
                        abs(v1->position - v2->position) < 5.0 && 
                        !v1->hasCollided && !v2->hasCollided) {
                        v1->handleCollision();
                        v2->handleCollision();
                        totalCollisions++;
                        logger.log("Collision between vehicles " + to_string(v1->id) + 
                                  " and " + to_string(v2->id) + " on road " + to_string(road->id));
                    }
                }
            }
        }
    }

    void optimizeTrafficLights() {
        for (auto* light : trafficLights) {
            double avgCongestion = 0.0;
            for (auto* road : light->intersection->incomingRoads) {
                avgCongestion += road->congestion;
            }
            avgCongestion /= max(1, static_cast<int>(light->intersection->incomingRoads.size()));
            light->optimizeTiming(avgCongestion);
        }
    }

    void updateTrafficLights() {
        for (auto* light : trafficLights) {
            int waitingVehicles = 0;
            int waitingPedestrians = 0;
            for (auto* vehicle : vehicles) {
                if (vehicle->atIntersection() && vehicle->currentRoad->end == light->intersection && !light->canPassVehicles()) {
                    waitingVehicles++;
                    totalVehicleWaitTime += config.simulationStep;
                }
            }
            for (auto* pedestrian : pedestrians) {
                if (pedestrian->atIntersection() && pedestrian->currentRoad->end == light->intersection && !light->canPassPedestrians()) {
                    waitingPedestrians++;
                    totalPedestrianWaitTime += config.simulationStep;
                }
            }
            light->update(config.simulationStep, waitingVehicles, waitingPedestrians);
        }
    }

    void updateVehicles() {
        for (auto* vehicle : vehicles) {
            if (vehicle->hasCollided) continue;

            if (getRandom(0.0, 1.0) < 0.005 && !vehicle->atIntersection() && !vehicle->isEmergency) {
                int newLane = getRandom(0, vehicle->currentRoad->numLanes - 1);
                vehicle->changeLane(newLane);
            }

            if (vehicle->atIntersection()) {
                if (vehicle->currentRoad->end == vehicle->destination) {
                    vehicle->isStopped = true;
                    totalVehicleTravelTime += vehicle->timeTraveling;
                    continue;
                }

                TrafficLight* light = vehicle->currentRoad->end->trafficLight;
                if (light && !light->canPassVehicles() && !vehicle->isEmergency) {
                    vehicle->isStopped = true;
                    continue;
                }

                if (!vehicle->path.empty()) {
                    vehicle->currentRoad->removeVehicle(vehicle);
                    vehicle->currentRoad = vehicle->path[0];
                    vehicle->path.erase(vehicle->path.begin());
                    vehicle->currentRoad->addVehicle(vehicle);
                    vehicle->position = 0.0;
                    vehicle->isStopped = false;
                } else {
                    vehicle->isStopped = true;
                    auto newPath = graph.dijkstra(vehicle->currentRoad->end, vehicle->destination, vehicle->isEmergency);
                    if (!newPath.empty()) {
                        vehicle->setPath(newPath);
                        vehicle->isStopped = false;
                    } else {
                        logger.error("Vehicle " + to_string(vehicle->id) + " has no path");
                    }
                }
            }

            vehicle->move(config.simulationStep, config.weatherImpact);
        }

        for (auto* road : graph.roads) {
            road->updateCongestion();
        }
    }

    void updatePedestrians() {
        for (auto* pedestrian : pedestrians) {
            if (pedestrian->atIntersection()) {
                if (pedestrian->currentRoad->end == pedestrian->destination) {
                    pedestrian->isWaiting = true;
                    continue;
                }

                TrafficLight* light = pedestrian->currentRoad->end->trafficLight;
                pedestrian->move(config.simulationStep, light);

                if (pedestrian->atIntersection() && !pedestrian->isWaiting) {
                    if (!pedestrian->path.empty()) {
                        pedestrian->currentRoad->removePedestrian(pedestrian);
                        pedestrian->currentRoad = pedestrian->path[0];
                        pedestrian->path.erase(pedestrian->path.begin());
                        pedestrian->currentRoad->addPedestrian(pedestrian);
                        pedestrian->position = 0.0;
                        pedestrian->isWaiting = false;
                    } else {
                        auto newPath = graph.dijkstra(pedestrian->currentRoad->end, pedestrian->destination);
                        if (!newPath.empty()) {
                            pedestrian->setPath(newPath);
                            pedestrian->isWaiting = false;
                        } else {
                            pedestrian->isWaiting = true;
                            logger.error("Pedestrian " + to_string(pedestrian->id) + " has no path");
                        }
                    }
                }
            } else {
                TrafficLight* light = pedestrian->currentRoad->end->trafficLight;
                pedestrian->move(config.simulationStep, light);
            }
        }
    }

    void checkForStalledVehicles() {
        for (auto* vehicle : vehicles) {
            if (vehicle->isStopped && !vehicle->hasCollided && vehicle->timeWaiting > 30.0) {
                logger.error("Vehicle " + to_string(vehicle->id) + " stalled for " +
                            to_string(vehicle->timeWaiting) + " seconds");
                Intersection* start = vehicle->atIntersection() ? vehicle->currentRoad->end : vehicle->currentRoad->start;
                auto newPath = graph.dijkstra(start, vehicle->destination, vehicle->isEmergency);
                if (!newPath.empty()) {
                    vehicle->setPath(newPath);
                    vehicle->isStopped = false;
                    vehicle->timeWaiting = 0.0;
                } else {
                    vehicle->isStopped = true;
                    vehicle->speed = 0.0;
                }
            }
        }
    }

    void printStatistics() {
        stringstream ss;
        ss << "Simulation Statistics (Scenario: " << config.scenarioName << "):\n";
        ss << "Total Steps: " << stepCount << "\n";
        ss << "Total Collisions: " << totalCollisions << "\n";
        ss << "Average Vehicle Travel Time: " << fixed << setprecision(2)
           << totalVehicleTravelTime / max(1, static_cast<int>(vehicles.size())) << " s\n";
        ss << "Average Vehicle Wait Time: " << totalVehicleWaitTime / max(1, static_cast<int>(vehicles.size())) << " s\n";
        ss << "Average Pedestrian Wait Time: " << totalPedestrianWaitTime / max(1, static_cast<int>(pedestrians.size())) << " s\n";
        ss << "Average Congestion: " << calculateAverageCongestion() << "\n";
        ss << "Active Events: " << events.size() << "\n";
        logger.log(ss.str());
    }

    double calculateAverageCongestion() {
        double totalCongestion = 0.0;
        for (auto* road : graph.roads) {
            totalCongestion += road->congestion;
        }
        return totalCongestion / max(1, static_cast<int>(graph.roads.size()));
    }

    void exportAnalytics() {
        string dir = config.analyticsDir;
        system(("mkdir " + dir).c_str()); // Create analytics directory

        // Vehicle analytics
        ofstream vehicleFile(dir + "vehicles_" + config.scenarioName + ".csv");
        if (vehicleFile.is_open()) {
            vehicleFile << "VehicleID,RoadID,Position,Speed,Lane,Status,WaitTime,TravelTime,DistanceTraveled,IsEmergency\n";
            for (auto* vehicle : vehicles) {
                vehicleFile << vehicle->id << "," << vehicle->currentRoad->id << ","
                           << vehicle->position << "," << vehicle->speed << ","
                           << vehicle->currentLane << "," << (vehicle->isStopped ? "Stopped" : "Moving") << ","
                           << vehicle->timeWaiting << "," << vehicle->timeTraveling << ","
                           << vehicle->distanceTraveled << "," << (vehicle->isEmergency ? "Yes" : "No") << "\n";
            }
            vehicleFile.close();
        } else {
            logger.error("Could not create vehicle analytics file");
        }

        // Pedestrian analytics
        ofstream pedestrianFile(dir + "pedestrians_" + config.scenarioName + ".csv");
        if (pedestrianFile.is_open()) {
            pedestrianFile << "PedestrianID,RoadID,Position,Status,WaitTime,DistanceTraveled\n";
            for (auto* pedestrian : pedestrians) {
                pedestrianFile << pedestrian->id << "," << pedestrian->currentRoad->id << ","
                              << pedestrian->position << "," << (pedestrian->isWaiting ? "Waiting" : "Moving") << ","
                              << pedestrian->timeWaiting << "," << pedestrian->distanceTraveled << "\n";
            }
            pedestrianFile.close();
        } else {
            logger.error("Could not create pedestrian analytics file");
        }

        // Road analytics
        ofstream roadFile(dir + "roads_" + config.scenarioName + ".csv");
        if (roadFile.is_open()) {
            roadFile << "RoadID,Congestion,Vehicles,Pedestrians,Blocked\n";
            for (auto* road : graph.roads) {
                roadFile << road->id << "," << road->congestion << ","
                         << road->vehiclesOnRoad.size() << "," << road->pedestriansOnRoad.size() << ","
                         << (road->isBlocked ? "Yes" : "No") << "\n";
            }
            roadFile.close();
        } else {
            logger.error("Could not create road analytics file");
        }

        logger.log("Exported analytics to " + dir);
    }

    #if USE_SFML_VISUALIZATION
    void render() {
        window.clear(sf::Color::Black);

        for (auto* road : graph.roads) {
            sf::Vertex line[] = {
                sf::Vertex(sf::Vector2f(road->start->x, road->start->y), road->isBlocked ? sf::Color::Red : sf::Color::White),
                sf::Vertex(sf::Vector2f(road->end->x, road->end->y), road->isBlocked ? sf::Color::Red : sf::Color::White)
            };
            window.draw(line, 2, sf::Lines);
        }

        for (auto* inter : graph.intersections) {
            sf::CircleShape circle(6.0);
            circle.setPosition(inter->x - 6.0, inter->y - 6.0);
            circle.setFillColor(inter->hasCrosswalk ? sf::Color::Magenta : inter->trafficLight ? sf::Color::Yellow : sf::Color::Blue);
            window.draw(circle);
        }

        for (auto* vehicle : vehicles) {
            if (vehicle->hasCollided) continue;
            double t = vehicle->position / vehicle->currentRoad->length;
            double x = (1.0 - t) * vehicle->currentRoad->start->x + t * vehicle->currentRoad->end->x;
            double y = (1.0 - t) * vehicle->currentRoad->start->y + t * vehicle->currentRoad->end->y;
            sf::CircleShape car(4.0);
            car.setPosition(x - 4.0, y - 4.0);
            car.setFillColor(vehicle->isEmergency ? sf::Color::Cyan : vehicle->isStopped ? sf::Color::Red : sf::Color::Green);
            window.draw(car);
        }

        for (auto* pedestrian : pedestrians) {
            double t = pedestrian->position / pedestrian->currentRoad->length;
            double x = (1.0 - t) * pedestrian->currentRoad->start->x + t * pedestrian->currentRoad->end->x;
            double y = (1.0 - t) * pedestrian->currentRoad->start->y + t * pedestrian->currentRoad->end->y;
            sf::RectangleShape ped(sf::Vector2f(3.0, 3.0));
            ped.setPosition(x - 1.5, y - 1.5);
            ped.setFillColor(pedestrian->isWaiting ? sf::Color::Cyan : sf::Color::White);
            window.draw(ped);
        }

        for (auto* light : trafficLights) {
            sf::CircleShape lightCircle(5.0);
            lightCircle.setPosition(light->intersection->x + 7.0, light->intersection->y + 7.0);
            lightCircle.setFillColor(
                light->state == TrafficLight::GREEN ? sf::Color::Green :
                light->state == TrafficLight::YELLOW ? sf::Color::Yellow : sf::Color::Red
            );
            window.draw(lightCircle);
        }

        window.display();
    }
    #endif

    void printStatus() {
        stringstream ss;
        ss << "\nStep " << stepCount << " (Time: " << fixed << setprecision(2) << stepCount * config.simulationStep << " s):\n";
        ss << "Status: " << (isPaused ? "Paused" : "Running") << "\n";
        ss << "Vehicles:\n";
        for (auto* vehicle : vehicles) {
            ss << "  Vehicle " << vehicle->id << ": Road " << vehicle->currentRoad->id
               << ", Pos " << fixed << setprecision(1) << vehicle->position
               << ", Speed " << vehicle->speed << " m/s, Lane " << vehicle->currentLane
               << ", " << (vehicle->isStopped ? "Stopped" : "Moving")
               << (vehicle->hasCollided ? ", Collided" : "")
               << (vehicle->isEmergency ? ", Emergency" : "")
               << ", Wait Time: " << vehicle->timeWaiting << " s\n";
        }
        ss << "Pedestrians:\n";
        for (auto* pedestrian : pedestrians) {
            ss << "  Pedestrian " << pedestrian->id << ": Road " << pedestrian->currentRoad->id
               << ", Pos " << fixed << setprecision(1) << pedestrian->position
               << ", " << (pedestrian->isWaiting ? "Waiting" : "Moving")
               << ", Wait Time: " << pedestrian->timeWaiting << " s\n";
        }
        ss << "Traffic Lights:\n";
        for (auto* light : trafficLights) {
            ss << "  Intersection " << light->intersection->id << ": " << light->getStateString()
               << ", Waiting Vehicles: " << light->waitingVehicles
               << ", Waiting Pedestrians: " << light->waitingPedestrians << "\n";
        }
        ss << "Roads:\n";
        for (auto* road : graph.roads) {
            ss << "  Road " << road->id << ": Congestion " << fixed << setprecision(2) << road->congestion
               << ", Vehicles: " << road->vehiclesOnRoad.size()
               << ", Pedestrians: " << road->pedestriansOnRoad.size()
               << ", " << (road->isBlocked ? "Blocked" : "Open") << "\n";
        }
        ss << "Events:\n";
        for (auto* event : events) {
            ss << "  Event on Road " << event->road->id << ": Type " 
               << (event->type == Event::ACCIDENT ? "Accident" :
                   event->type == Event::CONSTRUCTION ? "Construction" :
                   event->type == Event::EMERGENCY_VEHICLE ? "Emergency Vehicle" : "Traffic Jam")
               << ", Time Left: " << fixed << setprecision(1) << (event->startTime + event->duration - stepCount * config.simulationStep) << " s\n";
        }
        logger.log(ss.str());
    }

    void exportAnalytics() {
        string dir = config.analyticsDir;
        system(("mkdir " + dir).c_str());

        ofstream vehicleFile(dir + "vehicles_" + config.scenarioName + ".csv");
        if (vehicleFile.is_open()) {
            vehicleFile << "VehicleID,RoadID,Position,Speed,Lane,Status,WaitTime,TravelTime,DistanceTraveled,IsEmergency\n";
            for (auto* vehicle : vehicles) {
                vehicleFile << vehicle->id << "," << vehicle->currentRoad->id << ","
                           << vehicle->position << "," << vehicle->speed << ","
                           << vehicle->currentLane << "," << (vehicle->isStopped ? "Stopped" : "Moving") << ","
                           << vehicle->timeWaiting << "," << vehicle->timeTraveling << ","
                           << vehicle->distanceTraveled << "," << (vehicle->isEmergency ? "Yes" : "No") << "\n";
            }
            vehicleFile.close();
        } else {
            logger.error("Could not create vehicle analytics file");
        }

        ofstream pedestrianFile(dir + "pedestrians_" + config.scenarioName + ".csv");
        if (pedestrianFile.is_open()) {
            pedestrianFile << "PedestrianID,RoadID,Position,Status,WaitTime,DistanceTraveled\n";
            for (auto* pedestrian : pedestrians) {
                pedestrianFile << pedestrian->id << "," << pedestrian->currentRoad->id << ","
                              << pedestrian->position << "," << (pedestrian->isWaiting ? "Waiting" : "Moving") << ","
                              << pedestrian->timeWaiting << "," << pedestrian->distanceTraveled << "\n";
            }
            pedestrianFile.close();
        } else {
            logger.error("Could not create pedestrian analytics file");
        }

        ofstream roadFile(dir + "roads_" + config.scenarioName + ".csv");
        if (roadFile.is_open()) {
            roadFile << "RoadID,Congestion,Vehicles,Pedestrians,Blocked\n";
            for (auto* road : graph.roads) {
                roadFile << road->id << "," << road->congestion << ","
                         << road->vehiclesOnRoad.size() << "," << road->pedestriansOnRoad.size() << ","
                         << (road->isBlocked ? "Yes" : "No") << "\n";
            }
            roadFile.close();
        } else {
            logger.error("Could not create road analytics file");
        }

        logger.log("Exported analytics to " + dir);
    }

    void run() {
        logger.log("Starting simulation (Scenario: " + config.scenarioName + "). Press 'P' to pause/resume, 'Q' to quit.");
        cout << "Progress: [";

        unique_lock<mutex> lock(simulationMutex);
        auto startTime = chrono::steady_clock::now();
        int progressInterval = config.maxSimulationSteps / 20;

        while (stepCount < config.maxSimulationSteps && !shouldStop) {
            if (isPaused) {
                simulationCV.wait(lock);
                continue;
            }

            double elapsedTime = chrono::duration_cast<chrono::seconds>(
                chrono::steady_clock::now() - startTime
            ).count();

            if (elapsedTime > config.maxSimulationTime) {
                logger.log("Simulation stopped: Maximum time limit reached (" + 
                           to_string(config.maxSimulationTime) + " s)");
                break;
            }

            #if USE_SFML_VISUALIZATION
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    shouldStop = true;
                    window.close();
                    logger.log("Simulation stopped: Window closed");
                }
            }
            #endif

            simulateEvents();
            simulateCollisions();
            optimizeTrafficLights();
            updateTrafficLights();
            updateVehicles();
            updatePedestrians();
            checkForStalledVehicles();

            if (stepCount % config.logFrequency == 0) {
                printStatus();
                exportAnalytics();
            }

            if (stepCount % progressInterval == 0) {
                cout << "=";
                cout.flush();
            }

            #if USE_SFML_VISUALIZATION
            if (window.isOpen()) render();
            #endif

            stepCount++;
            lock.unlock();
            this_thread::sleep_for(chrono::milliseconds(static_cast<int>(config.simulationStep * 1000)));
            lock.lock();
        }

        cout << "] 100%\n";
        printStatistics();
        exportAnalytics();
        logger.log("Simulation completed after " + to_string(stepCount) + " steps");

        #if USE_SFML_VISUALIZATION
        if (window.isOpen()) window.close();
        #endif
    }

    const SimulationConfig& getConfig() const { return config; }

    // Unit tests (disabled by default)
    void runTests() {
        logger.log("Running unit tests...");
        // Test Dijkstra's algorithm
        auto path = graph.dijkstra(graph.intersections[0], graph.intersections[1]);
        logger.log("Dijkstra test: Path size = " + to_string(path.size()));
        // Test traffic light
        TrafficLight light(graph.intersections[0], 10.0, 2.0, 10.0);
        light.update(15.0, 0, 0);
        logger.log("Traffic light test: State = " + light.getStateString());
        logger.log("Unit tests completed");
    }
};

// Command-line interface
class CommandLineInterface {
public:
    static void runInteractive(TrafficSimulation& sim) {
        cout << "Traffic Simulation Interactive Mode\n";
        cout << "Commands: start, pause, resume, config, test, exit\n";

        string command;
        while (true) {
            cout << "> ";
            getline(cin, command);
            transform(command.begin(), command.end(), command.begin(), ::tolower);

            if (command == "start") {
                sim.run();
            } else if (command == "pause") {
                sim.isPaused = true;
                cout << "Simulation paused\n";
            } else if (command == "resume") {
                sim.isPaused = false;
                sim.simulationCV.notify_all();
                cout << "Simulation resumed\n";
            } else if (command == "config") {
                cout << "Enter config file path (or press Enter for default): ";
                string configFile;
                getline(cin, configFile);
                if (configFile.empty()) configFile = "traffic_sim_config.txt";
                TrafficSimulation newSim(configFile);
                sim = move(newSim);
                cout << "Loaded new configuration: " << sim.getConfig().scenarioName << "\n";
            } else if (command == "test") {
                sim.runTests();
            } else if (command == "exit") {
                cout << "Exiting interactive mode\n";
                break;
            } else {
                cout << "Unknown command. Available: start, pause, resume, config, test, exit\n";
            }
        }
    }
};

// Main function
int main() {
    cout << "Traffic Simulation System v3.0\n";
    cout << "Initializing...\n";

    TrafficSimulation sim("traffic_sim_config.txt");
    
    cout << "Choose mode: (1) Run Simulation, (2) Interactive Mode\n";
    int choice;
    cin >> choice;
    cin.ignore(numeric_limits<streamsize>::max(), '\n');

    if (choice == 2) {
        CommandLineInterface::runInteractive(sim);
    } else {
        sim.run();
    }

    cout << "Simulation complete. Logs saved to " << sim.getConfig().logFile << "\n";
    cout << "Errors logged to " << sim.getConfig().errorLogFile << "\n";
    cout << "Analytics saved to " << sim.getConfig().analyticsDir << "\n";
    return 0;
}