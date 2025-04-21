#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <string>
#include <limits>
#include <algorithm>
#include <random>
#include <sstream>
#include <iomanip>

// Constants for simulation
const int WINDOW_WIDTH = 1000;
const int WINDOW_HEIGHT = 800;
const int GRID_SIZE = 6; // 6x6 grid
const float ROAD_WIDTH = 20.0f;
const float INTERSECTION_SIZE = 15.0f;
const double SIMULATION_TIME = 60.0; // 60 seconds

// Enum for weather conditions
enum class WeatherCondition {
    CLEAR,
    RAIN,
    FOG
};

// Intersection class
class Intersection {
public:
    int id;
    double x, y;
    class TrafficLight* traffic_light;
    std::map<Road*, std::string> incoming_road_labels;
    std::map<Road*, std::queue<class Vehicle*>> queues;

    Intersection(int id, double x, double y) : id(id), x(x), y(y), traffic_light(nullptr) {}

    // Get the number of vehicles waiting at this intersection
    int get_queue_size() const {
        int total = 0;
        for (const auto& [road, queue] : queues) {
            total += queue.size();
        }
        return total;
    }

    // Check if the intersection is congested
    bool is_congested() const {
        return get_queue_size() > 5;
    }
};

// Road class
class Road {
public:
    int from, to;
    double length;
    double speed_limit;
    int num_lanes;
    bool blocked;
    std::vector<std::vector<std::pair<Vehicle*, double>>> lanes;

    Road(int from, int to, double length, double speed_limit, int num_lanes)
        : from(from), to(to), length(length), speed_limit(speed_limit), num_lanes(num_lanes),
          blocked(false), lanes(num_lanes) {}

    // Add a vehicle to a specific lane
    void add_vehicle(Vehicle* vehicle, int lane) {
        if (lane >= 0 && lane < num_lanes) {
            lanes[lane].push_back({vehicle, vehicle->position});
            vehicle->current_road = this;
            vehicle->current_lane = lane;
        }
    }

    // Get the number of vehicles on the road
    int get_vehicle_count() const {
        int count = 0;
        for (const auto& lane : lanes) {
            count += lane.size();
        }
        return count;
    }

    // Check if the road is congested
    bool is_congested() const {
        return get_vehicle_count() > num_lanes * 3;
    }
};

// TrafficLight class
class TrafficLight {
public:
    enum State { RED, YELLOW, GREEN };
    State current_state;
    int phase_duration;
    int time_in_phase;
    int offset;

    TrafficLight(int duration, int offset)
        : current_state(RED), phase_duration(duration), time_in_phase(0), offset(offset) {}

    // Update the traffic light state
    void update(double delta_time, int vehicle_density) {
        time_in_phase += static_cast<int>(delta_time * 1000);
        int adjusted_duration = phase_duration;
        if (vehicle_density > 5) adjusted_duration = phase_duration * 0.8; // Shorten phase for high density
        if (time_in_phase >= adjusted_duration) {
            time_in_phase = 0;
            switch (current_state) {
                case RED: current_state = GREEN; break;
                case GREEN: current_state = YELLOW; break;
                case YELLOW: current_state = RED; break;
            }
        }
    }

    // Check if movement is allowed
    bool allows_movement() const {
        return current_state == GREEN;
    }

    // Get allowed directions
    std::vector<std::string> get_allowed_labels(int time) {
        int adjusted_time = (time + offset) % (2 * phase_duration);
        int phase = (adjusted_time / phase_duration) % 2;
        return phase == 0 ? std::vector<std::string>{"north", "south"} : std::vector<std::string>{"east", "west"};
    }
};

// Vehicle base class
class Vehicle {
public:
    int id;
    std::vector<int> path;
    int current_index;
    Road* current_road;
    int current_lane;
    double position;
    double speed;
    double max_speed;
    sf::Color color;
    sf::Vector2f size;
    double acceleration;
    bool emergency;

    Vehicle(int id, const std::vector<int>& path, double max_speed, sf::Color color, sf::Vector2f size, bool emergency = false)
        : id(id), path(path), current_index(0), current_road(nullptr), current_lane(0),
          position(0), speed(0), max_speed(max_speed), color(color), size(size), acceleration(0), emergency(emergency) {}

    virtual ~Vehicle() = default;

    // Update vehicle position
    virtual void update(double delta_time, WeatherCondition weather) {
        double weather_factor = (weather == WeatherCondition::RAIN) ? 0.8 : (weather == WeatherCondition::FOG) ? 0.6 : 1.0;
        speed += acceleration * delta_time;
        if (speed > max_speed * weather_factor) speed = max_speed * weather_factor;
        if (speed < 0) speed = 0;
        position += speed * delta_time;
    }

    // Check for collision risk
    bool check_collision(const std::vector<std::pair<Vehicle*, double>>& lane) {
        for (const auto& [other, pos] : lane) {
            if (other != this && std::abs(pos - position) < size.x) {
                return true;
            }
        }
        return false;
    }

    // Attempt to change lane
    bool change_lane(Road* road, int new_lane) {
        if (new_lane >= 0 && new_lane < road->num_lanes && !check_collision(road->lanes[new_lane])) {
            road->lanes[current_lane].erase(
                std::remove_if(road->lanes[current_lane].begin(), road->lanes[current_lane].end(),
                    [this](const auto& p) { return p.first == this; }),
                road->lanes[current_lane].end());
            road->add_vehicle(this, new_lane);
            return true;
        }
        return false;
    }

    // Get vehicle shape for rendering
    sf::RectangleShape get_shape() const {
        sf::RectangleShape shape(size);
        shape.setFillColor(color);
        return shape;
    }
};

// Derived vehicle classes
class Car : public Vehicle {
public:
    Car(int id, const std::vector<int>& path)
        : Vehicle(id, path, 12.0, sf::Color::Green, sf::Vector2f(20, 10)) {}
};

class Truck : public Vehicle {
public:
    Truck(int id, const std::vector<int>& path)
        : Vehicle(id, path, 8.0, sf::Color::Blue, sf::Vector2f(30, 15)) {}
};

class Motorcycle : public Vehicle {
public:
    Motorcycle(int id, const std::vector<int>& path)
        : Vehicle(id, path, 15.0, sf::Color::Red, sf::Vector2f(10, 5)) {}
};

class Bus : public Vehicle {
public:
    Bus(int id, const std::vector<int>& path)
        : Vehicle(id, path, 6.0, sf::Color::Yellow, sf::Vector2f(40, 20)) {}
};

class EmergencyVehicle : public Vehicle {
public:
    EmergencyVehicle(int id, const std::vector<int>& path)
        : Vehicle(id, path, 20.0, sf::Color::Magenta, sf::Vector2f(25, 12), true) {}
};

// Pedestrian class
class Pedestrian {
public:
    sf::Vector2f position;
    sf::Vector2f destination;
    double speed;
    sf::Color color;
    bool crossing;

    Pedestrian(sf::Vector2f pos, sf::Vector2f dest)
        : position(pos), destination(dest), speed(2.0), color(sf::Color::White), crossing(false) {}

    // Update pedestrian position
    void update(double delta_time) {
        sf::Vector2f direction = destination - position;
        float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
        if (length > 0) {
            direction /= length;
            position += direction * static_cast<float>(speed * delta_time);
        }
    }

    // Get pedestrian shape for rendering
    sf::RectangleShape get_shape() const {
        sf::RectangleShape shape(sf::Vector2f(5, 5));
        shape.setFillColor(color);
        shape.setPosition(position);
        return shape;
    }

    // Check if pedestrian is at crosswalk
    bool is_at_crosswalk(const sf::Vector2f& crosswalk_pos) const {
        return std::abs(position.x - crosswalk_pos.x) < 10 && std::abs(position.y - crosswalk_pos.y) < 10;
    }
};

// Accident class
class Accident {
public:
    Road* road;
    double position;
    int duration;
    int time_remaining;

    Accident(Road* road, double position, int duration)
        : road(road), position(position), duration(duration), time_remaining(duration) {}

    // Update accident status
    void update(double delta_time) {
        time_remaining -= static_cast<int>(delta_time * 1000);
        if (time_remaining <= 0) {
            road->blocked = false;
        }
    }

    // Get accident shape for rendering
    sf::RectangleShape get_shape() const {
        sf::RectangleShape shape(sf::Vector2f(20, 20));
        shape.setFillColor(sf::Color::Red);
        shape.setPosition(static_cast<float>(position - 10), 0);
        return shape;
    }
};

// RoadNetwork class
class RoadNetwork {
public:
    std::vector<Intersection*> intersections;
    std::vector<Road*> roads;
    std::map<std::pair<int, int>, Road*> road_map;

    // Add an intersection
    void add_intersection(int id, double x, double y) {
        intersections.push_back(new Intersection(id, x, y));
    }

    // Add a road
    void add_road(int from, int to, double length, double speed_limit, int num_lanes) {
        Road* road = new Road(from, to, length, speed_limit, num_lanes);
        roads.push_back(road);
        road_map[{from, to}] = road;
    }

    // Setup traffic lights with random offsets
    void setup_traffic_lights() {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 20);
        for (auto inter : intersections) {
            int offset = dis(gen);
            inter->traffic_light = new TrafficLight(20000, offset); // 20s phases
            for (auto road : roads) {
                if (road->to == inter->id) {
                    Intersection* from_inter = nullptr;
                    for (auto i : intersections) {
                        if (i->id == road->from) {
                            from_inter = i;
                            break;
                        }
                    }
                    double dx = from_inter->x - inter->x;
                    double dy = from_inter->y - inter->y;
                    if (std::abs(dx) > std::abs(dy)) {
                        inter->incoming_road_labels[road] = (dx > 0) ? "west" : "east";
                    } else {
                        inter->incoming_road_labels[road] = (dy > 0) ? "south" : "north";
                    }
                }
            }
        }
    }

    // Compute shortest path using Dijkstra's algorithm
    std::vector<int> shortest_path(int start, int end) {
        std::map<int, double> dist;
        std::map<int, int> prev;
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

        for (auto inter : intersections) {
            dist[inter->id] = std::numeric_limits<double>::infinity();
        }
        dist[start] = 0;
        pq.push({0, start});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();
            if (u == end) break;
            for (auto road : roads) {
                if (road->from == u && !road->blocked) {
                    double weight = road->length / road->speed_limit;
                    if (road->is_congested()) weight *= 1.5; // Penalty for congestion
                    int v = road->to;
                    if (dist[u] + weight < dist[v]) {
                        dist[v] = dist[u] + weight;
                        prev[v] = u;
                        pq.push({dist[v], v});
                    }
                }
            }
        }

        std::vector<int> path;
        for (int at = end; at != start; at = prev[at]) {
            path.push_back(at);
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }

    // Get road between two intersections
    Road* get_road(int from, int to) {
        auto it = road_map.find({from, to});
        return it != road_map.end() ? it->second : nullptr;
    }
};

// Simulation class
class Simulation {
public:
    double current_time;
    RoadNetwork* road_network;
    std::vector<Vehicle*> vehicles;
    std::vector<Pedestrian> pedestrians;
    std::vector<Accident> accidents;
    std::random_device rd;
    std::mt19937 gen;
    sf::RenderWindow window;
    sf::Font font;
    int next_spawn_time;
    bool paused;
    double simulation_speed;
    WeatherCondition weather;
    std::vector<double> average_speeds;
    int total_vehicles;

    Simulation(RoadNetwork* rn)
        : current_time(0), road_network(rn), gen(rd()), window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Traffic Simulation"),
          next_spawn_time(0), paused(false), simulation_speed(1.0), weather(WeatherCondition::CLEAR), total_vehicles(0) {
        if (!font.loadFromFile("arial.ttf")) {
            std::cerr << "Error loading font" << std::endl;
        }
    }

    // Spawn a vehicle with random type and path
    void spawn_vehicle() {
        int id = vehicles.size() + 1;
        int start = std::uniform_int_distribution<>(0, GRID_SIZE * GRID_SIZE - 1)(gen);
        int end = std::uniform_int_distribution<>(0, GRID_SIZE * GRID_SIZE - 1)(gen);
        while (end == start) end = std::uniform_int_distribution<>(0, GRID_SIZE * GRID_SIZE - 1)(gen);
        std::vector<int> path = road_network->shortest_path(start, end);
        if (path.size() < 2) return;

        int type_idx = std::uniform_int_distribution<>(0, 4)(gen);
        Vehicle* veh;
        switch (type_idx) {
            case 0: veh = new Car(id, path); break;
            case 1: veh = new Truck(id, path); break;
            case 2: veh = new Motorcycle(id, path); break;
            case 3: veh = new Bus(id, path); break;
            case 4: veh = new EmergencyVehicle(id, path); break;
            default: veh = new Car(id, path);
        }
        vehicles.push_back(veh);
        Road* first_road = road_network->get_road(path[0], path[1]);
        if (first_road) {
            veh->current_road = first_road;
            first_road->add_vehicle(veh, 0);
            total_vehicles++;
            std::cout << "Time " << current_time << ": " << vehicle_type_to_string(type_idx) << " " << id
                      << " spawned on road " << path[0] << " to " << path[1] << " (speed: " << veh->max_speed << " m/s)" << std::endl;
        }
    }

    // Convert vehicle type index to string
    std::string vehicle_type_to_string(int type_idx) {
        switch (type_idx) {
            case 0: return "Car";
            case 1: return "Truck";
            case 2: return "Motorcycle";
            case 3: return "Bus";
            case 4: return "Emergency Vehicle";
            default: return "Unknown";
        }
    }

    // Spawn a pedestrian
    void spawn_pedestrian() {
        float x = std::uniform_real_distribution<float>(0, WINDOW_WIDTH)(gen);
        float y = std::uniform_real_distribution<float>(0, WINDOW_HEIGHT)(gen);
        float dest_x = std::uniform_real_distribution<float>(0, WINDOW_WIDTH)(gen);
        float dest_y = std::uniform_real_distribution<float>(0, WINDOW_HEIGHT)(gen);
        pedestrians.emplace_back(sf::Vector2f(x, y), sf::Vector2f(dest_x, dest_y));
    }

    // Spawn an accident
    void spawn_accident() {
        int road_idx = std::uniform_int_distribution<>(0, road_network->roads.size() - 1)(gen);
        Road* road = road_network->roads[road_idx];
        if (!road->blocked) {
            road->blocked = true;
            double pos = std::uniform_real_distribution<>(0, road->length)(gen);
            accidents.emplace_back(road, pos, 10000); // 10s duration
            std::cout << "Time " << current_time << ": Accident on road " << road->from << " to " << road->to << std::endl;
            reroute_vehicles();
        }
    }

    // Update vehicles
    void update_vehicles(double delta_time) {
        for (auto it = vehicles.begin(); it != vehicles.end();) {
            Vehicle* veh = *it;
            if (veh->current_road && !veh->current_road->blocked) {
                veh->update(delta_time, weather);
                if (veh->position >= veh->current_road->length) {
                    Intersection* inter = nullptr;
                    for (auto i : road_network->intersections) {
                        if (i->id == veh->current_road->to) {
                            inter = i;
                            break;
                        }
                    }
                    inter->queues[veh->current_road].push(veh);
                    veh->position = veh->current_road->length;
                }
                // Attempt lane change
                if (std::uniform_real_distribution<>(0, 1)(gen < 0.01)) {
                    int new_lane = veh->current_lane + (std::uniform_int_distribution<>(0, 1)(gen) ? 1 : -1);
                    veh->change_lane(veh->current_road, new_lane);
                }
                ++it;
            } else {
                ++it;
            }
        }
        // Remove vehicles from roads
        for (auto road : road_network->roads) {
            for (auto& lane : road->lanes) {
                lane.erase(
                    std::remove_if(lane.begin(), lane.end(),
                        [road](const auto& p) { return p.second >= road->length; }),
                    lane.end());
            }
        }
    }

    // Process intersections
    void process_intersections() {
        for (auto inter : road_network->intersections) {
            if (!inter->traffic_light) continue;
            inter->traffic_light->update(0.1, inter->get_queue_size());
            auto allowed = inter->traffic_light->get_allowed_labels(static_cast<int>(current_time * 1000));
            for (auto& [road, q] : inter->queues) {
                std::string label = inter->incoming_road_labels[road];
                bool pedestrian_safe = true;
                for (const auto& ped : pedestrians) {
                    sf::Vector2f crosswalk_pos(inter->x * 160 + 80, inter->y * 160 + 80);
                    if (ped.is_at_crosswalk(crosswalk_pos)) {
                        pedestrian_safe = false;
                        break;
                    }
                }
                if (inter->traffic_light->allows_movement() && pedestrian_safe &&
                    std::find(allowed.begin(), allowed.end(), label) != allowed.end() && !q.empty()) {
                    Vehicle* veh = q.front();
                    q.pop();
                    veh->current_index++;
                    if (veh->current_index < veh->path.size() - 1) {
                        int next_from = veh->path[veh->current_index];
                        int next_to = veh->path[veh->current_index + 1];
                        Road* next_road = road_network->get_road(next_from, next_to);
                        if (next_road) {
                            veh->current_road = next_road;
                            veh->position = 0;
                            next_road->add_vehicle(veh, veh->current_lane);
                            std::cout << "Time " << current_time << ": Vehicle " << veh->id << " moved to road "
                                      << next_from << " to " << next_to << std::endl;
                        }
                    } else {
                        std::cout << "Time " << current_time << ": Vehicle " << veh->id << " reached destination" << std::endl;
                        vehicles.erase(std::remove(vehicles.begin(), vehicles.end(), veh), vehicles.end());
                        delete veh;
                    }
                }
            }
        }
    }

    // Update pedestrians
    void update_pedestrians(double delta_time) {
        for (auto& ped : pedestrians) {
            ped.update(delta_time);
        }
        // Remove pedestrians that reached destination
        pedestrians.erase(
            std::remove_if(pedestrians.begin(), pedestrians.end(),
                [](const Pedestrian& ped) {
                    return std::abs(ped.position.x - ped.destination.x) < 5 &&
                           std::abs(ped.position.y - ped.destination.y) < 5;
                }),
            pedestrians.end());
    }

    // Update accidents
    void update_accidents(double delta_time) {
        for (auto it = accidents.begin(); it != accidents.end();) {
            it->update(delta_time);
            if (it->time_remaining <= 0) {
                it = accidents.erase(it);
            } else {
                ++it;
            }
        }
        if (std::uniform_real_distribution<>(0, 1)(gen) < 0.005) {
            spawn_accident();
        }
    }

    // Reroute vehicles affected by accidents
    void reroute_vehicles() {
        for (auto& veh : vehicles) {
            if (veh->current_road && veh->current_road->blocked) {
                int start = veh->path[veh->current_index];
                int end = veh->path.back();
                veh->path = road_network->shortest_path(start, end);
                veh->current_index = 0;
                Road* new_road = road_network->get_road(veh->path[0], veh->path[1]);
                if (new_road) {
                    new_road->add_vehicle(veh, veh->current_lane);
                    std::cout << "Time " << current_time << ": Vehicle " << veh->id << " rerouted" << std::endl;
                }
            }
        }
    }

    // Update weather conditions
    void update_weather() {
        double chance = std::uniform_real_distribution<>(0, 1)(gen);
        if (chance < 0.01) {
            weather = WeatherCondition::RAIN;
            std::cout << "Time " << current_time << ": Weather changed to Rain" << std::endl;
        } else if (chance < 0.02) {
            weather = WeatherCondition::FOG;
            std::cout << "Time " << current_time << ": Weather changed to Fog" << std::endl;
        } else if (chance < 0.05) {
            weather = WeatherCondition::CLEAR;
            std::cout << "Time " << current_time << ": Weather changed to Clear" << std::endl;
        }
    }

    // Collect traffic data
    void collect_data() {
        double total_speed = 0;
        int moving_vehicles = 0;
        for (const auto& veh : vehicles) {
            if (veh->speed > 0) {
                total_speed += veh->speed;
                moving_vehicles++;
            }
        }
        if (moving_vehicles > 0) {
            average_speeds.push_back(total_speed / moving_vehicles);
        } else {
            average_speeds.push_back(0);
        }
    }

    // Run the simulation
    void run() {
        while (current_time < SIMULATION_TIME && window.isOpen()) {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) window.close();
                else if (event.type == sf::Event::KeyPressed) {
                    if (event.key.code == sf::Keyboard::Space) paused = !paused;
                    else if (event.key.code == sf::Keyboard::Up) simulation_speed += 0.1;
                    else if (event.key.code == sf::Keyboard::Down) simulation_speed = std::max(0.1, simulation_speed - 0.1);
                }
            }

            if (!paused) {
                double delta_time = 0.1 * simulation_speed;
                if (current_time >= next_spawn_time) {
                    spawn_vehicle();
                    if (std::uniform_real_distribution<>(0, 1)(gen) < 0.2) spawn_pedestrian();
                    next_spawn_time = current_time + std::uniform_real_distribution<>(1, 5)(gen);
                }
                update_vehicles(delta_time);
                process_intersections();
                update_pedestrians(delta_time);
                update_accidents(delta_time);
                update_weather();
                collect_data();
                current_time += delta_time;
            }

            render();
            sf::sleep(sf::milliseconds(50));
        }
    }

    // Render the simulation
    void render() {
        window.clear(sf::Color::Black);

        // Draw roads
        for (auto road : road_network->roads) {
            Intersection* from = nullptr;
            Intersection* to = nullptr;
            for (auto inter : road_network->intersections) {
                if (inter->id == road->from) from = inter;
                if (inter->id == road->to) to = inter;
            }
            if (from && to) {
                for (int lane = 0; lane < road->num_lanes; lane++) {
                    sf::Color road_color = road->blocked ? sf::Color::Red :
                                          road->is_congested() ? sf::Color::Yellow : sf::Color::Magenta;
                    sf::Vertex line[] = {
                        sf::Vertex(sf::Vector2f(from->x * 160 + 80, from->y * 160 + 80 + lane * ROAD_WIDTH), road_color),
                        sf::Vertex(sf::Vector2f(to->x * 160 + 80, to->y * 160 + 80 + lane * ROAD_WIDTH), road_color)
                    };
                    window.draw(line, 2, sf::Lines);
                }
            }
        }

        // Draw intersections
        for (auto inter : road_network->intersections) {
            sf::CircleShape circle(INTERSECTION_SIZE);
            circle.setFillColor(inter->traffic_light->allows_movement() ? sf::Color::Green : sf::Color::Red);
            circle.setPosition(inter->x * 160 + 80 - INTERSECTION_SIZE, inter->y * 160 + 80 - INTERSECTION_SIZE);
            window.draw(circle);

            // Draw queue count
            sf::Text text;
            text.setFont(font);
            text.setString(std::to_string(inter->get_queue_size()));
            text.setCharacterSize(12);
            text.setFillColor(sf::Color::White);
            text.setPosition(inter->x * 160 + 80, inter->y * 160 + 80);
            window.draw(text);
        }

        // Draw vehicles
        for (auto veh : vehicles) {
            if (veh->current_road) {
                Intersection* from = nullptr;
                Intersection* to = nullptr;
                for (auto inter : road_network->intersections) {
                    if (inter->id == veh->current_road->from) from = inter;
                    if (inter->id == veh->current_road->to) to = inter;
                }
                if (from && to) {
                    double ratio = veh->position / veh->current_road->length;
                    double x = from->x * 160 + 80 + ratio * (to->x * 160 + 80 - from->x * 160 - 80);
                    double y = from->y * 160 + 80 + ratio * (to->y * 160 + 80 - from->y * 160 - 80) + veh->current_lane * ROAD_WIDTH;
                    sf::RectangleShape rect = veh->get_shape();
                    rect.setPosition(static_cast<float>(x - veh->size.x / 2), static_cast<float>(y - veh->size.y / 2));
                    window.draw(rect);
                }
            } else {
                int inter_id = veh->path[veh->current_index];
                Intersection* inter = nullptr;
                for (auto i : road_network->intersections) {
                    if (i->id == inter_id) {
                        inter = i;
                        break;
                    }
                }
                if (inter) {
                    sf::RectangleShape rect = veh->get_shape();
                    rect.setPosition(inter->x * 160 + 80 - veh->size.x / 2, inter->y * 160 + 80 - veh->size.y / 2);
                    window.draw(rect);
                }
            }
        }

        // Draw pedestrians
        for (const auto& ped : pedestrians) {
            window.draw(ped.get_shape());
        }

        // Draw accidents
        for (const auto& acc : accidents) {
            Intersection* from = nullptr;
            for (auto inter : road_network->intersections) {
                if (inter->id == acc.road->from) {
                    from = inter;
                    break;
                }
            }
            if (from) {
                sf::RectangleShape shape = acc.get_shape();
                shape.setPosition(from->x * 160 + 80 + static_cast<float>(acc.position) - 10, from->y * 160 + 80);
                window.draw(shape);
            }
        }

        // Draw mini-map
        sf::RectangleShape mini_map(sf::Vector2f(200, 200));
        mini_map.setFillColor(sf::Color::Black);
        mini_map.setOutlineColor(sf::Color::White);
        mini_map.setOutlineThickness(2);
        mini_map.setPosition(WINDOW_WIDTH - 210, 10);
        window.draw(mini_map);
        for (auto road : road_network->roads) {
            Intersection* from = nullptr;
            Intersection* to = nullptr;
            for (auto inter : road_network->intersections) {
                if (inter->id == road->from) from = inter;
                if (inter->id == road->to) to = inter;
            }
            if (from && to) {
                sf::Vertex line[] = {
                    sf::Vertex(sf::Vector2f(WINDOW_WIDTH - 210 + from->x * 33 + 10, 20 + from->y * 33), sf::Color::White),
                    sf::Vertex(sf::Vector2f(WINDOW_WIDTH - 210 + to->x * 33 + 10, 20 + to->y * 33), sf::Color::White)
                };
                window.draw(line, 2, sf::Lines);
            }
        }

        // Draw stats
        std::stringstream stats;
        stats << "Time: " << std::fixed << std::setprecision(1) << current_time << " s\n"
              << "Vehicles: " << vehicles.size() << "\n"
              << "Total Vehicles: " << total_vehicles << "\n"
              << "Weather: " << weather_to_string() << "\n"
              << "Avg Speed: " << (average_speeds.empty() ? 0 : average_speeds.back()) << " m/s\n"
              << "Accidents: " << accidents.size() << "\n"
              << "Pedestrians: " << pedestrians.size() << "\n"
              << "Paused: " << (paused ? "Yes" : "No") << "\n"
              << "Speed: " << simulation_speed << "x";
        sf::Text text;
        text.setFont(font);
        text.setString(stats.str());
        text.setCharacterSize(18);
        text.setFillColor(sf::Color::White);
        text.setPosition(10, 10);
        window.draw(text);

        // Draw control instructions
        sf::Text controls;
        controls.setFont(font);
        controls.setString("Controls:\nSpace: Pause/Resume\nUp: Increase Speed\nDown: Decrease Speed");
        controls.setCharacterSize(16);
        controls.setFillColor(sf::Color::White);
        controls.setPosition(10, WINDOW_HEIGHT - 100);
        window.draw(controls);

        window.display();
    }

    // Convert weather to string
    std::string weather_to_string() const {
        switch (weather) {
            case WeatherCondition::CLEAR: return "Clear";
            case WeatherCondition::RAIN: return "Rain";
            case WeatherCondition::FOG: return "Fog";
            default: return "Unknown";
        }
    }

    // Cleanup
    ~Simulation() {
        for (auto veh : vehicles) delete veh;
        for (auto road : road_network->roads) delete road;
        for (auto inter : road_network->intersections) {
            delete inter->traffic_light;
            delete inter;
        }
        delete road_network;
    }
};

// Main function
int main() {
    // Setup 6x6 grid
    RoadNetwork* rn = new RoadNetwork();
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            rn->add_intersection(i * GRID_SIZE + j, i, j);
        }
    }
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE - 1; j++) {
            int id1 = i * GRID_SIZE + j;
            int id2 = i * GRID_SIZE + j + 1;
            rn->add_road(id1, id2, 100, 10, 2);
            rn->add_road(id2, id1, 100, 10, 2);
        }
    }
    for (int j = 0; j < GRID_SIZE; j++) {
        for (int i = 0; i < GRID_SIZE - 1; i++) {
            int id1 = i * GRID_SIZE + j;
            int id2 = (i + 1) * GRID_SIZE + j;
            rn->add_road(id1, id2, 100, 10, 2);
            rn->add_road(id2, id1, 100, 10, 2);
        }
    }
    rn->setup_traffic_lights();

    Simulation sim(rn);
    sim.run();

    return 0;
}