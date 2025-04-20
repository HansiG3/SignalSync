#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <string>
#include <limits>
#include <algorithm>
#include <random>

using namespace std;

// Forward declarations
class Intersection;
class Road;
class TrafficLight;
class Vehicle;
class RoadNetwork;
class Simulation;

// Intersection class
class Intersection {
public:
    int id;
    double x, y;
    TrafficLight* traffic_light;
    map<Road*, string> incoming_road_labels;
    map<Road*, queue<Vehicle*>> queues;

    Intersection(int id, double x, double y) : id(id), x(x), y(y), traffic_light(nullptr) {}
};

// Road class
class Road {
public:
    int from, to;
    double length;
    double speed_limit;
    vector<pair<Vehicle*, double>> vehicles;

    Road(int from, int to, double length, double speed_limit)
        : from(from), to(to), length(length), speed_limit(speed_limit) {}
};

// TrafficLight class
class TrafficLight {
public:
    int phase_duration;
    int offset;

    TrafficLight(int duration, int offset) : phase_duration(duration), offset(offset) {}

    vector<string> get_allowed_labels(int time) {
        int adjusted_time = (time + offset) % (2 * phase_duration);
        int phase = (adjusted_time / phase_duration) % 2;
        return phase == 0 ? vector<string>{"north", "south"} : vector<string>{"east", "west"};
    }
};

// Vehicle class
class Vehicle {
public:
    int id;
    vector<int> path;
    int current_index;
    Road* current_road;
    double position;
    double speed;
    sf::Color color;
    sf::Vector2f size;

    Vehicle(int id, const vector<int>& path, double speed, sf::Color color, sf::Vector2f size)
        : id(id), path(path), current_index(0), current_road(nullptr), position(0), speed(speed), color(color), size(size) {}
};

// Derived vehicle classes
class Car : public Vehicle {
public:
    Car(int id, const vector<int>& path, mt19937& gen)
        : Vehicle(id, path, uniform_real_distribution<>(8.0, 12.0)(gen), sf::Color::Green, sf::Vector2f(20, 10)) {}
};

class Truck : public Vehicle {
public:
    Truck(int id, const vector<int>& path, mt19937& gen)
        : Vehicle(id, path, uniform_real_distribution<>(5.0, 8.0)(gen), sf::Color::Blue, sf::Vector2f(30, 15)) {}
};

class Motorcycle : public Vehicle {
public:
    Motorcycle(int id, const vector<int>& path, mt19937& gen)
        : Vehicle(id, path, uniform_real_distribution<>(10.0, 15.0)(gen), sf::Color::Red, sf::Vector2f(10, 5)) {}
};

// RoadNetwork class
class RoadNetwork {
public:
    vector<Intersection*> intersections;
    vector<Road*> roads;
    map<pair<int, int>, Road*> road_map;

    void add_intersection(int id, double x, double y) {
        intersections.push_back(new Intersection(id, x, y));
    }

    void add_road(int from, int to, double length, double speed_limit) {
        Road* road = new Road(from, to, length, speed_limit);
        roads.push_back(road);
        road_map[{from, to}] = road;
    }

    void setup_traffic_lights() {
        random_device rd;
        mt19937 gen(rd());
        uniform_int_distribution<> dis(0, 20);

        for (auto inter : intersections) {
            int offset = dis(gen);
            inter->traffic_light = new TrafficLight(20, offset);
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
                    if (abs(dx) > abs(dy)) {
                        inter->incoming_road_labels[road] = (dx > 0) ? "west" : "east";
                    } else {
                        inter->incoming_road_labels[road] = (dy > 0) ? "south" : "north";
                    }
                }
            }
        }
    }

    vector<int> shortest_path(int start, int end) {
        map<int, double> dist;
        map<int, int> prev;
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;

        for (auto inter : intersections) {
            dist[inter->id] = numeric_limits<double>::infinity();
        }
        dist[start] = 0;
        pq.push({0, start});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();
            if (u == end) break;
            for (auto road : roads) {
                if (road->from == u) {
                    double weight = road->length / road->speed_limit;
                    int v = road->to;
                    if (dist[u] + weight < dist[v]) {
                        dist[v] = dist[u] + weight;
                        prev[v] = u;
                        pq.push({dist[v], v});
                    }
                }
            }
        }

        vector<int> path;
        for (int at = end; at != start; at = prev[at]) {
            path.push_back(at);
        }
        path.push_back(start);
        reverse(path.begin(), path.end());
        return path;
    }

    Road* get_road(int from, int to) {
        auto it = road_map.find({from, to});
        return it != road_map.end() ? it->second : nullptr;
    }
};

// Simulation class with SFML
class Simulation {
public:
    int current_time;
    RoadNetwork* road_network;
    vector<Vehicle*> vehicles;
    random_device rd;
    mt19937 gen;
    sf::RenderWindow window;
    sf::Font font;
    int next_spawn_time;

    Simulation(RoadNetwork* rn) : current_time(0), road_network(rn), gen(rd()), window(sf::VideoMode(800, 800), "Traffic Simulation"), next_spawn_time(0) {
        if (!font.loadFromFile("arial.ttf")) {
            cout << "Error loading font" << endl;
        }
    }

    void spawn_vehicle(int id, int start, int end) {
        vector<int> path = road_network->shortest_path(start, end);
        if (path.size() < 2) return; // Invalid path
        int type_idx = uniform_int_distribution<>(0, 2)(gen);
        Vehicle* veh;
        if (type_idx == 0) veh = new Car(id, path, gen);
        else if (type_idx == 1) veh = new Truck(id, path, gen);
        else veh = new Motorcycle(id, path, gen);
        vehicles.push_back(veh);
        Road* first_road = road_network->get_road(path[0], path[1]);
        if (first_road) {
            veh->current_road = first_road;
            first_road->vehicles.push_back({veh, 0});
            cout << "Time " << current_time << ": " << vehicle_type_to_string(type_idx) << " " << id << " spawned on road " << path[0] << " to " << path[1]
                 << " (speed: " << veh->speed << " m/s)" << endl;
        }
    }

    string vehicle_type_to_string(int type_idx) {
        switch (type_idx) {
            case 0: return "Car";
            case 1: return "Truck";
            case 2: return "Motorcycle";
        }
        return "Unknown";
    }

    void update_vehicles_on_roads() {
        for (auto road : road_network->roads) {
            for (auto& veh_pair : road->vehicles) {
                Vehicle* veh = veh_pair.first;
                veh->position += veh->speed;
                veh_pair.second = veh->position;
                if (veh->position >= road->length) {
                    Intersection* inter = nullptr;
                    for (auto i : road_network->intersections) {
                        if (i->id == road->to) {
                            inter = i;
                            break;
                        }
                    }
                    inter->queues[road].push(veh);
                    veh->position = road->length;
                }
            }
            road->vehicles.erase(
                remove_if(road->vehicles.begin(), road->vehicles.end(),
                    [&](pair<Vehicle*, double>& p) { return p.second >= road->length; }),
                road->vehicles.end());
        }
    }

    void process_intersections() {
        for (auto inter : road_network->intersections) {
            if (!inter->traffic_light) continue;
            auto allowed = inter->traffic_light->get_allowed_labels(current_time);
            for (auto& [road, q] : inter->queues) {
                string label = inter->incoming_road_labels[road];
                if (find(allowed.begin(), allowed.end(), label) != allowed.end() && !q.empty()) {
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
                            next_road->vehicles.push_back({veh, 0});
                            cout << "Time " << current_time << ": Vehicle " << veh->id << " moved to road "
                                 << next_from << " to " << next_to << endl;
                        }
                    } else {
                        // Reached destination
                        cout << "Time " << current_time << ": Vehicle " << veh->id << " reached destination" << endl;
                        vehicles.erase(remove(vehicles.begin(), vehicles.end(), veh), vehicles.end());
                        delete veh;
                    }
                }
            }
        }
    }

    void run(int max_time) {
        while (current_time < max_time && window.isOpen()) {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) window.close();
            }

            if (current_time >= next_spawn_time) {
                int start = uniform_int_distribution<>(0, 24)(gen);
                int end = uniform_int_distribution<>(0, 24)(gen);
                while (end == start) end = uniform_int_distribution<>(0, 24)(gen);
                spawn_vehicle(vehicles.size() + 1, start, end);
                next_spawn_time = current_time + uniform_int_distribution<>(1, 5)(gen);
            }

            update_vehicles_on_roads();
            process_intersections();
            render();
            current_time++;

            sf::sleep(sf::milliseconds(100));
        }
    }

    void render() {
        window.clear(sf::Color::Black);

        // Draw roads with density coloring
        for (auto road : road_network->roads) {
            int num_vehicles = road->vehicles.size();
            sf::Color road_color = num_vehicles == 0 ? sf::Color::White : num_vehicles <= 2 ? sf::Color::Yellow : sf::Color::Red;
            Intersection* from = nullptr;
            Intersection* to = nullptr;
            for (auto inter : road_network->intersections) {
                if (inter->id == road->from) from = inter;
                if (inter->id == road->to) to = inter;
            }
            if (from && to) {
                sf::Vertex line[] = {
                    sf::Vertex(sf::Vector2f(from->x * 160 + 80, from->y * 160 + 80), road_color),
                    sf::Vertex(sf::Vector2f(to->x * 160 + 80, to->y * 160 + 80), road_color)
                };
                window.draw(line, 2, sf::Lines);
            }
        }

        // Draw intersections with traffic light status
        for (auto inter : road_network->intersections) {
            sf::CircleShape circle(10);
            auto allowed = inter->traffic_light->get_allowed_labels(current_time);
            circle.setFillColor(find(allowed.begin(), allowed.end(), "north") != allowed.end() ? sf::Color::Green : sf::Color::Red);
            circle.setPosition(inter->x * 160 + 70, inter->y * 160 + 70);
            window.draw(circle);
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
                    double y = from->y * 160 + 80 + ratio * (to->y * 160 + 80 - from->y * 160 - 80);
                    sf::RectangleShape rect(veh->size);
                    rect.setFillColor(veh->color);
                    rect.setPosition(x - veh->size.x / 2, y - veh->size.y / 2);
                    window.draw(rect);
                }
            } else {
                // Waiting at intersection
                int inter_id = veh->path[veh->current_index];
                Intersection* inter = nullptr;
                for (auto i : road_network->intersections) {
                    if (i->id == inter_id) {
                        inter = i;
                        break;
                    }
                }
                if (inter) {
                    sf::RectangleShape rect(veh->size);
                    rect.setFillColor(veh->color);
                    rect.setPosition(inter->x * 160 + 80 - veh->size.x / 2, inter->y * 160 + 80 - veh->size.y / 2);
                    window.draw(rect);
                }
            }
        }

        // Draw simulation stats
        sf::Text text;
        text.setFont(font);
        text.setString("Time: " + to_string(current_time) + " s\nVehicles: " + to_string(vehicles.size()));
        text.setCharacterSize(24);
        text.setFillColor(sf::Color::White);
        text.setPosition(10, 10);
        window.draw(text);

        window.display();
    }

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

int main() {
    // 5x5 grid setup
    RoadNetwork* rn = new RoadNetwork();
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            rn->add_intersection(i * 5 + j, i, j);
        }
    }
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 4; j++) {
            int id1 = i * 5 + j;
            int id2 = i * 5 + j + 1;
            rn->add_road(id1, id2, 100, 10);
            rn->add_road(id2, id1, 100, 10);
        }
    }
    for (int j = 0; j < 5; j++) {
        for (int i = 0; i < 4; i++) {
            int id1 = i * 5 + j;
            int id2 = (i + 1) * 5 + j;
            rn->add_road(id1, id2, 100, 10);
            rn->add_road(id2, id1, 100, 10);
        }
    }
    rn->setup_traffic_lights();

    Simulation sim(rn);
    sim.run(60); // Run for 60 seconds

    return 0;
}