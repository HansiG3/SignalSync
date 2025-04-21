#include <iostream>
#include <vector>

using namespace std;

enum Light { RED = 4, YELLOW = 2, GREEN = 5 };

class TrafficLight {
	public:
	Light state;
	int timer;
	
	TrafficLight(Light state) {
		this->state = state;
		this->timer = RED;
	}

	void update() {
		timer--;
		if(timer <= 0) {
			switch(state) {
				case RED:
					timer = state = GREEN;
					break;
				case GREEN:
					timer = state = YELLOW;
					break;
				case YELLOW:
					timer = state = RED;
					break;
				default:
					cout << "ERROR CHANGING LIGHT" << endl;
					break;
			}
		}
	}
};
class Road {
	public:
	int from;
	int to;
	bool blocked;

	Road(int from, int to, bool blocked) {
		this->from = from;
		this->to = to;
		this->blocked = blocked;
	}
};