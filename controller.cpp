#include <iostream>
#include <string>
#include "json.hpp"

// for convenience
using json = nlohmann::json;

struct Input {
	double angle;
	double angular_velocity;
	double cart_position;
	double cart_velocity;
};

struct Output {
	double force;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Input,angle,angular_velocity,cart_position,cart_velocity);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Output, force);


int setpoint{0};

double Kp = 150.0;
double Kd = 40.00;
double previous_error{0};
double dt = 0.01;
double Kx = 0.1;
double Kv = 1.0;

int main() {
	std::string input;
	while (std::getline(std::cin, input)) {

	Input state = json::parse(input).get<Input>();
	
	Output output {force: 0};

	double error = setpoint - state.angle;
	
	double proportional = -Kp * error;
	double derivative = -Kd * (error - previous_error) / dt;
	double cart_position = -Kx * state.cart_position;
	double cart_velocity = - Kv * state.cart_velocity;

	output.force = proportional + derivative - cart_position - cart_velocity;
	
	json json_output = output;	
	
	previous_error = error;

	std::cout << json_output.dump() << std::endl;
	}
	
	return 0;
}