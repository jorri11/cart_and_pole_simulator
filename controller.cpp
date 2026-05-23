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


Output PD_Controller(Input input) {
	Output output {force: 0};

	double error = setpoint - input.angle;
	
	double proportional = -Kp * error;
	double derivative = -Kd * (error - previous_error) / dt;
	double cart_position = -Kx * input.cart_position;
	double cart_velocity = - Kv * input.cart_velocity;

	output.force = proportional + derivative - cart_position - cart_velocity;
	
	previous_error = error;
	return output;
}

int main() {
	std::string input;
	while (std::getline(std::cin, input)) {

	Input state = json::parse(input).get<Input>();
	
	json json_output = PD_Controller(state);	

	std::cout << json_output.dump() << std::endl;
	}
	
	return 0;
}