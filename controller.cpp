#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

using namespace std;

struct StateDerivative
{
	double cart_position_rate;
	double cart_velocity_rate;
	double angle_rate;
	double angular_velocity_rate;
};

struct State
{
	double cart_position;
	double cart_velocity;
	double angle;
	double angular_velocity;
};

string filename{"data.csv"};

double pole_length = 1;
double pole_mass = 1;
double cart_mass = 10;
double gravity = 9.81;

int setpoint{0};
double Kp = 150.0;
double Kd = 40.00;
double previous_error{0};
double dt = 0.01;
double Kx = 0.1;
double Kv = 1.0;

StateDerivative dynamics(double force, State state)
{
	double sin_angle = sin(state.angle);
	double cos_angle = cos(state.angle);

	double effective_inertia = cart_mass + pole_mass * pow(sin_angle, 2);

	double cart_acceleration = (force + pole_mass * sin_angle * (pole_length * pow(state.angular_velocity, 2) + gravity * cos_angle)) / effective_inertia;
	double angle_acceleration = (-force * cos_angle - pole_mass * pole_length * pow(state.angular_velocity, 2) * cos_angle * sin_angle + (cart_mass + pole_mass) * gravity * sin_angle) / (pole_length * effective_inertia);

	return StateDerivative{
		.cart_position_rate = state.cart_velocity,
		.cart_velocity_rate = cart_acceleration,
		.angle_rate = state.angular_velocity,
		.angular_velocity_rate = angle_acceleration};
}

State add_scaled_state(State state, StateDerivative derivative, double scale)
{
	State s{
		.cart_position = state.cart_position + derivative.cart_position_rate * scale,
		.cart_velocity = state.cart_velocity + derivative.cart_velocity_rate * scale,
		.angle = state.angle + derivative.angle_rate * scale,
		.angular_velocity = state.angular_velocity + derivative.angular_velocity_rate * scale,
	};
	return s;
}

State euler(double force, State state)
{
	StateDerivative derivatives = dynamics(force, state);
	return State{
		.cart_position = derivatives.cart_position_rate * dt,
		.cart_velocity = derivatives.cart_velocity_rate * dt,
		.angle = derivatives.angle_rate * dt,
		.angular_velocity = derivatives.angular_velocity_rate * dt,
	};
}

State rk4(double force, State state)
{
	StateDerivative k1 = dynamics(force, state);

	State state2 = add_scaled_state(state, k1, dt / 2);
	StateDerivative k2 = dynamics(force, state2);

	State state3 = add_scaled_state(state, k2, dt / 2);
	StateDerivative k3 = dynamics(force, state3);

	State state4 = add_scaled_state(state, k3, dt);
	StateDerivative k4 = dynamics(force, state4);

	int total_amount_of_weights = 6;

	double new_cart_position = (state.cart_position + dt / total_amount_of_weights * (k1.cart_position_rate + 2 * k2.cart_position_rate + 2 * k3.cart_position_rate + k4.cart_position_rate));
	double new_cart_velocity = state.cart_velocity + dt / total_amount_of_weights * (k1.cart_velocity_rate + 2 * k2.cart_velocity_rate + 2 * k3.cart_velocity_rate + k4.cart_velocity_rate);
	double new_angle = state.angle + dt / total_amount_of_weights * (k1.angle_rate + 2 * k2.angle_rate + 2 * k3.angle_rate + k4.angle_rate);
	double new_angular_velocity = state.angular_velocity + dt / total_amount_of_weights * (k1.angular_velocity_rate + 2 * k2.angular_velocity_rate + 2 * k3.angular_velocity_rate + k4.angular_velocity_rate);
	return State{
		.cart_position = new_cart_position,
		.cart_velocity = new_cart_velocity,
		.angle = new_angle,
		.angular_velocity = new_angular_velocity};
}

double PD_Controller(State input)
{

	double error = setpoint - input.angle;

	double proportional = -Kp * error;
	double derivative = -Kd * (error - previous_error) / dt;
	double cart_position = -Kx * input.cart_position;
	double cart_velocity = -Kv * input.cart_velocity;

	double force = proportional + derivative - cart_position - cart_velocity;

	previous_error = error;
	return force;
}

int main(int argc, char *argv[])
{
	int run_seconds = 60;
	if (argc == 1)
	{
		cout << "No argument given, default running 60seconds" << endl;
	}
	else
	{
		run_seconds = stoi(argv[1]);
		cout << "Running sim for " + to_string(run_seconds) + " seconds" << endl;
	}
	int sim_steps = run_seconds / dt;

	ofstream ofs;
	ofs.open(filename, ofstream::app);
	ofs << "time,cart_pos,pole_angle\n";

	State state{
		.cart_position = 0,
		.cart_velocity = 0,
		.angle = 0.4,
		.angular_velocity = 0,
	};

	double force = 0;
	double time = 0;

	for (int i = 0; i <= sim_steps; i++)
	{
		state = rk4(force, state);
		force = PD_Controller(state);
		time += dt;
		string csv_string = to_string(time) + "," + to_string(state.cart_position) + "," + to_string(state.angle) + "\n";
		ofs << csv_string;
	}
	ofs.close();
	return 0;
}