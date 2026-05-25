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

struct Params
{
	double pole_length;
	double pole_mass;
	double cart_mass;
	double gravity;
};

StateDerivative dynamics(double force, State state, Params params)
{
	double sin_angle = sin(state.angle);
	double cos_angle = cos(state.angle);

	double effective_inertia = params.cart_mass + params.pole_mass * pow(sin_angle, 2);

	double cart_acceleration = (force + params.pole_mass * sin_angle * (params.pole_length * pow(state.angular_velocity, 2) + params.gravity * cos_angle)) / effective_inertia;
	double angle_acceleration = (-force * cos_angle - params.pole_mass * params.pole_length * pow(state.angular_velocity, 2) * cos_angle * sin_angle + (params.cart_mass + params.pole_mass) * params.gravity * sin_angle) / (params.pole_length * effective_inertia);

	return StateDerivative{
		.cart_position_rate = state.cart_velocity,
		.cart_velocity_rate = cart_acceleration,
		.angle_rate = state.angular_velocity,
		.angular_velocity_rate = angle_acceleration};
}

class DifferentialEquationSolver
{
protected:
	double dt;
	Params param;

public:
	virtual State solve(double force, State current_state) = 0;
};

class Euler : public DifferentialEquationSolver
{

public:
	Euler(double timestep, Params _param)
	{
		dt = timestep;
		param = _param;
	};

	State solve(double force, State current_state) override
	{
		StateDerivative derivatives = dynamics(force, current_state, param);
		return State{
			.cart_position = derivatives.cart_position_rate * dt,
			.cart_velocity = derivatives.cart_velocity_rate * dt,
			.angle = derivatives.angle_rate * dt,
			.angular_velocity = derivatives.angular_velocity_rate * dt,
		};
	}
};
class Rk4 : public DifferentialEquationSolver
{
private:
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

public:
	Rk4(double timestep, Params _param)
	{
		dt = timestep;
		param = _param;
	}

	State solve(double force, State current_state) override
	{
		StateDerivative k1 = dynamics(force, current_state, param);

		State state2 = add_scaled_state(current_state, k1, dt / 2);
		StateDerivative k2 = dynamics(force, state2, param);

		State state3 = add_scaled_state(current_state, k2, dt / 2);
		StateDerivative k3 = dynamics(force, state3, param);

		State state4 = add_scaled_state(current_state, k3, dt);
		StateDerivative k4 = dynamics(force, state4, param);

		int total_amount_of_weights = 6;

		double new_cart_position = (current_state.cart_position + dt / total_amount_of_weights * (k1.cart_position_rate + 2 * k2.cart_position_rate + 2 * k3.cart_position_rate + k4.cart_position_rate));
		double new_cart_velocity = current_state.cart_velocity + dt / total_amount_of_weights * (k1.cart_velocity_rate + 2 * k2.cart_velocity_rate + 2 * k3.cart_velocity_rate + k4.cart_velocity_rate);
		double new_angle = current_state.angle + dt / total_amount_of_weights * (k1.angle_rate + 2 * k2.angle_rate + 2 * k3.angle_rate + k4.angle_rate);
		double new_angular_velocity = current_state.angular_velocity + dt / total_amount_of_weights * (k1.angular_velocity_rate + 2 * k2.angular_velocity_rate + 2 * k3.angular_velocity_rate + k4.angular_velocity_rate);
		return State{
			.cart_position = new_cart_position,
			.cart_velocity = new_cart_velocity,
			.angle = new_angle,
			.angular_velocity = new_angular_velocity};
	}
};

class Control_strategy
{
public:
	int setpoint{0};
	virtual double calculate_control_output(State input) = 0;
};

class Pd_controller : public Control_strategy
{
private:
	double Kx = 0.1;
	double Kv = 1.0;
	double Kp = 150.0;
	double Kd = 40.00;

public:
	double error;
	double previous_error;
	double dt;
	Pd_controller(double _dt)
	{
		dt = _dt;
	}
	Pd_controller(int _setpoint)
	{
		setpoint = _setpoint;
	}
	Pd_controller(int _setpoint, double kp, double kd, double kx, double kv, double _dt)
	{
		setpoint = _setpoint;
		Kp = kp;
		Kd = kd;
		Kx = kx;
		Kv = kv;
		dt = _dt;
	}

	double calculate_control_output(State input) override
	{
		error = setpoint - input.angle;

		// P
		double proportional = -Kp * error;
		// D
		double derivative = -Kd * (error - previous_error) / dt;
		// basicly P for position and velocity so the cart will try and recenter itself
		double cart_position = -Kx * input.cart_position;
		double cart_velocity = -Kv * input.cart_velocity;

		double force = proportional + derivative - cart_position - cart_velocity;
		previous_error = error;
		return force;
	}
};

int main(int argc, char *argv[])
{
	string filename{"data.csv"};
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

	Params p{
		.pole_length = 1,
		.pole_mass = 1,
		.cart_mass = 10,
		.gravity = 9.81,
	};

	double dt{0.01};

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

	Rk4 solver(dt, p);
	Pd_controller controller(dt);

	double force = 0;
	double time = 0;

	for (int i = 0; i <= sim_steps; i++)
	{
		state = solver.solve(force, state);
		force = controller.calculate_control_output(state);
		time += dt;

		string csv_string = to_string(time) + "," + to_string(state.cart_position) + "," + to_string(state.angle) + "\n";
		ofs << csv_string;
	}
	ofs.close();
	return 0;
}