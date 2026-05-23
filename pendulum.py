import math
from dataclasses import dataclass, asdict, fields
import matplotlib.pyplot as plt
import json
import subprocess
import csv

times = []
cart_positions = []
cart_velocities = []
angles_deg = []
angular_velocities = []

time = 0.0

@dataclass
class StateDerivative:
    cart_position_rate: float
    cart_velocity_rate: float
    angle_rate: float
    angular_velocity_rate: float

@dataclass
class State:
	angle: float = 2.03633
	angular_velocity: float = 0.0
	cart_position: float = 0
	cart_velocity: float = 0

@dataclass
class Params:
	pole_length: float = 1
	pole_mass: float = 1
	cart_mass: float = 10
	dt:float = 0.01
	gravity: float = 9.81

def add_scaled_state(state, derivative, scale):
    return State(
        cart_position=state.cart_position + derivative.cart_position_rate * scale,
        cart_velocity=state.cart_velocity + derivative.cart_velocity_rate * scale,
        angle=state.angle + derivative.angle_rate * scale,
        angular_velocity=state.angular_velocity + derivative.angular_velocity_rate * scale,
    )

params = Params()
state = State(angle=0.1, angular_velocity=0.0)


def dynamics(force, state, params):
	sin_angle = math.sin(state.angle)
	cos_angle = math.cos(state.angle)

	denom = params.cart_mass + params.pole_mass * sin_angle**2

	cart_acceleration = (force + params.pole_mass * sin_angle * (params.pole_length * state.angular_velocity**2 + params.gravity * cos_angle))/denom
	angle_acceleration = (-force * cos_angle - params.pole_mass*params.pole_length  * state.angular_velocity**2 * cos_angle * sin_angle + (params.cart_mass + params.pole_mass) * params.gravity * sin_angle) / (params.pole_length * denom)
	return StateDerivative(cart_position_rate=state.cart_velocity,cart_velocity_rate=cart_acceleration,angle_rate=state.angular_velocity,angular_velocity_rate=angle_acceleration)

def euler():
	derivatives = dynamics(0,state, params)
	state.cart_position += (derivatives.cart_position_rate * params.dt)
	state.cart_velocity += (derivatives.cart_velocity_rate * params.dt)
	state.angle += (derivatives.angle_rate * params.dt)
	state.angular_velocity += (derivatives.angular_velocity_rate * params.dt)

def rk4(force, state, params):
	dt = params.dt
	k1 = dynamics(force, state, params)

	state2 = add_scaled_state(state, k1, dt/2)
	k2 = dynamics(force, state2, params)

	state3 = add_scaled_state(state, k2, dt / 2)
	k3 = dynamics(force, state3, params)

	state4 = add_scaled_state(state, k3, dt)
	k4 = dynamics(force, state4, params)

	new_cart_position = (state.cart_position + dt / 6 * (k1.cart_position_rate + 2 * k2.cart_position_rate
        + 2 * k3.cart_position_rate
        + k4.cart_position_rate))
	new_cart_velocity = (
    state.cart_velocity
    + dt / 6 * (
        k1.cart_velocity_rate
        + 2 * k2.cart_velocity_rate
        + 2 * k3.cart_velocity_rate
        + k4.cart_velocity_rate
    )
)

	new_angle = (
    	state.angle
    	+ dt / 6 * (
        	k1.angle_rate
        	+ 2 * k2.angle_rate
        	+ 2 * k3.angle_rate
        	+ k4.angle_rate
    		)
	)

	new_angular_velocity = (
		state.angular_velocity
		+ dt / 6 * (
			k1.angular_velocity_rate
			+ 2 * k2.angular_velocity_rate
			+ 2 * k3.angular_velocity_rate
			+ k4.angular_velocity_rate
		)
	)

	return State(
		cart_position=new_cart_position,
		cart_velocity=new_cart_velocity,
		angle=new_angle,
		angular_velocity=new_angular_velocity,
	)


process = subprocess.Popen(
    ["./controller"],
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE,
    text=True,
)

def writeToFile(state: dataclass):
	filename="data.csv"
	try:
		with open(filename, "r") as f:
			has_header = True
	except FileNotFoundError:
		has_header = False
	
	with open(filename, "a", newline="", encoding="utf-8") as f:
		fieldnames = [field.name for field in fields(state)]
		writer = csv.DictWriter(f,fieldnames=fieldnames)

		if not has_header:
			writer.writeheader()
		
		writer.writerow(asdict(state))


force = 0

for x in range(5000):
	state = rk4(force,state,params)
	times.append(time)
	cart_positions.append(state.cart_position)
	cart_velocities.append(state.cart_velocity)
	angles_deg.append(math.degrees(state.angle))
	angular_velocities.append(math.degrees(state.angular_velocity))
	time += params.dt
	process.stdin.write(json.dumps(asdict(state)) + "\n")
	process.stdin.flush()
	response = process.stdout.readline()
	output = json.loads(response)
	force = output["force"]
	writeToFile(state)
	print(force)


plt.figure()
plt.plot(times, angles_deg)
plt.xlabel("Time [s]")
plt.ylabel("Pole angle [deg]")
plt.title("Pole angle over time")
plt.grid(True)
plt.savefig("angles")


plt.figure()
plt.plot(times, cart_positions)
plt.xlabel("Time [s]")
plt.ylabel("Cart position [m]")
plt.title("Cart position over time")
plt.grid(True)
plt.savefig("pos")


plt.figure()
plt.plot(times, angular_velocities)
plt.xlabel("Time [s]")
plt.ylabel("Angular velocity [deg/s]")
plt.title("Pole angular velocity over time")
plt.grid(True)
plt.savefig("ang_vel")


plt.figure()
plt.plot(times, cart_velocities)
plt.xlabel("Time [s]")
plt.ylabel("Cart velocity [m/s]")
plt.title("Cart velocity over time")
plt.grid(True)
plt.savefig("cart_vel")