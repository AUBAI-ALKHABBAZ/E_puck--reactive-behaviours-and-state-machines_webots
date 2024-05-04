"""epuck_controller controller."""

from typing import List, Literal
from controller import Robot, Motor
from controller.distance_sensor import DistanceSensor


FIVE_CM_DSENSOR_VALUE = 110
EPUCK_MAX_VELOCITY = 6.28

# NOTE: The number of iterations to turn a given number of degrees is highly
# dependent on the rotation speed. With a higher rotation speed,
# the number of iterations would be fewer. This is not very dynamic,
# but works for this specific task.
ROTATION_VELOCITY = 0.2 * EPUCK_MAX_VELOCITY
ITERATIONS_TO_ROTATE_180 = 112
ITERATIONS_TO_ROTATE_90 = 56


robot = Robot()


def get_motor(name: str) -> Motor:
    m = robot.getDevice(name)
    if isinstance(m, Motor):
        return m
    raise Exception(f"{name=} is not a motor")


def get_distance_sensor(name: str) -> DistanceSensor:
    s = robot.getDevice(name)
    if isinstance(s, DistanceSensor):
        return s
    raise Exception(f"{name=} is not a distance sensor")


timestep = int(robot.getBasicTimeStep())

motor_left = get_motor("left wheel motor")
motor_right = get_motor("right wheel motor")

motor_left.setPosition(float("inf"))
motor_right.setPosition(float("inf"))
motor_left.setVelocity(0)
motor_right.setVelocity(0)

d_sensors: List[DistanceSensor] = []

for i in range(8):
    d = get_distance_sensor(f"ps{i}")
    d.enable(timestep)
    d_sensors.append(d)

state: Literal["STOP", "GO_FORWARD", "ROTATE_180", "ROTATE_90"] = "GO_FORWARD"
iterations = 0
rotation_iterations = 0
walls_detected = 0

print("Everything initialized. Starting loop.")

while robot.step(timestep) != -1:
    # ==========================
    # Read sensors
    d_vals = []
    for sensor in d_sensors:
        d_vals.append(sensor.getValue())

    d_front_right = d_vals[0]
    d_front_left = d_vals[7]
    d_left = d_vals[5]

    # ==========================
    # Decide what to do
    phi_left = 0.0
    phi_right = 0.0

    if state == "ROTATE_180":
        rotation_iterations += 1
        phi_left = ROTATION_VELOCITY
        phi_right = -ROTATION_VELOCITY
        if rotation_iterations >= ITERATIONS_TO_ROTATE_180:
            state = "GO_FORWARD"
    elif state == "ROTATE_90":
        rotation_iterations += 1
        phi_left = ROTATION_VELOCITY
        phi_right = -ROTATION_VELOCITY
        if rotation_iterations >= ITERATIONS_TO_ROTATE_90:
            state = "GO_FORWARD"
    else:
        rotation_iterations = 0

    if state == "GO_FORWARD":
        phi_right = EPUCK_MAX_VELOCITY
        phi_left = EPUCK_MAX_VELOCITY
        if walls_detected == 2 and d_left < FIVE_CM_DSENSOR_VALUE:
            print("Done.")
            state = "STOP"

        if any(dv > FIVE_CM_DSENSOR_VALUE for dv in [d_front_left, d_front_right]):
            walls_detected += 1
            if walls_detected == 1:
                print("Found first wall.")
                state = "ROTATE_180"
            if walls_detected == 2:
                print("Found second wall.")
                state = "ROTATE_90"

    elif state == "STOP":
        phi_left = 0.0
        phi_right = 0.0

    # ==========================
    # Set actuators
    motor_right.setVelocity(phi_right)
    motor_left.setVelocity(phi_left)

    iterations += 1
