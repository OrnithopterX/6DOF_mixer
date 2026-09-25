import numpy as np
import math

# 6-DOF Thruster Kinematics Test
# Coordinate convention: +X forward, +Y left, +Z up
# Wrench: [Tx, Ty, Tz, Rx, Ry, Rz]
### Currently a work in progress, but there is a lot of progress!! ### 


# <<<<<<<<<< Aircraft data input >>>>>>>>>> #

positions = np.array([
    [ 0.0,  0.0, -1.0],
    [ 0.0,  1.0,  0.0],
    [ 1.0,  0.0,  0.0],
    [ 0.0, -1.0,  0.0],
    [-1.0,  0.0,  0.0],
    [ 0.0,  0.0,  1.0],
], dtype=float)

directions = np.array([
    [-1.0,  1.0,  0.0],
    [ 1.0,  0.0,  1.0],
    [ 0.0, -1.0, -1.0],
    [-1.0,  0.0,  1.0],
    [ 0.0,  1.0, -1.0],
    [ 1.0,  1.0,  0.0],
], dtype=float)

# +1 clockwise, 0 none, -1 counterclockwise
counter_torque = np.array([-0.5, 0.5, 0.5, 0.5, 0.5, -0.5], dtype=float)

# [Tx, Ty, Tz, Rx, Ry, Rz]
desired = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)

### <<<<<<<<<< Mixer >>>>>>>>>>>>> ### 
# Maps desired 6-DOF command -> actuator outputs
# Matrix size is actuators x 6-DOF

### mixing rate constant for 45 degrees, halved again due to two motors in plane
rate = math.sqrt(2) / 4 

mixer = np.array([
    [-rate,  rate,  0.00, -rate,  rate,  0.00],
    [ rate,  0.00,  rate,  rate,  0.00,  rate],
    [ 0.00, -rate, -rate,  0.00, -rate, -rate],
    [-rate,  0.00,  rate, -rate,  0.00,  rate],
    [ 0.00,  rate, -rate,  0.00,  rate, -rate],
    [ rate,  rate,  0.00,  rate,  rate,  0.00],
], dtype=float)


def normalize_directions(directions):
    directions = np.asarray(directions, dtype=float)
    magnitudes = np.linalg.norm(directions, axis=1, keepdims=True)
    if np.any(magnitudes == 0):
        raise ValueError("Every thruster direction must be non-zero.")
    return directions / magnitudes


def build_effectiveness_matrix(positions, directions, counter_torque):
    """Build the 6 x N actuator-to-wrench matrix."""
    positions = np.asarray(positions, dtype=float)
    directions = normalize_directions(directions)
    counter_torque = np.asarray(counter_torque, dtype=float)

    n = len(positions)
    if directions.shape != (n, 3):
        raise ValueError("positions and directions must have the same number of actuators.")
    if counter_torque.shape != (n,):
        raise ValueError("counter_torque must contain one value per actuator.")

    effectiveness = np.zeros((6, n))

    for i in range(n):
        force = directions[i]
        moment = np.cross(positions[i], force)
        reaction = counter_torque[i] * force
        effectiveness[:, i] = np.concatenate([force, moment + reaction])

    return effectiveness


def calculate_actuators(mixer, desired):
    return mixer @ desired


def calculate_wrench(effectiveness, actuators):
    return effectiveness @ actuators


def print_vector(name, values):
    labels = ["Tx", "Ty", "Tz", "Rx", "Ry", "Rz"]
    print(name)
    for label, value in zip(labels, values):
        print(f"  {label:>2}: {value:+.6f}")
    print()


if __name__ == "__main__":
    effectiveness = build_effectiveness_matrix(
        positions, directions, counter_torque
    )

    actuators = calculate_actuators(mixer, desired)
    actual = calculate_wrench(effectiveness, actuators)
    error = actual - desired

    print("=" * 60)
    print("             6-DOF KINEMATICS TEST")
    print("=" * 60)
    print()
    print_vector("DESIRED INPUT", desired)

    print("ACTUATOR OUTPUTS")
    for i, value in enumerate(actuators):
        print(f"  A{i + 1}: {value:+.6f}")
    print()

    print_vector("ACTUAL FORCE / TORQUE", actual)
    print_vector("ERROR (ACTUAL - DESIRED)", error)

    print("EFFECTIVENESS MATRIX")
    print(effectiveness)
