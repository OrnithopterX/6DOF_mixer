import numpy as np

class Quaternion:
    # Initial quaternion for 3D coordinate transformation

    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w = float(w)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def normalized(self):
        magnitude = np.sqrt(
            self.w**2 +
            self.x**2 +
            self.y**2 +
            self.z**2
        )

        return Quaternion(
            self.w / magnitude,
            self.x / magnitude,
            self.y / magnitude,
            self.z / magnitude
        )

    def conjugate(self):
        return Quaternion(
            self.w,
            -self.x,
            -self.y,
            -self.z
        )

    def __mul__(self, other):

        return Quaternion(
            self.w * other.w
            - self.x * other.x
            - self.y * other.y
            - self.z * other.z,

            self.w * other.x
            + self.x * other.w
            + self.y * other.z
            - self.z * other.y,

            self.w * other.y
            - self.x * other.z
            + self.y * other.w
            + self.z * other.x,

            self.w * other.z
            + self.x * other.y
            - self.y * other.x
            + self.z * other.w
        )

    @staticmethod
    def from_axis_angle(axis, angle):

        axis = np.asarray(axis, dtype=float)
        axis = axis / np.linalg.norm(axis)

        half = angle / 2

        return Quaternion(
            np.cos(half),
            axis[0] * np.sin(half),
            axis[1] * np.sin(half),
            axis[2] * np.sin(half)
        )

    def rotate_vector(self, vector):

        vector = np.asarray(vector, dtype=float)

        v = Quaternion(
            0,
            vector[0],
            vector[1],
            vector[2]
        )

        rotated = self * v * self.conjugate()

        return np.array([
            rotated.x,
            rotated.y,
            rotated.z
        ])


class Aircraft:
    """
    Minimal omni-axis test model with no physics.

    Position and orientation are simply states that allow testing the coordinate transformations.
    """

    def __init__(self):

        self.position = np.zeros(3)

        # BODY -> GLOBAL
        self.orientation = Quaternion()

    # --------------------------------------------------------
    # Coordinate transformations
    # --------------------------------------------------------

    def body_to_global(self, vector):

        return self.orientation.rotate_vector(vector)

    def global_to_body(self, vector):

        return self.orientation.conjugate().rotate_vector(vector)

    # --------------------------------------------------------
    # Kinematic commands
    # --------------------------------------------------------

    def move_global(self, translation, dt):

        self.position += np.asarray(translation) * dt

    def rotate_global(self, rotation_rate, dt):

        rotation_rate = np.asarray(rotation_rate)

        magnitude = np.linalg.norm(rotation_rate)

        if magnitude == 0:
            return

        axis = rotation_rate / magnitude
        angle = magnitude * dt

        delta = Quaternion.from_axis_angle(
            axis,
            angle
        )

        # Global-frame rotation
        self.orientation = (
            delta * self.orientation
        ).normalized()