import copy

import numpy as np

# Функция для вычисления угла между двумя векторами
def bearing(point1, point2):
    delta = point2 - point1
    angle = np.arctan2(delta[1], delta[0])
    bearing = np.degrees(angle)
    if bearing < 0:
        bearing += 360
    return bearing

def angle(v):
    return np.arctan2(v[1], v[0])


def rotate(v, angle):
    cos = np.cos(angle)
    sin = np.sin(angle)
    rotationMatrix = np.array([[cos, -sin],
                               [sin, cos]])
    return np.matmul(rotationMatrix, v)

def calc_bearing(longitudinal_axis, sight_line_vector):
    # Normalize both vectors
    unit_longitudinal_axis = longitudinal_axis / np.linalg.norm(longitudinal_axis)
    unit_sight_line_vector = sight_line_vector / np.linalg.norm(sight_line_vector)

    # Ensure vectors have the correct shape
    unit_longitudinal_axis = np.squeeze(unit_longitudinal_axis)
    unit_sight_line_vector = np.squeeze(unit_sight_line_vector)

    # Compute dot product
    dot_product = np.dot(unit_longitudinal_axis, unit_sight_line_vector)

    # Compute cross product (for 2D vectors)
    cross_product = unit_longitudinal_axis[0] * unit_sight_line_vector[1] - unit_longitudinal_axis[1] * unit_sight_line_vector[0]

    # Calculate the angle using arctan2
    angle = np.arctan2(cross_product, dot_product)
    bearing_angle = np.degrees(angle)

    return bearing_angle

class Missile:
    def __init__(self):
        self.stepsCount = None
        self.launchPoint = None
        self.startVelocity = None
        self.hasHit = False
        self.controller = None
        self.currentDistances = []
        self.currentBearings = []

    def copy(self):
        return copy.deepcopy(self)

    def trajectory(self, aircraftPoints):
        self._velocity = self.startVelocity
        self._points = np.hstack((self.launchPoint, self.launchPoint + self._velocity))

        return self._calcPoints(aircraftPoints)

    def _calcPoints(self, aircraftPoints):
        if np.shape(aircraftPoints)[1] < 2 or self.stepsCount < 0:
            return self._points

        self.sightLine = aircraftPoints[:, 0] - self._points[:, -2]
        # print(self.sightLine)
        nextSightLine = aircraftPoints[:, 1] - self._points[:, -1]

        self._currentDistance = np.linalg.norm(self.sightLine)
        self.currentDistances.append(round(self._currentDistance, 2))

        # Calculate the bearing and add to the list
        longitudinal_axis = self._velocity  # Current velocity vector represents the longitudinal axis
        sight_line_vector = self.sightLine

        # Calculate the bearing angle
        bearing_angle = calc_bearing(longitudinal_axis, sight_line_vector)
        self.currentBearings.append(round(bearing_angle, 2))  # Append the calculated bearing

        if self._currentDistance <= 5:
            self.hasHit = True
            return self._points

        self._sightAngleDelta = angle(nextSightLine) \
                                - angle(self.sightLine)
        self._approachVelocity = abs(np.linalg.norm(nextSightLine) - self._currentDistance)

        rotationAngle = self.controller.rotationAngle(self)
        self._velocity = rotate(self._velocity, rotationAngle)

        nextPoint = np.reshape(self._points[:, -1], (2, 1)) + self._velocity
        self._points = np.hstack((self._points, nextPoint))

        self.sightLine = nextSightLine
        return self._calcPoints(aircraftPoints[:, 1:])
