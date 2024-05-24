import copy

import numpy as np


def angle(v):
    return np.arctan2(v[1], v[0])


def rotate(v, angle):
    cos = np.cos(angle)
    sin = np.sin(angle)
    rotationMatrix = np.array([[cos, -sin],
                               [sin, cos]])
    return np.matmul(rotationMatrix, v)


class Missile:
    def __init__(self):
        self.stepsCount = None
        self.launchPoint = None
        self.startVelocity = None
        self.hasHit = False
        self.controller = None
        self.currentDistances = []

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
        nextSightLine = aircraftPoints[:, 1] - self._points[:, -1]

        self._currentDistance = np.linalg.norm(self.sightLine)
        self.currentDistances.append(round(self._currentDistance, 2))


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
