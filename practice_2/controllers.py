import numpy as np
import skfuzzy as fz

_defaultAngleLimit = np.pi / 36


class Proportional:
    def __init__(self, coeff, angleLimit=_defaultAngleLimit):
        self._coeff = coeff
        self._angleLimit = angleLimit

    def rotationAngle(self, missile):
        return np.clip(3 * missile._approachVelocity * missile._sightAngleDelta,
                       -self._angleLimit, self._angleLimit)


class Fuzzy:
    def __init__(self, inferenceMethod, defuzzMethod,
                 angleLimit=_defaultAngleLimit):

        self._angleLimit = angleLimit

        self._defineUniversalSets()
        self._defineMemberFunctions()
        self._setInferenceMethod(inferenceMethod)
        self._setDefuzzMethod(defuzzMethod)

    def _defineUniversalSets(self):
        self._approachVelocityX = np.linspace(0, 10)
        self._sightAngleDeltaX = np.linspace(0, 2 * np.pi)
        self._distanceX = np.linspace(0, 10 ** 2)
        self._rotationAngleX = np.linspace(0, self._angleLimit, 9)

    def _defineMemberFunctions(self):
        self._approachVelocityU = _makeMemberFunctions(self._approachVelocityX)
        self._sightAngleDeltaU = _makeMemberFunctions(self._sightAngleDeltaX)
        self._distanceU = _makeMemberFunctions(self._distanceX)
        self._rotationAngleU = _makeMemberFunctions(self._rotationAngleX)

    def _setInferenceMethod(self, m):
        maxMin = 'Max-Min'  # Метод максимума-минимума
        maxProd = 'Max-Prod'  # Метод максимума-произведения

        print(m)

        # l - membership levels vector.
        if m == maxMin:
            self._inferenceMethod = lambda l: np.fmin(l, self._rotationAngleU)
        elif m == maxProd:
            self._inferenceMethod = lambda l: l * self._rotationAngleU
        else:
            raise ValueError(f'Expected "{maxMin}" or "{maxProd}"')

    def _setDefuzzMethod(self, m):
        rightMax = 'Right-Max'  # Метод правого максимума
        centroid = 'Centroid'  # Метод центра тяжести

        print(m)

        # u - aggregated membership function values.
        if m == rightMax:
            self._defuzzMethod = lambda u: fz.defuzz(self._rotationAngleX, u, 'lom')
        elif m == centroid:
            self._defuzzMethod = lambda u: fz.defuzz(self._rotationAngleX, u, 'centroid')
        else:
            raise ValueError(f'Expected "{rightMax}" or "{centroid}"')

    def rotationAngle(self, missile):
        fuzzyInputs = self._fuzzInputs(missile)
        return np.sign(missile._sightAngleDelta) * self._angleModule(fuzzyInputs)

    def _fuzzInputs(self, missile):
        approachVelocityL = _fuzz(self._approachVelocityX, self._approachVelocityU,
                                  missile._approachVelocity)
        sightAngleDeltaL = _fuzz(self._sightAngleDeltaX, self._sightAngleDeltaU,
                                 abs(missile._sightAngleDelta))
        distanceL = _fuzz(self._distanceX, self._distanceU, missile._currentDistance)
        distanceL = np.flipud(distanceL)

        return [approachVelocityL, sightAngleDeltaL, distanceL]

    def _angleModule(self, fuzzyInputs):
        u = np.amax(np.vstack(tuple(map(self._inferenceMethod, fuzzyInputs))), 0)
        return self._defuzzMethod(u)


def _makeMemberFunctions(x):
    """
    Returns matrix of member functions values for given universal set.

    Rows denote values for following terms: Low, Medium, High.
    """

    half = x[x.size // 2]

    return np.vstack((fz.zmf(x, x[0], half),
                      fz.trimf(x, [x[0], half, x[-1]]),
                      fz.smf(x, half, x[-1])))


def _fuzz(x, u, value):
    """Fuzz value.
    
    Parameters: 
    x - universals set.
    u - matrix of member functions values for given universal set.
    value - value to fuzz.

    Returns column vector of membership levels for each term.    
    """

    value = np.clip(value, x[0], x[-1])

    result = np.array(tuple(map(lambda termRow: fz.interp_membership(x, termRow, value),
                                u)))
    return result.reshape(u.shape[0], 1)
