from umath import sin, cos, pi, copysign, sqrt, atan2, degrees, radians, asin, fabs

def clamp(value, min_value, max_value) -> float:
    return max(min_value, min(value, max_value))  

def average(*values):
    return sum(values) / len(values)

def sign(x: float) -> int:
    """sign(x) -> int

    Determines the sign of a value.

    Arguments:
        x (float): The value to be checked.

    Returns:
        ``1`` if ``x`` is positive, ``-1`` if ``x`` is negative, else ``0``.
    """
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0
class vec2:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __add__(self, other):
        return vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float):
        return vec2(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float):
        return self * scalar

    def __truediv__(self, scalar: float):
        return vec2(self.x / scalar, self.y / scalar)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __repr__(self):
        return f"vec2({self.x}, {self.y})"

    def length(self):
        return sqrt(self.x**2 + self.y**2)
    
    def orientation(self):
        """Calculate the angle of the vector in radians."""
        return atan2(self.y, self.x)

    def normalized(self):
        """Normalize the vector to have a length of 1. -> unit vector, not normal vector"""
        l = self.length()
        if l == 0:
            return vec2(0, 0)
        return self / l
    
    def rotated(self, angle: float):
        """Rotate the vector by a given angle in radians.
        for some reason not concidered as vector afterwards"""
        return mat2.rotation(angle) * self

class mat2:
    def __init__(self, a: float, b:float , c:float, d:float):

        self.m = [[a, b], [c, d]]

    def rotation(angle: float):
        c = cos(angle)
        s = sin(angle)
        return mat2(c, -s, s, c)
    
    def identity():
        return mat2(1, 0, 0, 1)
    
    def __repr__(self):
        return f"mat2({self.m[0][0]}, {self.m[0][1]}, {self.m[1][0]}, {self.m[1][1]})"
    
    def __mul__(self, other):
        if isinstance(other, mat2):
            a = self.m[0][0] * other.m[0][0] + self.m[0][1] * other.m[1][0]
            b = self.m[0][0] * other.m[0][1] + self.m[0][1] * other.m[1][1]
            c = self.m[1][0] * other.m[0][0] + self.m[1][1] * other.m[1][0]
            d = self.m[1][0] * other.m[0][1] + self.m[1][1] * other.m[1][1]
            return mat2(a, b, c, d)
        elif isinstance(other, vec2):
            x = self.m[0][0] * other.x + self.m[0][1] * other.y
            y = self.m[1][0] * other.x + self.m[1][1] * other.y
            return vec2(x, y)
        elif isinstance(other, (int, float)):
            a = self.m[0][0] * other
            b = self.m[0][1] * other
            c = self.m[1][0] * other
            d = self.m[1][1] * other
            return mat2(a, b, c, d)
        else:
            print("Unsupported operand type(s) for *: 'mat2' and '{}'".format(type(other).__name__))

    def __rmul__(self, other):
        if isinstance(other, (int, float)):
            return self * other
        else:
            print("Unsupported operand type(s) for *: '{}' and 'mat2'".format(type(other).__name__))

    def det(self):
        return self.m[0][0] * self.m[1][1] - self.m[0][1] * self.m[1][0]

    def transpose(self):  return mat2(self.m[0][0], self.m[1][0], self.m[0][1], self.m[1][1])
    
    def adj(self):
        return mat2(self.m[1][1], -self.m[0][1], -self.m[1][0], self.m[0][0]) # adj(Matrix) = (Matrix of minors)^T
    
    def inverse(self):
        det = self.det() # det^(-1)*adj(Matrix) = Matrix^(-1)
        if det == 0:
            raise ValueError("Matrix is singular and cannot be inverted.")
        return 1/det * self.adj()

def rotateVec2(vec: vec2, orientaton: float):
    '''rotate vec2 from polar coordinates.'''
    return mat2.rotation(orientaton) * vec

def normalizeVec2(vec: vec2):
    """Normalize the vector to have a length of 1. -> unit vector, not normal vector"""
    l = vec.length()
    if l == 0:
        return vec2(0, 0)
    return vec / l

def minV(*values):
    minValue = values[0]
    for v in values:
        if v < minValue:
            minValue = v
    return minValue

def maxV(*values):
    maxValue = values[0]
    for v in values:
        if v > maxValue:
            maxValue = v
    return maxValue

def angleDiff(a, b):
    '''returns difference between angles a and b
    - in radians
    - [(a - b) modulo 2pi] - pi'''
    diff = a - b
    while diff > pi:
        diff -= 2 * pi
    while diff < -pi:
        diff += 2 * pi
    return diff

def raiseError(message: str, exceptionType=Exception):
    raise exceptionType(message)