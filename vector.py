# vector.py
import math

class Vector2:
    def __init__(self, x = 0, y = 0):
        self.x = x
        self.y = y
    def __str__(self):
        return "<"+str(self.x)+", "+str(self.y)+">"
    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)
    def normalize(self):
        m = self.magnitude()
        return Vector2(self.x / m, self.y / m)
    def add(self, addend):
        return Vector2(self.x + addend.x, self.y + addend.y)
    def subtract(self, sub):
        return Vector2(self.x - sub.x, self.y - sub.y)
    def multiply(self, coef):
        return Vector2(self.x * coef, self.y * coef)
    def dotMultiply(self, factor):
        return self.x * factor.x + self.y * factor.y
    def crossMultiply(vec1, vec2 = None, self = None):
        if vec2 == None:
            return (self.x * vec1.y) - (self.y * vec1.x)
        else:
            return (vec1.x * vec2.y) - (vec1.y * vec2.x)

class Vector3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)
    def normalize(self):
        m = self.magnitude()
        return Vector3(self.x / m, self.y / m, self.z / m)
    def add(self, addend):
        return Vector3(self.x + addend.x, self.y + addend.y, self.z + addend.z)
    def subtract(self, sub):
        return Vector3(self.x - sub.x, self.y - sub.y, self.z - sub.z)
    def multiply(self, coef):
        return Vector3(self.x * coef, self.y * coef, self.z * coef)
    def dotMultiply(self, factor):
        return self.x * factor.x + self.y * factor.y + self.z * factor.z
    def crossMultiply(vec1, vec2):
        return Vector3(vec1.y * vec2.z - vec1.z * vec2.y, vec1.z * vec2.x - vec1.x * vec2.z, vec1.x * vec2.y - vec1.y * vec2.x)