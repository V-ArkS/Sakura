import math
class Circle:
    dimention = 11
    def __init__(self, radius):
        self.radius = radius
    def getArea(self):
        return math.pi * math.pow(self.radius, 2)
    def getCircumference(self):
        return 2 * math.pi * self.radius

class Sphere:
    dimention = 22
    def __init__(self, radius):
        self.radius = radius
    def getSurfaceArea(self):
        return 4 * math.pi * math.pow(self.radius, 2)
    def getVolume(self):
        return 4 / 3 * math.pi * math.pow(self.radius, 3)

class Cube:
    dimention = 33
    def __init__(self, edge_length):
        self.edge_length = edge_length
    def getSurfaceArea(self):
        return 6 * math.pow(self.edge_length, 2)
    def getVolume(self):
        return math.pow(self.edge_length, 3)
    def getDiagonal(self):
        return math.pow(self.edge_length, pow(3, 0.5))
