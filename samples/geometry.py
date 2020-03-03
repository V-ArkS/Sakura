import math
class Circle:
    dimension = 2
    def __init__(self, radius):
        self.radius = radius
    def getArea(self):
        return math.pi * math.pow(self.radius, 2)
    def getCircumference(self):
        return 2 * math.pi * self.radius

class Sphere:
    dimension = 3
    def __init__(self, radius):
        self.radius = radius
    def getSurfaceArea(self):
        return 4 * math.pi * math.pow(self.radius, 2)
    def getVolume(self):
        return 4 / 3 * math.pi * math.pow(self.radius, 3)

class Cube:
    dimension = 3
    def __init__(self, edge_length):
        self.edge_length = edge_length
    def getSurfaceArea(self):
        return 6 * math.pow(self.edge_length, 2)
    def getVolume(self):
        return math.pow(self.edge_length, 3)
    def getDiagonal(self):
        return math.pow(self.edge_length, pow(3, 0.5))
