from geometry import Circle, Sphere, Cube

def main():
    a = Circle(2)
    b = Sphere(3)
    c = Cube(4)
    print("The area of the circle is %.2f" % (a.getArea()))
    print("The Volume of the Shpere is %.2f" % (b.getVolume()))
    print("The diagonal of the Cube is %.2f" % (c.getDiagonal()))

if __name__ == "__main__":
    main()
