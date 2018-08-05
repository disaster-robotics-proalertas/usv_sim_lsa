from tacking import *

if __name__ == '__main__':
    current = Point()
    current.x = 0
    current.y = 0
    target = Point()
    target.x = 100
    target.y = 0
    print(tackPoints(current, target, 60, 0.3, 0, 20))
