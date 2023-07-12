# statespace-rrt
Generic RRT Framework for Multidimensional State Spaces with Complex / Realistic Movement Costing

## Description
SSRRT is RRT for complex state spaces, where rules based in physics or other disciplines control the availability and cost of state transitions.  The developer is expected to create classes describing the state space and the rules and costs for state transitions.  The SSRRT engine generates movement plans as expected of RRT, though its optimizations are limited since much of the RRT optimization research is focused on simple 2D or 3D state spaces.

## Usage
```
import statespace_rrt

class MyStateSpace:
    def __init__():
        self.lat = 0
        self.lon = 0
        self.alt = 0

class MyCostFunction:
    def calcCost(state_from, state_to):
        pass

ssrrt = StateSpaceRRT(MyStateSpace, MyCostFunction)

class MyMap:
    def stateIsInBounds:
        return True

origin = MyStateSpace()
origin.lat = 85
origin.lon = -80
origin.alt = 1000

destination = MyStateSpace()
destination.lat = 83
destination.lon = 100
destination.alt = 500

map = MyMap()

num_samples = 1000

path = ssrrt.find_path(origin, destination, map, num_samples)
```