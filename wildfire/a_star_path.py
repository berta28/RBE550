from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import numpy as np

# Define a grid with a larger map

class Astar():
    def __init__(self,map) -> None:
        self.map = map
        self.grid = Grid()
        self.convert_map_to_proper_format()
        # Create an A* finder with custom cost and diagonal movement
        self.finder = AStarFinder(diagonal_movement=1)
        
        # Find the path from (0, 0) to (5, 8)

    def convert_map_to_proper_format(self):

        matrix = [[1 if val == 0 else 255 for val in row] for row in self.map.cost_map]
        self.grid = Grid(matrix=matrix)

    def plan(self, start, end):
        atol = self.map.discrete_size/2
        start_point_x_index = np.where(np.isclose(self.map.map_position[0], start[0],atol=atol))

        start_point_y_index = np.where(np.isclose(self.map.map_position[1], start[1],atol=atol))

        end_point_x_index = np.where(np.isclose(self.map.map_position[0], end[0]- self.map.discrete_size, atol=atol))

        end_point_y_index = np.where(np.isclose(self.map.map_position[1], end[1] - self.map.discrete_size,atol=atol))

        # node(y,x)
        start = self.grid.node(int(start_point_y_index[0]), int(start_point_x_index[0]))
        end = self.grid.node(int(end_point_y_index[0]), int(end_point_x_index[0]))
        path, iterations = self.finder.find_path(start, end, self.grid)
        # path returns the index values not the actual values on the grid
        # Print the path and steps taken
        formated_path = [(node.x, node.y) for node in path]
        print('Path:', path)
        print('Steps:', iterations)
        return formated_path
