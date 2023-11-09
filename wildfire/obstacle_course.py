import sys
import typing
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout,QVBoxLayout
from PyQt5.QtGui import QPainter, QColor, QPixmap, QFont
from PyQt5.QtCore import QTimer, Qt
# import pyqtgraph as pg
import random
import copy
import time
import json
import numpy as np
# import costmap
#Glbal vairables for the size of grid
GRID_SIZE = 50
CELL_SIZE = int(825/ GRID_SIZE)
iteration_data = []

class Cell():
    def __init__(self,coord):
            self.x = coord[0]
            self.y = coord[1]
            self.theta = coord[2]
            self.weight = 100000
            self.path=[]

class IterationData():
    def __init__(self,obstacle_coverage,BFS_iterations, DFS_iterations, Dijkstras_iterations, random_iterations) -> None:
        self.obstacle_coverage = obstacle_coverage
        self.DFS_iterations = DFS_iterations
        self.BFS_iterations = BFS_iterations
        self.Dijkstras_iterations = Dijkstras_iterations
        self.random_iterations = random_iterations
    
        

class TetrominoWidget(QWidget):
    def __init__(self,grid):
        super().__init__()
        self.grid = grid

        # self.timer = QTimer()
        # self.timer.timeout.connect(self.update)
        # self.timer.start(1000)
   
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setBrush(QColor(255, 255, 255))  # White color for free spaces
        painter.setPen(QColor(0, 0, 0))  # Black color for grid lines

        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                if self.grid[y][x] == 1:
                    painter.setBrush(QColor(255, 0, 0))  # Red color for occupied spaces
                elif self.grid[y][x] == 2:
                    painter.setBrush(QColor(255, 255, 0))
                elif self.grid[y][x] == 3:
                    painter.setBrush(QColor(0, 0, 255))
                elif self.grid[y][x] == 4:
                    painter.setBrush(QColor(0, 255, 255))
                elif self.grid[y][x] == 5:
                    painter.setBrush(QColor(120, 120, 255))
                else:
                    painter.setBrush(QColor(255, 255, 255))

                painter.drawRect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
       
    def save_image(self):
        pixmap = QPixmap(self.size())  # Create a QPixmap of the same size as the widget
        self.render(pixmap)  # Render the widget's contents onto the QPixmap
        pixmap.save("tetromino_grid.png", "PNG")  # Save the QPixmap as an image file
        # # print("Image saved as 'tetromino_grid.png'")


class GRIDWindow(QMainWindow):
    def __init__(self,grid,search_algorithum):
        super().__init__()

        self.grid = grid
        self.setWindowTitle(fr"Tetromino Obstacle Field {search_algorithum}")
        self.setGeometry(100, 100, CELL_SIZE * GRID_SIZE, CELL_SIZE * GRID_SIZE)
        self.central_widget = TetrominoWidget(self.grid)
        self.setCentralWidget(self.central_widget)
        self.start = [0,GRID_SIZE-1]
        self.end = [GRID_SIZE-1,0]
    
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Q:
            QApplication.quit()

    def Dijkstras(self):
    #priority queue
        path=[]
        reach_goal = False
        start = self.start
        end = self.end
        queue = [{"node":start,"path":[start],"weight": 0}]  # start point, empty pat
        visited = self.grid
        visited = mark_visted(start,visited)
        iterations=0
        while queue and visited[end[0]][end[1]] == 0:
            iterations+=1

            node_obj = min(queue, key=lambda obj: obj["weight"])
            queue.remove(node_obj)
            # Check nearest nieghbors
            nieghbors = get_neighbor_nodes(node_obj)
            for nieghbor in nieghbors:
                if visited[nieghbor["node"][0]][nieghbor["node"][1]] < 1:
                    queue.append(nieghbor)
                    visited = mark_visted(nieghbor["node"],visited)
                    
                if nieghbor["node"] == end:
                    reach_goal = True
                    final_path = nieghbor["path"]
                    for node in final_path:
                        visited[node[0]][node[1]] = 5
        if not reach_goal:
            print("No path was found for Dijkstras")
        #starting and ending values
        visited[start[0]][start[1]]=3
        visited[end[0]][end[1]]=4
        return iterations
    
    def get_neighbor_nodes(self,node_obj):
        # Direction vectors
        neighbor_nodes = []
        nieghbor_vector = [[-1, 0],[1, 0],[ 0, 1], [0, -1]]
        for vector in nieghbor_vector:
                nieghbor = [node_obj["node"][0] +vector[0], node_obj["node"][1]+vector[1]]
                #make sure they are within bounds
                if nieghbor[0] >=0 and nieghbor[0] <= GRID_SIZE-1 and nieghbor[1] >=0 and nieghbor[1] <= GRID_SIZE-1:
                    new_path = copy.copy(node_obj["path"])
                    new_path.append(nieghbor)
                    neighbor_nodes.append({"node":nieghbor, "path":new_path,"weight": np.norm(node_obj['node']- self.start['node']) + np.norm(node_obj['node']- self.end['node'])})
        return neighbor_nodes

    
def mark_visted(node, visited):
    visited[node[0]][node[1]] = 2
    return visited


class GraphWindow(QMainWindow):
    def __init__(self,data):
        super().__init__()
        
        self.setWindowTitle(fr"Iterations")
        # self.setGeometry(100, 100, CELL_SIZE * GRID_SIZE, CELL_SIZE * GRID_SIZE)
        self.central_widget = GraphWidget(data)
        self.setCentralWidget(self.central_widget)
        self.central_widget.save_to_json()
        # self.central_widget.load_from_json()
        self.central_widget.save_image()
    
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Q:
            QApplication.quit()
    
      

class GraphWidget(QWidget):
    def __init__(self, data):
        super().__init__()
        self.data = data

        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        self.setLayout(layout)

        # Create a PlotWidget
        self.plot_widget = pg.PlotWidget()
        layout.addWidget(self.plot_widget)

        # Add a PlotItem to the PlotWidget
        self.plot_item = self.plot_widget.getPlotItem()
        self.plot_item.setLabel('left', 'Iterations')
        self.plot_item.setLabel('bottom', 'Obstacle Coverage')
        self.plot_item.addLegend()

        # Add data to the plot
        # self.plot_data()

    def plot_data(self):
        obstacle_coverage = [obj.obstacle_coverage for obj in self.data]
        bfs_iterations = [obj.BFS_iterations for obj in self.data]
        dfs_iterations = [obj.DFS_iterations for obj in self.data]
        dijkstra_iterations = [obj.Dijkstras_iterations for obj in self.data]
        random_iterations = [obj.random_iterations for obj in self.data]

        # Create plot curves with names for the legend
        self.plot_item.plot(obstacle_coverage, bfs_iterations, pen='b', symbol='o', symbolSize=5, name='BFS')
        self.plot_item.plot(obstacle_coverage, dfs_iterations, pen='g', symbol='s', symbolSize=5, name='DFS')
        self.plot_item.plot(obstacle_coverage, dijkstra_iterations, pen='r', symbol='t', symbolSize=5, name='Dijkstra')
        self.plot_item.plot(obstacle_coverage, random_iterations, pen='y', symbol='t', symbolSize=5, name='Random')
    
    
    def save_to_json(self, filename = "iteration_data.json"):
        data_to_save = []
        for obj in self.data:
            data_to_save.append({
                'obstacle_coverage': obj.obstacle_coverage,
                'BFS_iterations': obj.BFS_iterations,
                'DFS_iterations': obj.DFS_iterations,
                'Dijkstra_iterations': obj.Dijkstras_iterations,
                'random_iterations': obj.random_iterations
            })

        with open(filename, 'w') as json_file:
            json.dump(data_to_save, json_file, indent=4)
    
    def load_from_json(self, filename="iteration_data.json"):
        try:
            with open(filename, 'r') as json_file:
                loaded_data = json.load(json_file)

            # Clear existing data
            self.data.clear()

            # Append loaded data to the data list
            for item in loaded_data:
                obj = IterationData(
                    item['obstacle_coverage'],
                    item['BFS_iterations'],
                    item['DFS_iterations'],
                    item['Dijkstra_iterations'],
                    item['random_iterations']
                )
                self.data.append(obj)

            # Update the plot
            self.plot_data()
        
        except FileNotFoundError:
            print(f"File not found: {filename}")
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")
    def save_image(self):
        pixmap = QPixmap(self.size())  # Create a QPixmap of the same size as the widget
        self.render(pixmap)  # Render the widget's contents onto the QPixmap
        pixmap.save("iterations.png", "PNG")  # Save the QPixmap as an image file
        

class Tetrominos():
    def __init__(self):
        # Define the possible Tetromino shapes as lists of coordinates
        self.tetrominos = [
            # [(0, 0), (1, 0), (2, 0), (3, 0)],  # I-shape
            # [(0, 0), (1, 0), (2, 0), (2, 1)],  # L-shape
            # [(0, 1), (1, 1), (1, 0), (2, 0)],  # S-shape
            [(0, 0), (0, 15), (10,15),(10,10), (5,10), (5,0)]   # T-shape
        ]

    def get_random_tetromino(self):
        #returns a random tetromino
        return random.choice(self.tetrominos)
    
    def get_random_orientated_tetromino(self):
        random_tetromino = self.get_random_tetromino()
        random_rotation = random.randint(0, 3)  # Randomly choose a rotation (0 to 3)

        # Rotate the Tetromino
        rotated_tetromino = []
        for coord in random_tetromino:
            new_coord = (coord[1], -coord[0])  # Rotate coordinates
            rotated_tetromino.append(new_coord)

        # Apply the chosen rotation
        for _ in range(random_rotation):
            rotated_tetromino = [(coord[1], -coord[0]) for coord in rotated_tetromino]
        return rotated_tetromino


# Function to add a Tetromino to the grid at a given position   
def add_tetromino(grid,tetromino, x, y):
    for coord in tetromino:
        grid[y + coord[1]][x + coord[0]] = 1

#Creates an obstacle field based on the input percentage of desired coverage
def create_obstacle_field(target_percentage):
    #Creates an obstacle field based on the input percentage of desired coverage
    grid = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]
    
    # Calculate the total number of cells in the grid
    total_cells = GRID_SIZE * GRID_SIZE

    # Calculate the target number of occupied cells based on the desired percentage
    target_occupied_cells = int(total_cells * (target_percentage))

    # Counter for the number of occupied cells
    occupied_cells = 0

    total_obsticles_added = 0
    failed_counter = 0
    tetrominos = Tetrominos()

    # Add Tetrominos until the target percentage is reached
    while occupied_cells < target_occupied_cells and failed_counter < 10000:
        #get a random tetromino
        rotated_tetromino = tetrominos.get_random_orientated_tetromino()

        # Find the maximum x and y values in the rotated Tetromino to avoid placing to far
        max_x = max(coord[0] for coord in rotated_tetromino)
        max_y = max(coord[1] for coord in rotated_tetromino)

        # Calculate random the position to place the Tetromino
        random_x = random.randint(0, GRID_SIZE - 1 - max_x)
        random_y = random.randint(0, GRID_SIZE - 1 - max_y)

        # Check if the Tetromino can be placed without going out of bounds or overlapping
        can_place = True
        for coord in rotated_tetromino:
            new_x = random_x + coord[0]
            new_y = random_y + coord[1]
            #insures tetrominos say intack due to negative index values of
            if new_x < 0 or new_x >= GRID_SIZE or new_y < 0 or new_y >= GRID_SIZE:
                can_place = False
                break
            if grid[new_y][new_x] == 1:
                can_place = False
                break
        
        # If the Tetromino can be placed, add it to the grid and update the occupied cells count
        if can_place:
            add_tetromino(grid,rotated_tetromino, random_x, random_y)
            total_obsticles_added += 1
            occupied_cells += len(rotated_tetromino)
            failed_counter = 0
        failed_counter += 1
  
    print(fr"Obstacles Added: {total_obsticles_added} | Occuppied cells: {occupied_cells} | Percentage of Field covered: {(occupied_cells/total_cells)*100}% target was: {target_percentage*100}%")
    return grid
#  depth first search, breadth first search, and Dijkstra’s, 

              
def mark_visted(node, visited):
    visited[node[0]][node[1]] = 2
    return visited
def get_neighbor_nodes(node_obj):
    # Direction vectors
    neighbor_nodes = []
    nieghbor_vector = [[-1, 0],[1, 0],[ 0, 1], [0, -1]]
    for vector in nieghbor_vector:
            nieghbor = [node_obj["node"][0] +vector[0], node_obj["node"][1]+vector[1]]
            #make sure they are within bounds
            if nieghbor[0] >=0 and nieghbor[0] <= GRID_SIZE-1 and nieghbor[1] >=0 and nieghbor[1] <= GRID_SIZE-1:
                new_path = copy.copy(node_obj["path"])
                new_path.append(nieghbor)
                neighbor_nodes.append({"node":nieghbor, "path":new_path,"weight": node_obj["weight"] + 1})
    return neighbor_nodes

if __name__ == "__main__":
    
    app = QApplication(sys.argv)
    # for x in range(0,10,10):
    x = 10
    desired_coverage = x/100
    grid_w_obstacles = create_obstacle_field(desired_coverage)
    grid_1= copy.copy(grid_w_obstacles)
    grid_2 = copy.deepcopy(grid_w_obstacles)
    grid_3 = copy.deepcopy(grid_w_obstacles)
    grid_4 = copy.deepcopy(grid_w_obstacles)
    while True:
        # BFS_window = GRIDWindow(grid_1,fr"BFS w/ coverage:{desired_coverage}")
        # DFS_window = GRIDWindow(grid_2,fr"DFS w/ coverage:{desired_coverage}")
        Dijkstras_window = GRIDWindow(grid_3,fr"Dijkstras w/ coverage:{desired_coverage}")
        # random_window = GRIDWindow(grid_4,fr"Random ")
        # iteration_data.append(IterationData(x,BFS_window.BFS(),DFS_window.DFS(),Dijkstras_window.Dijkstras(),50000))
        Dijkstras_window.Dijkstras()
        # iteration_data.append(IterationData(x,1,2,3,4))
        # graph_window = GraphWindow(iteration_data)

        # graph_window.show()
        if x%10 == 0:
            # BFS_window.show()
            # DFS_window.show()
            Dijkstras_window.show()
            # random_window.show()
        app.exec_()
        if True:
            break
    
    sys.exit(app.exec_())
