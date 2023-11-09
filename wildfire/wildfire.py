from costmap import Vehicle, Map
import numpy as np
import random
from animation import ploter, plt
import time
# stuff for bug
from a_star_path import Astar
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import matplotlib.image as mpimg

class Wurmpus():

    def __init__(self,map) -> None:
        self.x = 0
        self.y = 0
        self.speed= 1
        self.map = map
        self.planner = Astar(map)
        self.path = []
        self.onRoute= False
        self.wating = False
        self.reachedGoal = False
        self.figure, self.axes = plt.figure(1), plt.gca()
        bug = mpimg.imread("bug.jpg")
        self.bug_artist = AnnotationBbox(OffsetImage(bug, zoom=0.03), (self.x, self.y), frameon=False)
        self.axes.add_artist(self.bug_artist)
            

    def plan_to_next_target(self):
        closest_point_dis = np.inf
        for i, obstacle in enumerate(self.map.obstacles):
            # Need to find closest obstacle
            if not obstacle.isOnFire and obstacle.canBeLit:
                for point in obstacle.points:
                    dist = np.linalg.norm(np.array([self.x,self.y]- np.array(point)))
                    if closest_point_dis > dist:
                        closest_point_dis = dist
                        # make sure it is not in collision 
                        closest_point = point
                        closest_obs = i
        self.target_obs =  closest_obs
        self.goal = closest_point              
        self.path = self.planner.plan([self.x,self.y],closest_point.tolist())
        x_coordinates, y_coordinates = zip(*self.path)
        plt.plot(x_coordinates, y_coordinates, linestyle='-', color='grey')
        if not self.path:
            print('no path found')

    
    def plot_current_position(self):
        self.bug_artist.xy=(self.x,self.y)
        self.figure.canvas.draw()
        self.figure.show()
        
        
        # plt.scatter(self.x,self.y,s=50,color='red')

    def move(self):
        current_position = self.path.pop(0)
        # self.map.position_map[0][current_position[0]]
        # points_x_index = np.where((self.map_position[0] < original_points_position[0]) &
        #                                       (self.map_position[0] > (original_points_position[0]-self._discrete_x)))
        # points_y_index = np.where((self.map_position[1] < original_points_position[1]) &
        #                             (self.map_position[1] > (original_points_position[1]-self._discrete_y)))

        # if any(points_x_index) and any(points_y_index):
        #     self.cost_map[int(points_x_index[0])
        #                     ][int(points_y_index[0])] = 255
        self.x = self.map.map_position[0][current_position[0]]
        self.y = self.map.map_position[1][current_position[1]]
        if self.x == self.goal[0] and self.y == self.goal[1]:
            self.reachedGoal = True

if __name__ == '__main__':
    park_map = Map(discrete_size=1)
    obstacles_on_fire_indx = []
    # create a star for wurmpus
    wurmpus = Wurmpus(park_map)
    # animation
    # create truck map

    for dt in np.arange(0,3600, park_map.discrete_size):
        # wurmpus ignites instantly path to closest object clump based only needs to hit verticy not center point
        ploter.plot_obstacles(map=park_map)
        if not wurmpus.onRoute:
            wurmpus.plan_to_next_target()
            wurmpus.onRoute = True
            wurmpus.reachedGoal = False
        else:
            if wurmpus.path:
                wurmpus.move()
                wurmpus.plot_current_position()
            else:
                wurmpus.onRoute = False
                
        if wurmpus.reachedGoal:
            start_obstacle = random.choice(range(0,len(park_map.obstacles)-1))
            obstacles_on_fire_indx.append(start_obstacle)
            park_map.obstacles[start_obstacle].set_on_fire()
        
        # truck decides where to head revualted perodically during motion based on current fires 
        # possibly predicting future fires if it takes longer then 10 seconds to get there
        # truck choices path and does a single time step for it
        # when truck reaches destination
        # if vehicle.reachedGoal:
                # vehicle.time=0.0 

        # fire will spread if not put out 
        for obs_indx in obstacles_on_fire_indx:
            # check to see if truck has stoped long enough to put out fires
            #if vehicle.time % 5 = 0
            #   if np.linalg.norm(np.array(vehicle.location) - np.array(park_map.obstacles[obs_indx].centerpoint)) < 10:
                    # park_map.obstacles[obs_indx].extinguish_fire()
            if dt % 10 == 0:
        # check if any obstacles are within 30 meters are lit
                for i, obstacle in enumerate(park_map.obstacles):
                    if obstacle.canBeLit and np.linalg.norm(np.array(obstacle.centerpoint) - np.array(park_map.obstacles[obs_indx].centerpoint)) < 30:
                        park_map.obstacles[i].set_on_fire()
                        obstacles_on_fire_indx.append(i)


        # update the plot with current truck, wurmpus, fire locations
        # ploter.plot_obstacles(map=park_map)