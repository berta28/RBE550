
import copy
import numpy as np
from typing import Dict, Tuple, List

from a_star import hybrid_a_star
from animation import ploter
from costmap import Vehicle, Map
import collision_check
from rs_curve import PATH


class PathPlanner:
    def __init__(self,
                 config: dict = None,
                 map: Map = None,
                 vehicle: Vehicle = None) -> None:
        self.config = config
        self.map = map
        self.vehicle = vehicle
        self.collision_checker = collision_check.distance_checker(map=map,
                                                                      vehicle=vehicle,
                                                                      config=config)

        self.planner = hybrid_a_star(
            config=config, park_map=map, vehicle=vehicle)

    def path_planning(self) -> Tuple[List[List], Dict, List[List[List]]]:
        final_path = self.a_star_plan()
        return final_path

    def a_star_plan(self) -> Tuple[List[List], List[List], PATH]:
        '''
        use a star to search a feasible path and use rs curve to reach the goal,
        final_path = astar_path + rs_path
        return: final_path
        '''
        astar = self.planner

        reach_goal = False

        while not astar.open_list.empty() and not reach_goal:
            # get current node
            current_node = astar.open_list.get()
            # show info
            print('---------------')
            print('current node index:', current_node.index)
            print('distance:', np.sqrt((current_node.x-self.map.case.xf)
                                       ** 2 + (current_node.y - self.map.case.yf)**2))
            print('---------------')

            rs_path, collision, info = astar.try_reach_goal(current_node)
            # plot the collision position
            if collision and self.config['draw_collision']:
                collision_p = info['collision_position']
                ploter.plot_collision_p(
                    collision_p[0], collision_p[1], collision_p[2], self.map)

            if not collision and info['in_radius']:
                reach_goal = True
                break

            else:
                # expand node
                if self.vehicle.vec_type == 'car':
                    child_group = astar.expand_node_car(current_node)
                elif self.vehicle.vec_type == 'truck':
                    child_group = astar.expand_node_truck(current_node)
                elif self.vehicle.vec_type == 'diwheel':
                    child_group = astar.expand_node_diwheel(current_node)
                else:
                    raise Exception('Vehicle must be : car, truck, or diwheel')
                path = []
                for i in child_group.queue:
                    x = i.x
                    y = i.y
                    theta = i.theta
                    theta_1 = i.theta_1
                    path.append([x, y, theta, theta_1])

        a_star_path = astar.finish_path(current_node)
        final_path = copy.deepcopy(a_star_path)
        # final_path = a_star_path + rs_path
        # assemble all path
        for i in range(1, len(rs_path.x)):
            x = rs_path.x[i]
            y = rs_path.y[i]
            theta = rs_path.yaw[i]
            theta_1 = theta+rs_path.theta_1[i]
            # final_path.append([x, y, theta])
            final_path.append([x, y, theta, theta_1])

        return final_path