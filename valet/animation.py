import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from costmap import Vehicle, Map
from PIL import Image

class ploter:

    @staticmethod
    def plot_obstacles(map:Map, fig_id=None):
        plt.ion()
        if fig_id == None:
            fig_id = 1
        plt.figure(fig_id)
        # create original map
        ## create obstacles
        for j in range(0, map.case.obs_num):
            plt.fill(map.case.obs[j][:, 0], map.case.obs[j][:, 1], facecolor = 'k', alpha = 0.5)
        
        ## create start vahicle and terminate vehicle
        temp = map.case.vehicle.create_polygon(map.case.x0, map.case.y0, map.case.theta0)
        plt.plot(temp[:, 0], temp[:, 1], linestyle='--', linewidth = 0.4, color = 'green')
        temp = map.case.vehicle.create_polygon(map.case.xf, map.case.yf, map.case.thetaf)
        plt.plot(temp[:, 0], temp[:, 1], linestyle='--', linewidth = 0.4, color = 'red')

        ## create arrow
        plt.arrow(map.case.x0, map.case.y0, np.cos(map.case.theta0), np.sin(map.case.theta0), width=0.2, color = "gold")
        plt.arrow(map.case.xf, map.case.yf, np.cos(map.case.thetaf), np.sin(map.case.thetaf), width=0.2, color = "gold")

        plt.title("A Start Path")
        plt.xlim(map.boundary[0], map.boundary[1])
        plt.ylim(map.boundary[2], map.boundary[3])
        plt.gca().set_aspect('equal', adjustable = 'box')
        plt.gca().set_axisbelow(True)
        plt.draw()

    @staticmethod
    def plot_node(nodes, current_node):
        # plt.ion()
        plt.figure(1)

        plt.plot(current_node.x,current_node.y,'o',color='r')

        # create path
        for i in range(len(nodes)):
            plt.plot(nodes[i][0], nodes[i][1], 'o', color='grey')
        
        plt.draw()

    @staticmethod
    def plot_curve(x,y,color='grey',label=None):
        plt.plot(x,y,'-',linewidth=0.8,color=color,label=label)
        plt.draw()

    @staticmethod
    def plot_final_path(path, color='green', show_car=False, label:str=None,gif_name = "./animations.gif", vehicle=Vehicle()):
        x,y=[],[]
        v = vehicle
        frames = []
        fig1 = plt.figure(1, dpi=600, figsize=(16,12))
        car_line, trailer_line = None, None
        for i in range(len(path)):
            x.append(path[i][0])
            y.append(path[i][1])
            if i == 0:
                ploter.plot_curve(x,y,color,label)
            else:
                ploter.plot_curve(x,y,color)
            if show_car:
                points = v.create_polygon(path[i][0], path[i][1], path[i][2])
                if car_line:
                    car_line.set_data(points[:, 0], points[:, 1])
                else:
                    car_line, = plt.plot(points[:, 0], points[:, 1], linestyle='-', linewidth=0.4, color=color)

            # trailer
            if v.trailer:
                points = v.create_polygon(path[i][0] - 5 * np.cos(path[i][3]), path[i][1] - 5 * np.sin(path[i][3]), path[i][3])
                if trailer_line:
                    trailer_line.set_data(points[:, 0], points[:, 1])
                else:
                    trailer_line, = plt.plot(points[:, 0], points[:, 1], linestyle='-', linewidth=0.4, color=color)
            plt.draw()
            fig1.canvas.draw()
            frame = np.frombuffer(fig1.canvas.tostring_rgb(), dtype=np.uint8)
            frame = frame.reshape(fig1.canvas.get_width_height()[::-1] + (3,))
            frames.append(Image.fromarray(frame))
            plt.pause(0.1)

        frames[0].save(gif_name, save_all=True, append_images=frames[1:], loop=0, duration=100)

    @staticmethod
    def plot_collision_p(x,y,theta,map):
        plt.clf()
        v = Vehicle()
        ploter.plot_obstacles(map)
        plt.title('Collision Position')
        temp = v.create_polygon(x, y, theta)
        plt.plot(temp[:, 0], temp[:, 1], linestyle='--', linewidth = 0.4, color = 'blue')
        # compute circle diameter
        Rd = 0.5 * np.sqrt(((v.lr+v.lw+v.lf)/2)**2 + (v.lb**2))
        # compute circle center position
        front_circle = (x+1/4*(3*v.lw+3*v.lf-v.lr)*np.cos(theta), 
                        y+1/4*(3*v.lw+3*v.lf-v.lr)*np.sin(theta))
        rear_circle = (x+1/4*(v.lw+v.lf-3*v.lr)*np.cos(theta), 
                    y+1/4*(v.lw+v.lf-3*v.lr)*np.sin(theta))
        figure, axes = plt.figure(1), plt.gca()
        c1 = plt.Circle(front_circle, Rd, fill=False)
        c2 = plt.Circle(rear_circle, Rd, fill=False)
        axes.add_artist(c1)
        axes.add_artist(c2)

        plt.draw()