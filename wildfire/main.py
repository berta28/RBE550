import path_planner
from animation import ploter, plt
from curve_plot import CurvePloter
import costmap
from config import read_config

import os

import argparse


def main(file, config):
    # create the park map
    
    park_map = costmap.Map(
        file=file, discrete_size=config['map_discrete_size'],vehicle=costmap.Vehicle(config['vehicle']))

    ego_vehicle = park_map.case.vehicle
    
    # create path planner
    planner = path_planner.PathPlanner(config=config,
                                       map=park_map,
                                       vehicle=ego_vehicle)

    original_path = planner.path_planning()

    # animation
    ploter.plot_obstacles(map=park_map)
    park_map.visual_cost_map()

    fig_path = os.path.join(config['pic_path'], config['vehicle'])
    if not os.path.exists(fig_path):
        os.makedirs(fig_path)
    gif_name = config['vehicle'] + '.gif'
    save_gif_name = os.path.join(fig_path, gif_name)

    ploter.plot_final_path(path=original_path, label='Hybrid A*',
                           color='green', show_car=True, gif_name=save_gif_name, vehicle=ego_vehicle)
    
    fig_name = config['vehicle'] + '.png'
    save_fig = os.path.join(fig_path, fig_name)
    plt.savefig(save_fig, dpi=600)
    plt.close()

    print('solved')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Valet')
    parser.add_argument("--config_name", type=str, default="config")
    parser.add_argument("--vehicle_name", type=str, default="car")
    args = parser.parse_args()

    # initial
    # load configure file to a dict
    config = read_config.read_config(config_name=args.config_name)

    # read case
    case_name = config['vehicle'] +'_case' + '.csv'
    file = os.path.join(config['case_path'], case_name)
    main(file=file, config=config)

