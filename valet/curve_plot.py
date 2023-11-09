import matplotlib.pyplot as plt
import pandas as pd
import os

class CurvePloter:
    @staticmethod
    def plot_curve(data_path,
                   save_fig_path:str = None,
                   fig_id = 3):

        if not os.path.exists(save_fig_path):
            os.makedirs(save_fig_path)
        plt_item = ['v','a','sigma','omega']
        case_data = pd.read_csv(data_path, sep='\t')
        t = case_data.loc[:,'t'].to_numpy()

        for i in range(len(plt_item)):
            plt.figure(fig_id + i)
            y = case_data.loc[:, [plt_item[i]]].to_numpy()
            plt.plot(t, y, 'b-')
            plt.xlabel('time (s)')
            plt.ylabel(plt_item[i])
            plt.title(plt_item[i] + '-t')
            file_name = plt_item[i] + '-t' + '.png'
            fig_path = os.path.join(save_fig_path, file_name)
            plt.savefig(fig_path, dpi=600)
            plt.show()

        

