"""
Plot tools 2D
@author: huiming zhou
"""

import os
import sys
import matplotlib.pyplot as plt

import env


class Plotting:
    def __init__(self, xI, xG, obs, goal_points, start_points, img_dim):
        self.xI, self.xG = xI, xG
        self.env = env.Env()
        self.x_range = img_dim[0]
        self.y_range = img_dim[1]
        self.obs, self.goal_points, self.start_points = obs, goal_points, start_points

    def animation(self, path, visited, name):
        self.plot_grid(name)
        #self.plot_visited(visited)
        self.plot_path(path)
        plt.show()

    def plot_grid(self, name):
        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]

        goal_x = [x[0] for x in self.goal_points]
        goal_y = [x[1] for x in self.goal_points]

        start_x = [x[0] for x in self.start_points]
        start_y = [x[1] for x in self.start_points]

        ax = plt.gca()
        ax.set_xlim([0, self.x_range])
        ax.set_ylim([0, self.y_range])            

        plt.plot(self.xI[0], self.xI[1], "bs", markersize='1')
        plt.plot(self.xG[0], self.xG[1], "gs", markersize='1')
        plt.plot(obs_x, obs_y, "sk", markersize='1')
        plt.plot(goal_x, goal_y, "g", markersize='1')
        plt.plot(start_x, start_y, "b", markersize='1')
        plt.title(name)

    def plot_visited(self, visited, cl='gray'):
        if self.xI in visited:
            visited.remove(self.xI)

        if self.xG in visited:
            visited.remove(self.xG)

        count = 0

        for x in visited:
            count += 1
            plt.plot(x[0], x[1], color=cl, marker='o')
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if count < len(visited) / 3:
                length = 20
            elif count < len(visited) * 2 / 3:
                length = 30
            else:
                length = 40
            #
            # length = 15

            if count % length == 0:
                plt.pause(0.001)
        plt.pause(0.01)

    def plot_path(self, path, cl='lightskyblue', flag=False):
        path_x = [path[i][0] for i in range(len(path))]
        path_y = [path[i][1] for i in range(len(path))]

        if not flag:
            plt.plot(path_x, path_y, linewidth='3', color='r')
        else:
            plt.plot(path_x, path_y, linewidth='10', color='b')

        plt.plot(self.xI[0], self.xI[1], "bs")
        plt.plot(self.xG[0], self.xG[1], "gs")

        plt.pause(0.01)

    @staticmethod
    def color_list():
        cl_v = ['silver',
                'wheat',
                'lightskyblue',
                'royalblue',
                'slategray']
        cl_p = ['gray',
                'orange',
                'deepskyblue',
                'red',
                'm']
        return cl_v, cl_p

    @staticmethod
    def color_list_2():
        cl = ['silver',
              'steelblue',
              'dimgray',
              'cornflowerblue',
              'dodgerblue',
              'royalblue',
              'plum',
              'mediumslateblue',
              'mediumpurple',
              'blueviolet',
              ]
        return cl
