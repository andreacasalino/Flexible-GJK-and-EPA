# -*- coding: utf-8 -*-
"""
* Author:    Andrea Casalino
* Created:   26.12.2019
*
* report any bug to andrecasa91@gmail.com.
"""

from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
import json
from scipy.spatial import ConvexHull
import numpy as np
from random import random
import sys



def get_json_from_file(name):
    with open(name) as json_file:
        return json.load(json_file)
    
def plot_facet(x, y, z, ax, col, alp):
    ax.add_collection3d(Poly3DCollection([list(zip(x,y,z))], edgecolor='black', facecolors=col, alpha=alp, linewidth=0.2))

class AxisLimitsAware:
    def __init__(self):
        self.limits = [0,0]
    
    def update(self, new_value_in_plot):
        if(new_value_in_plot < self.limits[0]):
            self.limits[0] = new_value_in_plot
        if(new_value_in_plot > self.limits[1]):
            self.limits[1] = new_value_in_plot

    def min(self):
        return self.limits[0]
    def max(self):
        return self.limits[1]

class PlotLimitsAware:
    def __init__(self):
        self.x_limits = AxisLimitsAware()
        self.y_limits = AxisLimitsAware()
        self.z_limits = AxisLimitsAware()

    def update(self, new_point):
        self.x_limits.update(new_point[0])
        self.y_limits.update(new_point[1])
        self.z_limits.update(new_point[2])

    def update2(self, other):
        self.update([other.x_limits.min(), other.y_limits.min(), other.z_limits.min()])
        self.update([other.x_limits.max(), other.y_limits.max(), other.z_limits.max()])
    
    def resize(self, ax):
        min = self.x_limits.min()
        if(self.y_limits.min() < min):
            min = self.y_limits.min()
        if(self.z_limits.min() < min):
            min = self.z_limits.min()

        max = self.x_limits.max()
        if(self.y_limits.max() > max):
            max = self.y_limits.max()
        if(self.z_limits.max() > max):
            max = self.z_limits.max()

        incr = 0.15 * (max - min)
        min = min - incr
        max = max + incr

        ax.set_xlim3d(min, max)
        ax.set_ylim3d(min, max)
        ax.set_zlim3d(min, max)        

def plot_vertices_cloud(Vertices, color, ax) -> PlotLimitsAware:  
    limits = PlotLimitsAware()
    for point in Vertices:
        limits.update(point)
    Vertices_array = np.array(Vertices)
    hull = ConvexHull(Vertices_array)
    for s in hull.simplices:
        s = np.append(s, s[0])  # Here we cycle back to the first coordinate
        plot_facet(Vertices_array[s, 0], Vertices_array[s, 1], Vertices_array[s, 2], ax, color, 0.4)
        ax.plot(Vertices_array[s, 0], Vertices_array[s, 1], Vertices_array[s, 2] , '.', color=color, markersize=1)
    return limits
    
def plot_line(Vertices, ax):
    L_x =[Vertices[0][0], Vertices[1][0]]
    L_y =[Vertices[0][1], Vertices[1][1]]
    L_z =[Vertices[0][2], Vertices[1][2]]
    ax.plot( L_x ,L_y , L_z, 'k--', linewidth=0.5)
    ax.plot( L_x ,L_y , L_z, 'go', markersize=10)
    ax.text(L_x[0] ,L_y[0] , L_z[0], r"$\rho_A$", color='black')
    ax.text(L_x[1] ,L_y[1] , L_z[1], r"$\rho_B$", color='black')

def plot_result(subplot_json_data, ax):
    for line in subplot_json_data["Lines"]:
        plot_line(line["Peers"] ,ax)  
    limits = PlotLimitsAware()
    for politope in subplot_json_data["Politopes"]:
        new_limits = plot_vertices_cloud(politope["Vertices"], politope["Color"], ax)
        limits.update2(new_limits)
    limits.resize(ax)



log_name = sys.argv[1]
data_json = get_json_from_file(log_name)

fig = plt.figure(figsize=plt.figaspect(0.5))
fig.suptitle(log_name, fontsize=14)

k = 1
for subplot_json_data in data_json:
    ax = fig.add_subplot(1, len(data_json), k, projection='3d')
    ax.set_ylabel(subplot_json_data["title"])
    plot_result(subplot_json_data, ax)
    k = k + 1

plt.show()
