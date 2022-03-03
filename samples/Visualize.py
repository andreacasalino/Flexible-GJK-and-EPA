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

def plot_vertices_cloud(Vertices, color, ax):  
    Vertices_array = np.array(Vertices)
    hull = ConvexHull(Vertices_array)
    for s in hull.simplices:
        s = np.append(s, s[0])  # Here we cycle back to the first coordinate
        plot_facet(Vertices_array[s, 0], Vertices_array[s, 1], Vertices_array[s, 2], ax, color, 0.4)
        ax.plot(Vertices_array[s, 0], Vertices_array[s, 1], Vertices_array[s, 2] , '.', color=color, markersize=1)
    
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
    for politope in subplot_json_data["Politopes"]:
        plot_vertices_cloud(politope["Vertices"], politope["Color"], ax)



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
