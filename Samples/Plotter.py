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



def get_json_from_file(name):
    with open(name) as json_file:
        return json.load(json_file)
    
def plot_facet(x, y, z, ax, col, alp):
    ax.add_collection3d(Poly3DCollection([list(zip(x,y,z))], edgecolor='black', facecolors=col, alpha=alp, linewidth=0.2))

def plot_CH(Vertices, color, ax):  
    Vertices_array = np.array(Vertices)
    hull = ConvexHull(Vertices_array)
    for s in hull.simplices:
        s = np.append(s, s[0])  # Here we cycle back to the first coordinate
        plot_facet(Vertices_array[s, 0], Vertices_array[s, 1], Vertices_array[s, 2], ax, color, 0.4)
        ax.plot(Vertices_array[s, 0], Vertices_array[s, 1], Vertices_array[s, 2] , '.', color=color, markersize=1)
    
def plot_data(data, color, ax, plot_lines = 1):
    for i in range(0 ,len(data['Politopes'])):
        plot_CH(data['Politopes'][i]['V'], color[i], ax)
    if plot_lines == 1:
        for l in range(0 , len(data['Lines']) ):
            L_x =[data['Lines'][l]['V'][0][0], data['Lines'][l]['V'][1][0]]
            L_y =[data['Lines'][l]['V'][0][1], data['Lines'][l]['V'][1][1]]
            L_z =[data['Lines'][l]['V'][0][2], data['Lines'][l]['V'][1][2]]
            ax.plot( L_x ,L_y , L_z, 'k--', linewidth=0.5)
            ax.plot( L_x ,L_y , L_z, 'go', markersize=10)
            ax.text(L_x[0] ,L_y[0] , L_z[0], r"$\rho_A$", color='black')
            ax.text(L_x[1] ,L_y[1] , L_z[1], r"$\rho_B$", color='black')

#First opened figure represents the politopes and the closest points (if they are not in collision) or the vertices determining
#the penetration vector: one for every pair.
#A second figure is opened only when a single pair of politopes is in the file. In this second figure, the second politope is 
#drawed with the traslation that:
    # is the penetration vector in case of a collision
    # allows the objects to become close to each other in case of collision absence
def plot_Log(file):
    data = get_json_from_file(file)

    fig = plt.figure(figsize=plt.figaspect(0.5))
    ax1 = None
    if(len(data['Politopes']) == 2):
        ax1 = fig.add_subplot(1, 2, 1, projection='3d')
        ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    else:
        ax1 = fig.add_subplot(1, 1, 1, projection='3d')
    
    color = [[1,0,0], [0,0,1]]
    if (len(data['Politopes']) > 2 ):
        for k in range(0, len(data['Politopes'])):
            color.append([random() ,random() , random()])        
    plot_data(data, color, ax1)
    
    if(len(data['Politopes']) == 2):
        ax1.set_title(r"$\mathcal{A}$ and $\mathcal{B}$")        
        
        Delta = [data['Lines'][0]['V'][0][0]- data['Lines'][0]['V'][1][0] , data['Lines'][0]['V'][0][1]- data['Lines'][0]['V'][1][1], data['Lines'][0]['V'][0][2]- data['Lines'][0]['V'][1][2]]
        for v in range(0 ,len(data['Politopes'][1]['V'])):
            data['Politopes'][1]['V'][v][0] = data['Politopes'][1]['V'][v][0] + Delta[0]
            data['Politopes'][1]['V'][v][1] = data['Politopes'][1]['V'][v][1] + Delta[1]
            data['Politopes'][1]['V'][v][2] = data['Politopes'][1]['V'][v][2] + Delta[2]
        plot_data(data, color, ax2, 0)
        ax2.set_title(r"$\mathcal{A}$ and $\mathcal{B}^{'}=\mathcal{B} + (\rho_A - \rho_B  )$")
        
    plt.show()
