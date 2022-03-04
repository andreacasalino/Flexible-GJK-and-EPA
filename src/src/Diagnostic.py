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

def plot_origin(ax):
    ax.plot([0], [0], [0], 'o', color='black')

def plot_facet(vertexA, vertexB, vertexC, ax, col, alp):
    x = [vertexA[0], vertexB[0], vertexC[0]]
    y = [vertexA[1], vertexB[1], vertexC[1]]
    z = [vertexA[2], vertexB[2], vertexC[2]]
    ax.add_collection3d(Poly3DCollection([list(zip(x,y,z))], edgecolor='black', facecolors=col, alpha=alp, linewidth=0.2))
    
def plot_closest_point(closest, ax):
    L_x =[closest[0], 0]
    L_y =[closest[1], 0]
    L_z =[closest[2], 0]
    ax.plot( L_x ,L_y , L_z, 'k--', linewidth=0.5)

def middle_point(A, B, C):
    return [(1.0 / 3.0)*(A[0] + B[0] + C[0]),
            (1.0 / 3.0)*(A[1] + B[1] + C[1]),
            (1.0 / 3.0)*(A[2] + B[2] + C[2])]

def plot_vector_on_facet(A, B, C, vec, ax):
    middle = middle_point(A, B, C)
    ax.quiver(middle[0], middle[1], middle[2], vec[0], vec[1], vec[2])

def plot_gjk_iteration(init_or_end, data, counter):
    fig = plt.figure(figsize=plt.figaspect(0.5)*1.5)
    if init_or_end:
        fig.suptitle("GJK initial " + str(counter), fontsize=14)
    else:
        fig.suptitle("GJK ending " + str(counter), fontsize=14)
    ax = fig.add_subplot(projection='3d')
    limits = PlotLimitsAware()

    plot_origin(ax)

    closest_point = data.get("closest")
    if(None != closest_point):
        closest_point = closest_point.get("point")
    if(None != closest_point):
        plot_closest_point(data["closest"]["point"], ax)
    
    direction = data["direction"]
    A = data["plex"][0]
    limits.update(A)
    ax.text(A[0] ,A[1] , A[2], "A", color='black')
    B = data["plex"][1]
    limits.update(B)
    ax.text(B[0] ,B[1] , B[2], "B", color='black')
    if len(data["plex"]) == 2:
        ax.quiver(B[0], B[1], B[2], direction[0], direction[1], direction[2])
        ax.plot( [A[0], B[0]] ,[A[1], B[1]] , [A[2], B[2]], 'r-', linewidth=0.5)
    if len(data["plex"]) == 3:
        C = data["plex"][2]
        limits.update(C)
        ax.quiver(0.5 * (B[0] + C[0]), 0.5 * (B[1] + C[1]), 0.5 * (B[2] + C[2]), direction[0], direction[1], direction[2])
        ax.text(C[0] ,C[1] , C[2], "C", color='black')
        plot_facet(A, B, C, ax, [1,0,0], 0.4)
    if len(data["plex"]) == 4:
        C = data["plex"][2]
        limits.update(C)
        D = data["plex"][3]
        limits.update(D)
        ax.text(C[0] ,C[1] , C[2], "C", color='black')
        ax.text(D[0] ,D[1] , D[2], "D", color='black')
        
        plot_facet(A, B, C, ax, [1,0,0], 0.4)
        plot_vector_on_facet(A,B,C, data["facets"]["normals"][0], ax)

        plot_facet(A, B, D, ax, [1,0,0], 0.4)
        plot_vector_on_facet(A,B,D, data["facets"]["normals"][1], ax)

        plot_facet(A, C, D, ax, [1,0,0], 0.4)
        plot_vector_on_facet(A,C,D, data["facets"]["normals"][2], ax)

        plot_facet(B, C, D, ax, [1,0,0], 0.4)
        plot_vector_on_facet(B,C,D, direction, ax)

    limits.resize(ax)
    plt.show()

def plot_epa_iteration(data, counter):
    fig = plt.figure(figsize=plt.figaspect(0.5)*1.5)
    fig.suptitle("EPA " + str(counter), fontsize=14)
    ax = fig.add_subplot(projection='3d')
    limits = PlotLimitsAware()
    plot_origin(ax)
    for facet in data["facets"]:
        plot_facet(facet["A"],facet["B"], facet["C"], ax, [0,1,0], 0.4)
        limits.update(facet["A"])
        limits.update(facet["B"])
        limits.update(facet["C"])
    plot_facet(data["closest"]["A"],data["closest"]["B"], data["closest"]["C"], ax,  [1,0,0], 0.4)

    limits.resize(ax)
    plt.show()
    

log_name = sys.argv[1]
data_json = get_json_from_file(log_name)

gjk_initial_json = data_json.get("GJK_initial")
if(gjk_initial_json != None):
    k = 0
    for gjk_iter in gjk_initial_json:
        plot_gjk_iteration(True, gjk_iter, k)
        k = k +1

gjk_ending_json = data_json.get("GJK_ending")
if(gjk_ending_json != None):
    k = 0
    for gjk_iter in gjk_ending_json:
        plot_gjk_iteration(False, gjk_iter, k)
        k = k +1

epa_json = data_json.get("EPA")
if(epa_json != None):
    k = 0
    for epa_iter in epa_json:
        plot_epa_iteration(epa_iter, k)
        k = k +1
