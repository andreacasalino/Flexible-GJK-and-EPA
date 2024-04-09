from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import numpy as np
import os, json, random

class Paths:
    LOG_FOLDER = os.path.dirname(__file__)
    LOG_FOLDER = os.path.join(LOG_FOLDER, 'logs')
    SHAPES_AND_RESULTS = os.path.join(LOG_FOLDER, 'results')
    GJK_EPA_RESULTS = os.path.join(LOG_FOLDER, 'gjk-epa')

def read_file(filename):
    with open(filename) as fd:
        return json.load(fd)

class AxisLimits:
    def __init__(self):
        self.min = 0
        self.max = 0
    def update(self, val):
        self.min = min(self.min, val)
        self.max = max(self.max, val)
    def finalize(self):
        delta = self.max - self.min
        self.min -= delta * 0.25
        self.max += delta * 0.25

class XYZAxisLimits:
    def __init__(self) -> None:
        self.axis = [AxisLimits() for _ in range(0,3)]
    def update(self, point):
        for index in range(0, 3):
            self.axis[index].update(point[index])
    def scale(self, ax):
        val_min = 0
        val_max = 0
        for el in self.axis:
            el.finalize()
            val_min = min(el.min, val_min)
            val_max = max(el.max, val_max)
        ax.set_xlim(val_min, val_max)
        ax.set_ylim(val_min, val_max)
        ax.set_zlim(val_min, val_max)

class Figure:
    def __init__(self, title):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1, projection='3d')
        self.fig.suptitle(title)
        self.axis = XYZAxisLimits()
    def scale(self):
        self.axis.scale(self.ax)

class Colors:
    def __init__(self):
        self.colors = ['red', 'blue', '#DFFF00', '#FFBF00', '#40E0D0']
        self.index = 0
    def get(self):
        res = self.colors[self.index] if self.index < len(self.colors) else (random.random(), random.random(),random.random())
        self.index += 1
        return res

def plot_facet(ax, A, B, C, col, alp):
    x = [A[0], B[0], C[0]]
    y = [A[1], B[1], C[1]]
    z = [A[2], B[2], C[2]]
    collection = Poly3DCollection([list(zip(x,y,z))], edgecolor='black', facecolors=col, alpha=alp, linewidth=0.2)
    ax.add_collection3d(collection)

class ShapesAndResults(Figure):
    def __init__(self, filename, title):
        Figure.__init__(self, title)
        self.shapes_colors = Colors()
        tags = {
            'shape': lambda data : self.printCloud_(data),
            'coordinates': lambda data : self.printCoordinates_(data)
        }
        for obj in read_file(filename):
            tags[obj['tag']](obj['obj'])
        self.scale()
    def printCloud_(self, data):
        for point in data:
            self.axis.update(point)
        vertices = np.array(data)
        hull = ConvexHull(vertices)
        color = self.shapes_colors.get()
        for s in hull.simplices:
            s = np.append(s, s[0])  # Here we cycle back to the first coordinate
            collection = Poly3DCollection([list(zip(vertices[s, 0],vertices[s, 1],vertices[s, 2]))], edgecolor='black', facecolors=color, alpha=0.4, linewidth=0.2)
            self.ax.add_collection3d(collection)
            # plot_facet(self.ax, vertices[s, 0], vertices[s, 1], vertices[s, 2], color, 0.4)
            self.ax.plot(vertices[s, 0], vertices[s, 1], vertices[s, 2] , '.', color=color, markersize=1)        
    def printCoordinates_(self, data):
        L_x =[data['pointA'][0], data['pointB'][0]]
        L_y =[data['pointA'][1], data['pointB'][1]]
        L_z =[data['pointA'][2], data['pointB'][2]]
        self.ax.plot( L_x ,L_y , L_z, 'k--', linewidth=0.5)
        self.ax.plot( L_x ,L_y , L_z, 'go', markersize=10)
        self.ax.text(L_x[0] ,L_y[0] , L_z[0], r"$\rho_A$", color='black', fontsize = 15)
        self.ax.text(L_x[1] ,L_y[1] , L_z[1], r"$\rho_B$", color='black', fontsize = 15)

def print_origin(ax):
    ax.plot([0], [0], [0], 'r*', markersize=10)

def print_direction(ax, base, direction):
    ax.quiver(base[0], base[1], base[2], direction[0], direction[1], direction[2], color='k') 

class GjkIteration(Figure):
    def __init__(self, filename, title):
        Figure.__init__(self, title)
        print_origin(self.ax)
        self.data = read_file(filename)
        self.printClosestPoint_()
        cases = {
            2 : lambda : self.printSegment_(),
            3 : lambda : self.printFacet_(),
            4 : lambda : self.printTethraedron_()
        }
        for index, name in zip(range(0, len(self.data["vertices"])) , ['A', 'B', 'C', 'D']) :
            point = self.data["vertices"][index]["vertex_in_Minkowski_diff"]
            self.ax.plot( [point[0]], [point[1]], [point[2]], 'ro', markersize=10)
            self.ax.text( point[0], point[1], point[2], name, color='black', fontsize = 15)
            self.axis.update(point)
        cases[len(self.data["vertices"])]()
        print_direction(self.ax, self.data["vertices"][1]["vertex_in_Minkowski_diff"], self.data["direction"])
        self.scale()
    def printClosestPoint_(self):
        point = self.data["info"]["point"]
        self.ax.plot( [0, point[0]] , [0, point[1]] , [0, point[2]], 'r--', linewidth=0.5)
        self.ax.plot( [point[0]] , [point[1]] , [point[2]], 'o', markersize=2)        
    def printSegment_(self):
        A = self.data["vertices"][0]["vertex_in_Minkowski_diff"]
        B = self.data["vertices"][1]["vertex_in_Minkowski_diff"]
        self.ax.plot( [A[0], B[0]] , [A[1], B[1]] , [A[2], B[2]], 'r-', linewidth=1.0)
    def printFacet__(self, a, b, c):
        plot_facet(self.ax, self.data["vertices"][a]["vertex_in_Minkowski_diff"], self.data["vertices"][b]["vertex_in_Minkowski_diff"], self.data["vertices"][c]["vertex_in_Minkowski_diff"], 'red', 0.5)
    def printFacet_(self):
        self.printFacet__(0,1,2)
    def printTethraedron_(self):
        self.printFacet__(0,1,2)
        self.printFacet__(0,1,3)
        self.printFacet__(0,2,3)

class EpaIteration(Figure):
    def __init__(self, filename, title):
        Figure.__init__(self, title)
        print_origin(self.ax)
        facets = read_file(filename)
        for facet in facets:
            for point_name in ['vertexA', 'vertexB', 'vertexC']:
                self.axis.update(facet[point_name])
            plot_facet(self.ax, facet["vertexA"], facet["vertexB"], facet["vertexC"], 'green', 0.7)
        plot_facet(self.ax, facets[0]["vertexA"], facets[0]["vertexB"], facets[0]["vertexC"], 'red', 1.0) 
        self.scale()

class GjkEpaIterations:
    def __init__(self, folder):
        self.root = folder
        self.name = os.path.basename(folder)
        components = {
            'results': lambda filename : self.printResult_(filename),
            'InitialGjk': lambda filename : self.printInitialLoop_(filename),
            'FinalGjk': lambda filename : self.printFinishingLoop_(filename),
            'Epa': lambda filename : self.printEpa_(filename)
        }
        for name, pred in components.items():
            filename = os.path.join(self.root, name)
            if not os.path.exists(filename):
                continue
            pred(filename)
    def printResult_(self, filename):
        self.result = ShapesAndResults(filename, '{}-results'.format(self.name))
    def forEachStep_(self, folder):
        for filename in os.listdir(folder):
            yield os.path.join(folder, filename), filename
    def printInitialLoop_(self, folder):
        self.initialGjk = [GjkIteration(filename, '{}-InitialGjk-{}'.format(self.name, stepName) ) for filename, stepName in self.forEachStep_(folder)]
    def printFinishingLoop_(self, folder):
        self.initialGjk = [GjkIteration(filename, '{}-FinalGjk-{}'.format(self.name, stepName) ) for filename, stepName in self.forEachStep_(folder)]
    def printEpa_(self, folder):
        self.initialGjk = [EpaIteration(filename, '{}-Epa-{}'.format(self.name, stepName) ) for filename, stepName in self.forEachStep_(folder)]

def for_each_figure(parentDir, ObjectT):
    return [ObjectT(os.path.join(parentDir, filename)) for filename in os.listdir(parentDir)]

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--specificResult', default=None, help='print a specific shapes and results scene')
    parser.add_argument('--specificGjkEpa', default=None, help='print a specific gjk epa sequence of iterations')
    args = parser.parse_args()

    handlers = []
    if not args.specificResult == None:
        title = args.specificResult
        args.specificResult = os.path.join(Paths.SHAPES_AND_RESULTS, args.specificResult)
        handlers.append(ShapesAndResults(args.specificResult, title))
    elif not args.specificGjkEpa == None:
        args.specificGjkEpa = os.path.join(Paths.GJK_EPA_RESULTS, args.specificGjkEpa)
        handlers.append(GjkEpaIterations(args.specificGjkEpa))
    else:
        for hndl in for_each_figure(Paths.SHAPES_AND_RESULTS, ShapesAndResults):
            handlers.append(hndl)
        for hndl in for_each_figure(Paths.GJK_EPA_RESULTS, GjkEpaIterations):
            handlers.append(hndl)
    plt.show()
