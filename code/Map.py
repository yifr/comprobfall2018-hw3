import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from matplotlib import collections as mc
import shapely.geometry as shp

from ast import literal_eval as make_tuple

class Map_2D():
    obstacles = []
    obstacle_patches=[]
    
    #Takes path to map file as parameter
    def __init__(self, world):
        self.particles_list=[]
        self.path=[]
        
        coords = []
        f = open(world)
        for line in f:
            coords.append(line.split(",\n"))
        
        walls = coords[0][0].split(' ')
        obstacles = coords[2:]
        
        #Build walls of map:
        self.min_x = make_tuple(walls[2])[0]
        self.max_x = make_tuple(walls[0])[0]
        self.min_y = make_tuple(walls[3])[1]
        self.max_y = make_tuple(walls[1])[1]
        
        #Create obstacles
        for obstacle in obstacles:
            formatted = obstacle[0].split(' ')

            #Get rid of extra spaces newline characters:
            length = len(formatted)
            i = 0
            while i < length:
                if formatted[i] == '' or formatted[i] == '\n':
                    formatted.pop(i)
                    length -= 1
                else:
                    i += 1

            points = [make_tuple(point) for point in formatted]
            self.add_obstacle(points)

    def add_obstacle(self, points):
        #Add obstacle as Shapely object 
        polygon = Polygon(points)
        self.obstacles.append(polygon)
        #Plot obstacle:
        vertices = np.array(points)
        patch = patches.Polygon(vertices)
        self.obstacle_patches.append(patch)
        
    def collide(self,point1,point2=None):
        if point2==None:
            point=shp.Point(point1[0],point1[1])
            for obs in self.obstacles:
                if point.within(obs) and not point.touches(obs):
                    return True
        else:
            path =shp.LineString([point1,point2])
            if self.buff>0:
                path=path.buffer(self.buff)
            for obs in self.obstacles:
                if obs.intersects(path) and not path.touches(obs):
                    return True
        return False
    
    def plot(self):
        fig, ax = plt.subplots()
        
        #Set plot size
        fig_size = plt.rcParams["figure.figsize"]
        fig_size[0] = 12
        fig_size[1] = 9
        plt.rcParams["figure.figsize"] = fig_size
        
        #Set boundaries
        ax.set_xlim([self.min_x,self.max_x])
        ax.set_ylim([self.min_y,self.max_y])
        
        #Add obstacles
        for patch in self.obstacle_patches:
            ax.add_patch(patch)
            
        #Add particles from beginning to end
        gradient=1.0/len(self.particles_list)
        counter=0.0
        for part_list in self.particles_list:
            print counter
            for pt in part_list:
                plt.plot(pt[0],pt[1],marker='o', markersize=1, color=(0,counter,1))
            counter= counter+gradient
        #add robot path
        plt.plot(self.path[0][0][0],self.path[0][0][1],marker='o', markersize=5,color="black")
        lc = mc.LineCollection(self.path,linewidths = 2.5,color="black")
        ax.add_collection(lc)
        
#        ax = plt.axes(xlim=(self.min_x, self.max_x), ylim=(self.min_y, self.max_y))

        plt.show()
'''
def main():
    number = raw_input("Which map do you want to visualize? \nEnter 1, 2, 3, 4, 5, 6 or 7 to continue:\n\n")
    
    f = "../turtlebot_maps/map_"+str(number)+".txt"
    map1 = Map_2D(f)
    map1.plot()

if __name__=="__main__":
    main()
'''