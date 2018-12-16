import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from shapely.geometry import Point
from shapely.geometry import LineString
from shapely.geometry.polygon import Polygon
from matplotlib import collections as mc
from matplotlib import path as pth
import shapely.geometry as shp

from ast import literal_eval as make_tuple

class Map_2D():
    obstacles = []
    obstacle_patches=[]
    obstacle_lines=[]
    
    #Takes path to map file as parameter
    def __init__(self, world):
        self.particles_list=[]
        self.path_x=[]
        self.path_y=[]
#        self.scan=[]
        
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
        
        wall1 = LineString([(self.min_x, self.min_y), (self.max_x, self.min_y)])
        wall2 = LineString([(self.max_x, self.min_y), (self.max_x, self.max_y)])
        wall3 = LineString([(self.max_x, self.max_y), (self.min_x, self.max_y)])
        wall4 = LineString([(self.min_x, self.max_y), (self.min_x, self.min_y)])

        self.obstacles.append(wall1)
        self.obstacles.append(wall2)
        self.obstacles.append(wall3)
        self.obstacles.append(wall4)
        
        self.obstacle_lines.append(wall1)
        self.obstacle_lines.append(wall2)
        self.obstacle_lines.append(wall3)
        self.obstacle_lines.append(wall4)
        
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
            prev=points[0]
            for i in range(1,len(points)):
                self.obstacle_lines.append(LineString([prev,points[i]]))
                prev=points[i]
            self.obstacle_lines.append(LineString([points[0],prev]))

    def add_obstacle(self, points):
        #Add obstacle as Shapely object 
        polygon = Polygon(points)
        self.obstacles.append(polygon)
        #Plot obstacle:
        vertices = np.array(points)
        self.obstacle_patches.append(pth.Path(vertices))
        
    def collide(self,point1,point2=None):
        if point2==None:
            point=shp.Point(point1[0],point1[1])
            for obs in self.obstacles:
                if point.within(obs) or point.touches(obs):
                    return True
            return False
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
            ax.add_patch(patches.PathPatch(patch, facecolor='green'))
        
#        lc = mc.LineCollection(self.path,linewidths = 2.5,color="black")
#        ax.add_collection(lc)
        
        #Add particles from beginning to end
        div=len(self.particles_list)
        if div==0:
            div=1
        gradient=1.0/div
        counter=0.0
        odd=0
        
        est_color=(1,1,0)
        
        avg_x_pts=[]
        avg_y_pts=[]
        for part_list in self.particles_list:
#            print counter
            if odd%2==0:
                avg_x=0
                avg_y=0
                for pt in part_list:
                    plt.plot(pt[0],pt[1],marker='o', markersize=4, color=(1-counter,0,counter))
                    avg_x+=pt[0]
                    avg_y+=pt[1]
                avg_x/=len(part_list)
                avg_y/=len(part_list)
                avg_x_pts.append(avg_x)
                avg_y_pts.append(avg_y)
#                plt.plot(avg_x, avg_y,'-o', marker='o', markersize=5,mew=2, color=est_color)
#                if odd!=0:
#                    estimated_path.append([(prev_x,prev_y),(avg_x,avg_y)])
            else:
                for pt in part_list:
#                    print pt[2]*10.0
                    plt.plot(pt[0],pt[1],marker='o', markersize=2, color=(.2,.2,.2))
#            else:
#                for pt in part_list:
##                    print pt[2]*10.0
#                    plt.plot(pt[0],pt[1],marker='o', markersize=2, color="black")

            counter= counter+gradient
            odd+=1
        
                #add robot path
        plt.plot(self.path_x[0],self.path_y[0],marker='o', markersize=8,color="black")
        plt.plot(self.path_x,self.path_y,'-o',marker='o', markersize=1,color="black")

        
        plt.plot(avg_x_pts, avg_y_pts,'-o',marker='o', markersize=5,mew=2, color=est_color)

        #Add Estimated Path
#        ac=mc.LineCollection(estimated_path,linewidths = 2.5,color=est_color)
#        ax.add_collection(ac)
        
#        ac = mc.LineCollection(self.scan,linewidths = 0.1,color="black")
#        ax.add_collection(ac)
        
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
