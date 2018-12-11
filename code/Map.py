import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import shapely

from ast import literal_eval as make_tuple

class Map_2D():
    obstacles = []
    obstacle_patches=[]
    particles=[]
    
    #Takes path to map file as parameter
    def __init__(self, world):
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
            point=shapely.Point(point1[0],point1[1])
            for obs in self.obstacles:
                if point.within(obs) and not point.touches(obs):
                    return True
        else:
            path =shapely.LineString([point1,point2])
            if self.buff>0:
                path=path.buffer(self.buff)
            for obs in self.obstacles:
                if obs.intersects(path) and not path.touches(obs):
                    return True
        return False
    
    def plot(self):
        fig, ax = plt.subplots()
        
        #Set boundaries
        ax.set_xlim([self.min_x,self.max_x])
        ax.set_ylim([self.min_y,self.max_y])
        
        #Add obstacles
        for patch in self.obstacle_patches:
            ax.add_patch(patch)
            
        #Add particles
        for pt in self.particles:
            plt.plot(pt[0],pt[1],marker='o', markersize=5, color=(0,0,1))
        
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