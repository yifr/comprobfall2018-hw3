import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from ast import literal_eval as make_tuple

class Map_2D():
    obstacles = []
    fig = plt.figure()
    ax = plt.axes()

    #Takes path to map file as parameter
    def __init__(self, world):
        coords = []
        f = open(world)
        for line in f:
            coords.append(line.split(",\n"))
        
        walls = coords[0][0].split(' ')
        obstacles = coords[2:]
        
        #Build walls of map:
        min_x = make_tuple(walls[2])[0]
        max_x = make_tuple(walls[0])[0]
        min_y = make_tuple(walls[3])[1]
        max_y = make_tuple(walls[1])[1]

        self.ax = plt.axes(xlim=(min_x, max_x), ylim=(min_y, max_y))
        
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
        self.ax.add_patch(patch)

    def plot(self):
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