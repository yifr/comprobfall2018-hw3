import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import math
import random
from math import pi as pi
import heapq as hq

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.geometry import mapping

from Map import Map_2D

class Particle_Filter():
    x_start = 0
    y_start = 0
    scans = []
    particles = []

    #Initialize filter with n uniformly distributed particles
    def __init__(self, _map, n):
        for i in range(n):
            particle = (random.uniform(_map.min_x, _map.max_x), \
                                random.uniform(_map.min_y, _map.max_y), \
                                random.uniform(0, 2*pi))
            self.particles.append(particle)
'''
class Particle:
    x = 0
    y = 0
    theta = 0

    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta
'''
class Message():
    heading = 0
    distance = 0
    position = (0,0,0)
    orientation = (0,0,0,0)
    linear_twist = (0,0,0)
    angular_twist = (0,0,0)

    noisy_heading = 0
    noisy_distance = 0

    scan_data = []


#Param: Path to trajectory file
#       pf: particle filter object
def parse_trajectories(path_to_trajectory_file, pf):
    f = open(path_to_trajectory_file)
    lines = f.readlines()

    pf.x_start = int(lines[1].split(':')[1])
    pf.y_start = int(lines[2].split(':')[1])
    raw_messages = []

    i = 4
    while i < len(lines):
        current_reading = []
        if 'Heading:' in lines[i]:
            current_reading.append(lines[i])
            i += 1
        
            while i < len(lines) and 'Heading:' not in lines[i]:
                current_reading.append(lines[i])
                i += 1
            raw_messages.append(current_reading)
        else:
            i += 1
    
    for raw_message in raw_messages:
        message = Message()
        message.heading = float(raw_message[0].split(':')[2])
        message.distance = float(raw_message[1].split(':')[2])
        message.position = [float(line.split(':')[1]) for line in raw_message[6:9]]
        message.orientation = [float(line.split(':')[1]) for line in raw_message[10:14]]
        message.linear_twist = [float(line.split(':')[1]) for line in raw_message[16:19]]
        message.angular_twist = [float(line.split(':')[1]) for line in raw_message[20:23]]
        message.noisy_heading = float(raw_message[25].split(':')[1])
        message.noisy_distance = float(raw_message[27].split(':')[1])
        message.scan_data = [float(elem) for elem in raw_message[29].split(':')[1][2:-2].split(',')]

        pf.scans.append(message)

def main():
    number = raw_input("Which map do you want to visualize? \nEnter 1, 2, 3, 4, 5, 6 or 7 to continue:\n\n")
    f1 = "../turtlebot_maps/map_"+str(number)+".txt"
    map1 = Map_2D(f1)
    #map1.plot()

    f2 = '../turtlebot_maps/trajectories/trajectories_'+str(number)+'.txt'
    pf = Particle_Filter(map1, 1000)
    parse_trajectories(f2, pf)
    
    #print pf.particles

if __name__=='__main__':
    main()