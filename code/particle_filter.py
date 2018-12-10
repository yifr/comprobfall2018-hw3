import time
import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import math
import random
from math import pi as pi
import heapq as hq
import scipy.stats as st

from shapely.geometry import Point
from shapely.geometry import LineString
from shapely.geometry.polygon import Polygon
from shapely.geometry import mapping

from Map import Map_2D

class Particle_Filter():
    x_start = 0
    y_start = 0
    messages = []   
    particles = []
    scan_noise = 0.1
    
    #Initialize filter with n uniformly distributed particles
    def __init__(self, _map, n):
        self.n = n
        self.map=_map
        for i in range(n):
            particle = Particle(random.uniform(_map.min_x, _map.max_x), \
                                random.uniform(_map.min_y, _map.max_y), \
                                random.uniform(0, 2*pi))
            self.particles.append(particle)

    def compute_weights(self, message):
        mean_robot_dist = sum([distance for distance in message.scan_data if not math.isnan(distance)]) / len(message.scan_data)
        std = self.scan_noise
        for particle in self.particles:
            mean_particle_dist = sum([distance for distance in particle.distances if not math.isnan(distance)]) 
            mean_particle_dist /= len(particle.distances)
            d = mean_particle_dist - mean_robot_dist
            weight = 1/(math.sqrt(2*pi)*std)*math.e**-(d**2/(2*std**2))
            particle.weight = weight / self.n
            
        print "Particles sum to: "
        p = [particle.weight for particle in self.particles]
        print p
        print sum(p)
    
    def resample(self):
        new_particles=[]
        weights=[]
        tot_weight=0
        
        """Normalize Weights and set up 0 to 1 range"""
        for p in self.particles:
            weights.append(p.weight)
            tot_weight+=p.weight
        counter=0
        while counter<len(weights):
            weights[counter]=weights[counter]/tot_weight
            if counter>0:
                weights[counter]=weights[counter]+weights[counter-1]
                counter+=1
        
        """Resample with weight constraints"""
        counter=0
        while counter<self.n:
            """Get particle index for neighborhood"""
            mark=random.random()
            part_index=0
            while mark>weights[part_index]:
                part_index+=1
            
            """Get new particle pose"""
            heading=math.pi*random.random()
            dist=st.norm.ppf(random.random())*self.scan_noise
            x=self.particles[part_index].x+math.cos(heading)*dist
            y=self.particles[part_index].x+math.sin(heading)*dist
            """sample new point if collision"""
            while map.collide((x,y)):
                heading=math.pi*random.random()
                dist=st.norm.ppf(random.random())*self.scan_noise
                x=self.particles[part_index].x+math.cos(heading)*dist
                y=self.particles[part_index].x+math.sin(heading)*dist
            
            theta=random.random()*360.0
            
            new_particles.append(Particle(x,y,theta))
            counter+=1
        self.particles=new_particles
        
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

class Particle:
    x = 0
    y = 0
    theta = 0
    weight = 0
    distances = []  #Maintained as a min heap

    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

    def scan(self, _map):
        self.distances = []     #Refresh distance list each time scan is called
        obstacles = _map.obstacles
        #Take scans from -30 to 30 degrees at 1.125 intervals
        scan_angle = 30
        for i in range(54):
            angle = self.theta - scan_angle
            scan_point = (10*math.cos(angle),  10*math.sin(angle))
            line = LineString([(self.x, self.y), scan_point])
            linestring_type = type(line)    #used to check if intersection is shapely LineString object or MultiLineString object
            for o in obstacles:
                collision = line.intersection(o)
                if collision:
                    #Compute distance
                    if type(collision) != linestring_type:  #MultiLineStringObject (we may pass through two obstacles)
                        l = []
                        for i in collision.geoms:
                            linestring = (i.coords[0], i.coords[1])
                            l.append(linestring)
                    else:
                        l = [(collision.coords[0], collision.coords[1])]
                    distance = self.nearest_collision_point_on_line(l)
                    hq.heappush(self.distances, distance)
                else:
                    distance = float('NaN')
                    hq.heappush(self.distances, distance)
            scan_angle -= 1.125
        
    def nearest_collision_point_on_line(self, points):
        #print 'POINTS:' , points
        min_dist = 100000
        for i in range(len(points)):
            for j in range(len(points[i])):
                #print points[i][j]
                dist = math.sqrt((self.x - points[i][j][0])**2 + (self.y - points[i][j][1])**2) 
                if dist < min_dist:
                    #print 'Swapping distances. \tCurrent min: ', min_dist, '\tNew min: ', dist
                    min_dist = dist
        return dist


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
        unordered_scan_data = [float(elem) for elem in raw_message[29].split(':')[1][2:-2].split(',')]
        for sd in unordered_scan_data:
            hq.heappush(message.scan_data, sd)
        pf.messages.append(message)

def main():
    number = raw_input("Which map do you want to visualize? \nEnter 1, 2, 3, 4, 5, 6 or 7 to continue: ")
    print 
    f1 = "../turtlebot_maps/map_"+str(number)+".txt"
    map1 = Map_2D(f1)
    #map1.plot()

    f2 = '../turtlebot_maps/trajectories/trajectories_'+str(number)+'.txt'
    start = time.time()
    pf = Particle_Filter(map1, 100)
    parse_trajectories(f2, pf)
    for particle in pf.particles:
        particle.scan(map1)
    pf.compute_weights(pf.messages[0])
    end = time.time()
    print ("Total time taken: ", end - start)
    #print pf.particles

def test():
    print math.sin(math.pi/2)

if __name__=='__main__':
    main()