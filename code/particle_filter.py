import os
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

from collections import Counter
from Map import Map_2D

class Particle_Filter():
    x_start = 0
    y_start = 0
    messages = []   
    scan_noise = 0.1
    
    #Initialize filter with n uniformly distributed particles
    def __init__(self, _map, n):
        self.n = n
        self.map=_map
        self.particles = []

    def get_particle_scans(self, _map):
        for particle in self.particles:
            particle.scan(_map)
    
    def compute_weights(self, message):
        mean_robot_dist = sum([distance for distance in message.scan_data if not math.isnan(distance)]) / len(message.scan_data)
        std = self.scan_noise
        for particle in self.particles:
            mean_particle_dist = sum([distance for distance in particle.distances if not math.isnan(distance)])
            mean_particle_dist /= len(particle.distances)
            d = mean_particle_dist - mean_robot_dist
            weight = 1/(math.sqrt(2*pi)*std)*math.e**-(d**2/(2*std**2)) + 0.00001
            particle.weight = weight / self.n
        
        #Normalize weights:
        weights = [particle.weight for particle in self.particles]
        total_weight = sum(weights)
        for particle in self.particles:
            particle.weight /= total_weight
            
#        print "Particles sum to: "
#        p = [particle.weight for particle in self.particles]
#        print p
#        print sum(p)
    
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
            y=self.particles[part_index].y+math.sin(heading)*dist
            """sample new point if collision"""
            while self.map.collide((x,y)):
                heading=math.pi*random.random()
                dist=st.norm.ppf(random.random())*self.scan_noise
                x=self.particles[part_index].x+math.cos(heading)*dist
                y=self.particles[part_index].y+math.sin(heading)*dist
            
            theta=random.random()*2*math.pi
            
            """Scan and add particle"""
            particle=Particle(x,y,theta)
            #particle.scan(self.map)
            new_particles.append(particle)
            counter+=1
            
        self.particles=new_particles
    
    def particles_to_map(self):
        particles_xy=[]
        for part in self.particles:
            particles_xy.append((part.x,part.y))
        self.map.particles_list.append(particles_xy)
    
    def propogate(self,message):
        #code for propogation
        for pt in self.particles:
            heading=st.norm.ppf(random.random())*self.scan_noise+message.noisy_heading
            dist=st.norm.ppf(random.random())*self.scan_noise+message.noisy_distance
            x=pt.x+math.cos(heading)*dist
            y=pt.y+math.sin(heading)*dist
            while self.map.collide((x,y)):
                print ("\t\tSampling new collision free point.")
                heading=st.norm.ppf(random.random())*self.scan_noise+message.noisy_heading
                dist=st.norm.ppf(random.random())*self.scan_noise+message.noisy_distance
                x=pt.x+math.cos(heading)*dist
                y=pt.y+math.sin(heading)*dist
            pt.x=x
            pt.y=y
            pt.theta=heading
            start = time.time()
            pt.scan(self.map)
            stop = time.time()
            #print "Scanning took: " + str(stop - start) + " seconds."

    def intSample(self,known_start):
        new_particles=[]
        
        if known_start:
            for i in range(self.n):
                """Get particle pose """
                heading=math.pi*random.random()
                dist=st.norm.ppf(random.random())*self.scan_noise
                x=self.x_start+math.cos(heading)*dist
                y=self.y_start+math.sin(heading)*dist
                """sample new point if collision"""
                while self.map.collide((x,y)):
                    heading=math.pi*random.random()
                    dist=st.norm.ppf(random.random())*self.scan_noise
                    x=self.x_start+math.cos(heading)*dist
                    y=self.y_start+math.sin(heading)*dist
                    print x," ",y
                
                theta=random.random()*360.0
                
                """Scan and add new particle"""
                particle=Particle(x,y,theta)
                particle.scan(self.map)
                new_particles.append(particle)
        else:
            for i in range(self.n):
                particle = Particle(random.uniform(self.map.min_x, self.map.max_x), \
                                random.uniform(self.map.min_y, self.map.max_y), \
                                random.uniform(0, 2*pi))
                """Scan and add new particle"""
                particle.scan(self.map)
                new_particles.append(particle)
        self.particles=new_particles

    def iterate(self,message, start_time=0):
        #main()
#        self.intSample()
        
#        pf = Particle_Filter(map1, 100)
#        parse_trajectories(f2, pf)
#        for particle in pf.particles:
#            particle.scan(map1)
#        pf.compute_weights(pf.messages[0])
#        end = time.time()
#        print ("Total time taken: ", end - start)
        
        self.propogate(message)
        c1_time = time.time()
        print "\t Completed propogating (" + str(c1_time-start_time) + ") total seconds taken."
        self.compute_weights(message)
        c2_time = time.time()
        print "\t Computed Weights (" + str(c2_time - c1_time) + ") total seconds taken."
        self.resample()
        c3_time = time.time()
        print "\t Resampled Particles (" + str(c3_time - c2_time ) + ") total seconds taken."
        print

    

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
            angle = math.degrees(self.theta) - scan_angle
            scan_point = (10*math.cos(math.radians(angle)) + self.x,  10*math.sin(math.radians(angle)) + self.y)
            line = LineString([(self.x, self.y), scan_point])
            closest_collision_distance = 10000
    
            #Find closest obstacle for collision:
            for o in obstacles:
                collision = line.intersection(o)
                distance = self.compute_distance(collision)
                if distance < closest_collision_distance:
                    closest_collision_distance = distance

            if closest_collision_distance == 10000:
                closest_collision_distance = float('NaN')

            hq.heappush(self.distances, closest_collision_distance)
            scan_angle -= 1.125

    def compute_distance(self, collision):
        linestring_type = LineString([(0,0), (2,2)]).geom_type
        point_type = Point(0,0).geom_type

        #Intersection with wall
        if collision.geom_type == point_type:
            (collision_x, collision_y) = (collision.coords[0][0], collision.coords[0][1])
            dist = math.sqrt((self.x - collision_x)**2 + (self.y - collision_y)**2) 
            if dist >= 0.45 and dist <= 10.0:
                return dist
            else:
                return 10000

        #Intersection through obstacle
        elif collision.geom_type == linestring_type:
            endpoints = [collision.coords[0], collision.coords[1]]
            min_dist = 10000
            for tup in endpoints:
                (collision_x, collision_y) = (tup[0], tup[1])
                dist = math.sqrt((self.x - collision_x)**2 + (self.y - collision_y)**2) 
                min_dist = min(min_dist, dist)
                break   #First point is the closest - loop is unnecessary, as it turns out.
            if min_dist >= 0.45 and min_dist <= 10.0:
                return min_dist
            else:
                return 10000

        #MultiLineString Intersection: Intersected disjoint objects
        else:
            min_dist = 10000
            for linestring in collision.geoms:
                endpoints = (linestring.coords[0], linestring.coords[1])
                for (collision_x, collision_y) in endpoints:
                    dist = math.sqrt((self.x - collision_x)**2 + (self.y - collision_y)**2) 
                    min_dist = min(min_dist, dist)
                    break #Again, first point is always the closest - this loop is also unnecessary
                break
            if min_dist >= 0.45 and min_dist <= 10.0:
                return min_dist
            else:
                return 10000


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
    
    def __init__(self):
        self.heading = 0
        self.distance = 0
        self.position = (0,0,0)
        self.orientation = (0,0,0,0)
        self.linear_twist = (0,0,0)
        self.angular_twist = (0,0,0)

        self.noisy_heading = 0
        self.noisy_distance = 0

        self.scan_data = []

    def display(self):
        print "================================================================================================"
        print "Heading:"
        print "\t" + str(self.heading)
        print "Distance:"
        print "\t" + str(self.distance)
        print "Position:"
        print "\t" + str(self.position)
        print "Orientation:"
        print "\t" + str(self.orientation)
        print "Linear Twist:"
        print "\t" + str(self.linear_twist)
        print "Angular Twist:"
        print "\t" + str(self.angular_twist)
        print "Noisy Heading:"
        print "\t" + str(self.noisy_heading)
        print "Noisy Distance:"
        print "\t" + str(self.noisy_distance)
        print "Scan Data (" + str(len(self.scan_data)) + " observations):"
        i = 0
        for scan in self.scan_data:
            print str(scan) + ",",
            i += 1
            if i == 6:
                print "\n"
                i = 0
        print "================================================================================================"
        print


#Param: Path to trajectory file
#       pf: particle filter object
def parse_trajectories(path_to_trajectory_file, pf):
    f = open(path_to_trajectory_file)
    lines = f.readlines()

    pf.x_start = int(lines[1].split(':')[1])
    pf.y_start = int(lines[2].split(':')[1])

    raw_messages=[]

    i = 4
    while i < len(lines):
        #Separate individual messages in trajectory file
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
    
    #Parse each message for individual content
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

    f2 = '../turtlebot_maps/trajectories/trajectories_'+str(number)+'.txt'
#    runCode()
    
    
#    while there are new messages do:
#        propogate()
#        compute_weights()
#        resample()
    
    
    """Initialize particle filter"""
    pf = Particle_Filter(map1, 50)
    
    """Retrieve Data"""
    parse_trajectories(f2, pf)
    
    """Initialize First Sample"""
    pf.intSample(True)

    start = time.time()

    pf.particles_to_map()
    for message in pf.messages:
        ctime = time.time()
        print "Iterating..."
        pf.iterate(message, ctime)
        pf.particles_to_map()
    
    """Print robot Path"""
    prev=(pf.x_start,pf.y_start)
    for message in pf.messages:
        new_x=prev[0]+math.cos(message.heading)*message.distance
        new_y=prev[1]+math.sin(message.heading)*message.distance
        current=(new_x,new_y)
        map1.path.append([prev,current])
        prev=current
    end = time.time()  
    map1.plot()
    
    #pf.compute_weights(pf.messages[0])
    
    print ("Total time taken: ", end - start)
    
    #print pf.particles

def test():
    print math.sin(math.pi/2)

if __name__=='__main__':
    main()