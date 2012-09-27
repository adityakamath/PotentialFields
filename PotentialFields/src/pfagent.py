'''
Created on Sep 25, 2012

@author: hitokazu
'''

import sys
import math
import time
import random

from bzrc import BZRC, Command

class Agent(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []
        self.attractive_s= 5
        self.attractive_alpha = 1
        self.repulsive_s = 10
        #self.repulsive_radius = 10
        self.repulsive_beta = 1
        self.speed = 1
        #self.elapsed_time = 0
        #self.moving_time = self.set_moving_time()
        #self.shooting_time = self.set_shooting_time()
        
    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color !=
                        self.constants['team']]

        self.commands = []
        
        for tank in self.mytanks:
            self.get_direction(tank)
        
        results = self.bzrc.do_commands(self.commands)
        
    def get_direction(self, tank):
        """ Get the moving direction based on the strongest attractive vector """
        angle, attractive_delta_x, attractive_delta_y = self.compute_attractive_vectors(tank) # compute the strongest attractive vector and the target flag
        angle, repulsive_delta_x, repulsive_delta_y = self.compute_repulsive_vectors(tank)
        
        print "attractive x: %f \t repulsive x: %f" % (attractive_delta_x, repulsive_delta_x)
        print "attractive y: %f \t repulsive y: %f" % (attractive_delta_y, repulsive_delta_y)
        
        delta_x = attractive_delta_x + repulsive_delta_x
        delta_y = attractive_delta_y + repulsive_delta_y
        
        print "combined delta_x: %f \t combined delta_y: %f" % (delta_x, delta_y)
        
        angle = math.atan2(delta_y, delta_x)
        
        relative_angle = self.normalize_angle(angle - tank.angle)
        #print "relative angle: %f" % relative_angle
        
        command = Command(tank.index, 1, 2*relative_angle, False)
        self.commands.append(command)

    def compute_repulsive_vectors(self, tank):
        """ computer the strongest attractive vector and return the direction and the angle """
        
        delta_x = delta_y = 0
        theta = 0
        Infinity = float("inf")
        d = 0
        obs_radius = 10

        sign = lambda x : cmp(x, 0)

        obstacles = self.bzrc.get_obstacles();
        for obstacle in obstacles:
            # 1. get vertices of obstacle and give the radius of 10 for example
            # 2. calculate distance between tank and obstacle
            # 3. if the vertices are in the range of spread s:
                # a. calculate theta = atan(yo-y, xp-x) 
                # b. if d < r, then delta_x = -sign(cos(theta))infinity, delta_y = -sign(cos(theta))infinity
                # c. if r <= d <= s + r, then delta_x = -beta(s + r - d)cos(theta), delta_y = -beta(s+r-d)sin(theta)
                # d. if d > s + r delta_x = delta_y = 0
            
            for vertex in obstacle:
                d = math.sqrt((vertex[0]-tank.x)**2 + (vertex[1]-tank.x)**2)
                if d > obs_radius + self.repulsive_s:
                    continue
                else:
                    theta = math.atan2(vertex[1]-tank.y, vertex[0]-tank.x)
                    if obs_radius <= d and d <= self.repulsive_s + obs_radius:
                        delta_x += -self.repulsive_beta * (self.repulsive_s + obs_radius - d) * math.cos(theta)
                        delta_y += -self.repulsive_beta * (self.repulsive_s + obs_radius - d) * math.sin(theta)
                    elif d < obs_radius:
                        delta_x += -1 * sign(math.cos(theta)) * Infinity
                        delta_y += -1 * sign(math.sin(theta)) * Infinity
                    
        return (theta, delta_x, delta_y)
        
    def compute_attractive_vectors(self, tank):
        """ computer the strongest attractive vector and return the direction and the angle """
        
        min_d = float("inf")
        best_flag = None

        delta_x = delta_y = 0

        for flag in self.flags:
            if flag.color != self.constants['team'] and flag.poss_color != self.constants['team']:
                d = (flag.x - tank.x)**2 + (flag.x - tank.y)**2 # get distance between tank and flag
                if d < min_d:
                    min_d = d
                    best_flag = flag

        #print "color: %s \t d: %f" % (best_flag.color, d)

        
                    
        theta = math.atan2(best_flag.y-tank.y, best_flag.x-tank.x) # compute the angle between tank and flag
        if min_d >= 0 and d <= self.attractive_s:
            delta_x = self.attractive_alpha * self.attractive_s * min_d * math.cos(theta)
            delta_y = self.attractive_alpha * self.attractive_s * min_d * math.sin(theta)
        elif min_d > self.attractive_s:
            delta_x = self.attractive_alpha * self.attractive_s * math.cos(theta)
            delta_y = self.attractive_alpha * self.attractive_s * math.sin(theta)
            
         
        return (theta, delta_x, delta_y)
            
    def shoot(self, tank):
        command = Command(tank.index, 0, 0, True)
        self.commands.append(command)

    def move_to_position(self, tank, target_x, target_y):
        """Set command to move to given coordinates."""
        target_angle = math.atan2(target_y - tank.y,
                                  target_x - tank.x)
        relative_angle = self.normalize_angle(target_angle - tank.angle)
        command = Command(tank.index, self.speed, 2 * relative_angle, True)
        self.commands.append(command)
    
    def normalize_angle(self, angle):
        """Make any angle be between +/- pi."""
        angle -= 2 * math.pi * int (angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle

def main():
    # Process CLI arguments.
    try:
        execname, host, port = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >>sys.stderr, '%s: incorrect number of arguments' % execname
        print >>sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)

    # Connect.
    #bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))

    agent = Agent(bzrc)

    #agent.elapsed_time = prev_time = time.time()
    time_diff = 0

    #print "Moving Time: %d" % agent.moving_time

    # Run the agent
    try:
        while True: 
            #print "Elapsed Time: %f" % time_diff
            agent.tick(time_diff)
            #for flag in agent.flags:
                #print flag.x, flag.y, flag.color, flag.poss_color
#            time_diff = time.time() - prev_time
#            if time.time() - agent.elapsed_time > agent.shooting_time:
#                print "Shoot!"
#                agent.tick(time_diff, False, True)
#                agent.shooting_time = agent.set_shooting_time()
#                agent.elapsed_time = time.time()
#            if time_diff > agent.moving_time:
#                print "Turning 60 degrees." 
#                if time_diff < agent.moving_time + 0.83:
#                    agent.tick(time_diff, True, False)
#                else:
#                    agent.moving_time = agent.set_moving_time()
#                    prev_time = time.time()
#                    print "Moving Time: %d" % agent.moving_time
#            else:
#                agent.tick(time_diff, False, False)
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()

