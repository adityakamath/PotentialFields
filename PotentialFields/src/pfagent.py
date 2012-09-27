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
        self.attractive_s = 10
        self.attractive_alpha = 1
        self.repulsive_s = 30
        self.repulsive_beta = 1
        self.speed = 1
        self.base_radius = 3
        self.obstacles = self.bzrc.get_obstacles()
        self.infinity = 10000000
        flags = self.bzrc.get_flags()
        for flag in flags:
            if flag.color == self.constants['team']:
                self.myflag = flag
                break
        
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
    
    def combine_vectors(self, xlist, ylist):
        """ Combine x components and y components """
        delta_x = delta_y = 0
        for x in xlist:
            delta_x += x
        for y in ylist:
            delta_y += y
        
        return delta_x, delta_y
        
    def create_move_forward_command(self, tank, delta_x, delta_y):
        """ produce move forward command """
        angle = math.atan2(delta_y, delta_x)
        relative_angle = self.normalize_angle(angle - tank.angle)
        return Command(tank.index, 1, 2*relative_angle, True)
        
    def get_direction(self, tank):
        """ Get the moving direction based on the combined vector """
        
        if tank.flag != "-":
            self.go_back(tank)
        else:
            attractive_delta_x, attractive_delta_y = self.compute_attractive_vectors(tank) # compute the strongest attractive vector and the target flag
            repulsive_delta_x, repulsive_delta_y = self.compute_repulsive_vectors(tank)
                        
            delta_x, delta_y = self.combine_vectors([attractive_delta_x, repulsive_delta_x], \
                                                    [attractive_delta_y, repulsive_delta_y]) 
    
            if tank.index == 1:        
                #print "before: %f \t after: %f" % (attractive_delta_x, delta_x)
                print "attractive_x: %f \t attractive_y: %f" % (attractive_delta_x, attractive_delta_y)
                print "repulsive_x: %f \t\t repulsive_y: %f" % (repulsive_delta_x, repulsive_delta_y)
                print "delta_x: %f \t\t delta_y: %f" % (delta_x, delta_y)
                print
    
            command = self.create_move_forward_command(tank, delta_x, delta_y)
            self.commands.append(command)

    def compute_attractive_x_and_y(self, flag, d, tank, r):
        if d == 0:
            d = math.sqrt((flag.x - tank.x)**2 + (flag.y-tank.y)**2)
        else:
            d = math.sqrt(d)
            
        theta = math.atan2(flag.y-tank.y, flag.x-tank.x)
        if d < r:
            delta_x = delta_y = 0
        else:
            cos = math.cos(theta)
            sin = math.sin(theta)
            if r <= d and d <= self.attractive_s + r:
                const = self.attractive_alpha * (d - r)
                delta_x = const * cos
                delta_y = const * (d - r) * sin
            elif d > self.attractive_s + r:
                const = self.attractive_alpha * self.attractive_s
                delta_x = const * cos
                delta_y = const * sin

        return (delta_x, delta_y)

    def compute_attractive_vectors(self, tank):
        """ compute the strongest attractive vector and return the direction and the angle """
        
        min_d = self.infinity
        best_flag = None

        if tank.index == 1:
            print "tank.x: %f \t tank.y: %f" %(tank.x, tank.y)


        for flag in self.flags:
            if flag.color != self.constants['team'] and flag.poss_color != self.constants['team']:
                d = ((flag.x - tank.x)**2 + (flag.y - tank.y)**2) # get distance between tank and flag
                if tank.index == 1:
                    print "flag: %s \t flag.x: %f \t flag.y: %f \t d: %f" % \
                    (flag.color, flag.x, flag.y, d)
                if d < min_d:
                    min_d = d
                    best_flag = flag

        if tank.index == 1:
            print "closest flag: %s" % best_flag.color

        return self.compute_attractive_x_and_y(best_flag, min_d, tank, 0)
    
    def remove_tank(self, tank):
        """ remove tank from self.mytank """
        self.mytank = [t for t in self.mytank if t != tank]
    
    def go_back(self, tank):
        """ go back to the base if tank has the flag """
        
        d = math.sqrt((self.myflag.x - tank.x)**2 + (self.myflag.y - tank.y)**2)
        
        if d <= self.base_radius:
            command = Command(tank.index, 0, 0, False)
            self.remove_tank(tank)
            self.commands.append(command)
        else:
            a_d_x, a_d_y = self.compute_attractive_x_and_y(self.myflag, 0, tank, \
                                                           self.base_radius)
            r_d_x, r_d_y = self.compute_repulsive_vectors(tank)
            delta_x, delta_y = self.combine_vectors([a_d_x, r_d_x], [a_d_y, r_d_y])
            command = self.create_move_forward_command(tank, delta_x, delta_y)
            self.commands.append(command)

    def compute_repulsive_vectors(self, tank):
        """ computer the strongest attractive vector and return the direction and the angle """
        delta_x = delta_y = 0
        theta = 0

        #obstacles = self.bzrc.get_obstacles()
        for obstacle in self.obstacles:
            ox = (obstacle[2][0]+ obstacle[0][0])/2
            oy = (obstacle[2][1]+ obstacle[0][1])/2
            r = math.sqrt((obstacle[0][0] - obstacle[2][0])**2 + (obstacle[0][1] - obstacle[2][1])**2)/2 + 1
            d = math.sqrt((ox - tank.x)**2 + (oy - tank.y)**2)

            sign = lambda x : cmp(x, 0)
            
            if d < (self.repulsive_s + r):
                theta = math.atan2(oy-tank.y, ox-tank.x) # compute the angle between tank and flag
                if d < r:
                    delta_x -= sign(math.cos(theta)) * self.infinity
                    delta_y -= sign(math.sin(theta)) * self.infinity
                else:
                    delta_x -= self.repulsive_beta * (self.repulsive_s + r - d)* math.cos(theta)
                    delta_y -= self.repulsive_beta * (self.repulsive_s + r - d)*math.sin(theta)

        return (delta_x, delta_y)
                
            
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

    time_diff = 0

    # Run the agent
    try:
        while True: 
            agent.tick(time_diff)
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()

