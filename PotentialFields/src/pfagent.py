'''
Created on Sep 25, 2012

@author: hitokazu
'''

import sys
import math
import time
import random

from bzrc import BZRC, Command, Answer

class Agent(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []
        self.attractive_s = 10
        self.attractive_alpha = 0.7
        self.repulsive_s = 50
        self.repulsive_beta = 1
        self.repulsive_max = 2
        self.speed = 1
        self.base_radius = 3
        self.obstacles = self.bzrc.get_obstacles()
        self.infinity = 10000000
        self.fire = True
        self.tangential_clockwise = True
        flags = self.bzrc.get_flags()
        for flag in flags:
            if flag.color == self.constants['team']:
                self.myflag = Answer()
                self.myflag.color = flag.color
                self.myflag.poss_color = flag.poss_color
                self.myflag.x = flag.x
                self.myflag.y = flag.y
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

        #print "flag.poss_color: %s" % flags[0].poss_color
        
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
        
    def create_move_forward_command(self, tank, delta_x, delta_y, accelx=1, accely=1):
        """ produce move forward command """
        angle = math.atan2(delta_y, delta_x)
        relative_angle = self.normalize_angle(angle - tank.angle, tank)

        command = Command(tank.index, self.speed, 2*relative_angle, self.fire)

        return command
    
    def compute_acceleration(self, delta, attractive):
        sign = lambda x : cmp(x, 0)
        
        if sign(delta) != sign(attractive):
            accel = delta / attractive
        else:
            if delta < 0 and attractive < 0:
                accel = -1 * delta / attractive
            else:
                accel = delta / attractive
                
        return accel
        
        
    def compute_accelx_and_accely(self, delta_x, attractive_x, delta_y, attractive_y):
        """ calculate acceleration for x and y axes """
                
        accelx = self.compute_acceleration(delta_x, attractive_x)
        accely = self.compute_acceleration(delta_y, attractive_y)
        
        return (accelx, accely)
        
    def get_direction(self, tank):
        """ Get the moving direction based on the combined vector """
        
        all_captured = True
        for flag in self.flags:
            if flag.poss_color != self.constants['team']:
                all_captured = False
                break
        
        if tank.flag != "-":
        #if tank.flag != "-" or all_captured == True:
            self.go_back(tank)
        else:
            attractive_delta_x, attractive_delta_y = self.compute_attractive_vectors(tank) # compute the strongest attractive vector and the target flag
            repulsive_delta_x, repulsive_delta_y = self.compute_repulsive_vectors(tank)
            tangential_delta_x, tangential_delta_y = self.compute_tangential_vectors(tank)
                        
            delta_x, delta_y = self.combine_vectors([attractive_delta_x, repulsive_delta_x, tangential_delta_x], \
                                                    [attractive_delta_y, repulsive_delta_y, tangential_delta_y]) 
    
#            if tank.index == 1:        
#                #print "before: %f \t after: %f" % (attractive_delta_x, delta_x)
#                print "attractive_x: %f \t attractive_y: %f" % (attractive_delta_x, attractive_delta_y)
#                print "repulsive_x: %f \t\t repulsive_y: %f" % (repulsive_delta_x, repulsive_delta_y)
#                print "delta_x: %f \t\t delta_y: %f" % (delta_x, delta_y)
#                print
    
            #accelx, accely = self.compute_accelx_and_accely(delta_x, attractive_delta_x, delta_y, attractive_delta_y)
            command = self.create_move_forward_command(tank, delta_x, delta_y)
    
#            if repulsive_delta_x == self.repulsive_max or repulsive_delta_y == self.repulsive_max:
#                command = self.create_move_forward_command(tank, delta_x, delta_y)
#            else:
#                command = self.create_move_forward_command(tank, delta_x, delta_y)
            self.commands.append(command)

    def compute_attractive_x_and_y(self, flag, d, tank, r):
        if d == 0:
            d = math.sqrt((flag.x - tank.x)**2 + (flag.y-tank.y)**2)
        else:
            d = math.sqrt(d)
        
        if tank.status == "alive" and flag != None and flag.poss_color == "none":
            theta = math.atan2(flag.y-tank.y, flag.x-tank.x)
        else:
            theta = 0
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

#        if tank.index == 1:
#            print "tank.x: %f \t tank.y: %f" %(tank.x, tank.y)


        for flag in self.flags:
            if flag.poss_color != self.constants['team']:
                if self.constants['team'] == 'green':
                    print "%s flag.poss_color: %s" % (flag.color, flag.poss_color)
            
            if flag.color != self.constants['team'] and flag.poss_color != self.constants['team']:
                d = ((flag.x - tank.x)**2 + (flag.y - tank.y)**2) # get distance between tank and flag
                if d < min_d:
                    min_d = d
                    best_flag = flag


        #self.closest_flag_report('green', tank, best_flag, min_d)

#        if self.constants['team'] == 'green':
#            if tank.index == 1:
#                if tank.status == 'alive':
#                    print "closest flag: %s \t current position: (%f, %f)" % (best_flag.color, tank.x, tank.y)
#                else:
#                    print "dead now"
            

        delta_x, delta_y = self.compute_attractive_x_and_y(best_flag, min_d, tank, 0)

        return delta_x/self.attractive_s, delta_y/self.attractive_s
    
    def all_flag_report(self, team, flag, d):
        if self.constants['team'] == team:
            for tank in self.mytanks:
                self.flag_report(tank, flag, d, "")

    def closest_flag_report(self, team, tank, flag, d):
        if self.constants['team'] == team:
            self.flag_report(tank, flag, d, "closest")

    def flag_report(self, tank, flag, d, type_of_flag = ""):
        if tank.status == 'alive':
            if type_of_flag != "":
                print "%s flag: %s \t flag.x: %f \t flag.y: %f \t d: %f" % (type_of_flag, flag.color, flag.x, flag.y, d)
            else:
                print "flag: %s \t flag.x: %f \t flag.y: %f \t d: %f" % (flag.color, flag.x, flag.y, d)                
            return True
        else:
            return False
    
    def remove_tank(self, tank):
        """ remove tank from self.mytank """
        self.mytank = [t for t in self.mytank if t != tank]
    
    def go_back(self, tank):
        """ go back to the base if tank has the flag """
        
        d = math.sqrt((self.myflag.x - tank.x)**2 + (self.myflag.y - tank.y)**2)
        
        if d <= self.base_radius:
            command = Command(tank.index, 0, 0, False)
            #self.remove_tank(tank)
            self.commands.append(command)
        else:
            adx, ady = self.compute_attractive_x_and_y(self.myflag, 0, tank, \
                                                           self.base_radius)
            adx /= self.attractive_s
            ady /= self.attractive_s
            rdx, rdy = self.compute_repulsive_vectors(tank)
            tdx, tdy = self.compute_tangential_vectors(tank)
            delta_x, delta_y = self.combine_vectors([adx, rdx, tdx], \
                                                    [ady, rdy, tdy])
            command = self.create_move_forward_command(tank, delta_x, delta_y)
            self.commands.append(command)

    def compute_repulsive_vectors(self, tank, rotation = False):
        """ computer the strongest repulsive vector and return the direction and the angle """
        delta_x = delta_y = 0
        theta = 0
        
        #obstacles = self.bzrc.get_obstacles() # this is very time consuming process. Do it in the constructor.
        for obstacle in self.obstacles:
            ox = (obstacle[2][0]+ obstacle[0][0])/2
            oy = (obstacle[2][1]+ obstacle[0][1])/2
            r = math.sqrt((ox - obstacle[2][0])**2 + (oy - obstacle[2][1])**2)
            #r = math.sqrt((obstacle[0][0] - obstacle[2][0])**2 + (obstacle[0][1] - obstacle[2][1])**2)/2
            d = math.sqrt((ox - tank.x)**2 + (oy - tank.y)**2)

            sign = lambda x : cmp(x, 0)
            
            if d < (self.repulsive_s + r):
                if rotation == False:
                    theta = math.atan2(oy-tank.y, ox-tank.x) # compute the angle between tank and flag
                else:
                    if self.tangential_clockwise == True:
                        theta = math.atan2(oy-tank.y, ox-tank.x) - math.pi / 2
                    else:
                        theta = math.atan2(oy-tank.y, ox-tank.x) + math.pi / 2
                if d < r:
                    delta_x -= sign(math.cos(theta)) * self.infinity
                    delta_y -= sign(math.sin(theta)) * self.infinity
                elif r <= d and d <= self.repulsive_s + r: 
                    delta_x -= self.repulsive_beta * (self.repulsive_s + r - d)* math.cos(theta)
                    delta_y -= self.repulsive_beta * (self.repulsive_s + r - d)*math.sin(theta)

        if delta_x > self.infinity and delta_y > self.infinity:
            delta_x = delta_y = self.repulsive_max
        else:
            delta_x /= self.repulsive_s
            delta_y /= self.repulsive_s

        return (delta_x, delta_y)
                
    
    def compute_tangential_vectors(self, tank):
        """ computer tangential vectors based on repulsive vectors """
        # 1. call compute_repulsive_vectors(self, tank)
        # 2. if self.tangential_clockwise = true
        #        delta_x = delta_y
        #        delta_y = -delta_x
        # 3. else (counter_clockwise)
        #        delta_x = -delta_y
        #        delta_y = delta_x
        delta_x, delta_y = self.compute_repulsive_vectors(tank, True)
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
    
    def normalize_angle(self, angle, tank):
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
            #print "hi"
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()

