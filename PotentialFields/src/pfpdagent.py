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
        self.infinity = 10000000
        self.attractive_s = 10
        self.attractive_alpha = 0.7
        self.repulsive_s = 40
        self.repulsive_beta = 1
        self.repulsive_max = self.infinity
        self.max_speed = 1
        self.speed = 1
        self.base_radius = 3
        self.obstacles = self.bzrc.get_obstacles()
        self.fire = True
        self.tangential_clockwise = True#bool(random.getrandbits(1))
        self.kp = 0.75
        self.kd = 0.07
        self.attractive_dx = self.attractive_dy = 0
        self.prev_e_x = 0
        self.prev_e_y = 0
        self.prev_t = 0   
            
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
        #self.mytanks = []
        #self.mytanks.append(mytanks[0])
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color !=
                        self.constants['team']]

        self.commands = []

        for tank in self.mytanks:
            self.prev_t = time.time()
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
    
    def compute_angle(self, tank, delta_x, delta_y):
        angle = math.atan2(delta_y, delta_x)
        relative_angle = self.normalize_angle(angle - tank.angle, tank)
        return relative_angle
        
    def create_move_forward_command(self, tank, delta_x, delta_y):
        """ produce move forward command """
        
        #sign = lambda x : cmp(x, 0)
        pdx, pdy = self.pd_controller(tank, delta_x, delta_y)
        angle = self.compute_angle(tank, pdx, pdy)
        #relative_angle = self.normalize_angle(angle, tank)
        
        command = Command(tank.index, self.speed, 2*angle, self.fire)

        
        return command
                    
    def get_direction(self, tank):
        """ Get the moving direction based on the combined vector """
        
        all_captured = True
        for flag in self.flags:
            if flag.poss_color != self.constants['team']:
                all_captured = False
                break
        
        if tank.flag != "-":
            self.go_back(tank)
        else:
            attractive_delta_x, attractive_delta_y = self.compute_attractive_vectors(tank) # compute the strongest attractive vector and the target flag
            repulsive_delta_x, repulsive_delta_y = self.compute_repulsive_vectors(tank)
            tangential_delta_x, tangential_delta_y = self.compute_tangential_vectors(tank)
                        
            delta_x, delta_y = self.combine_vectors([attractive_delta_x, repulsive_delta_x, tangential_delta_x], \
                                                    [attractive_delta_y, repulsive_delta_y, tangential_delta_y]) 
            
            command = self.create_move_forward_command(tank, delta_x, delta_y)
    
            self.commands.append(command)
            
    def pd_controller(self, tank, delta_x, delta_y):

        sign = lambda x : cmp(x, 0)

        dt = time.time() - self.prev_t
        e_x = self.attractive_dx - delta_x
        e_y = self.attractive_dy - delta_y
        de_x = (e_x-self.prev_e_x)
        de_y = (e_y-self.prev_e_y)
        self.prev_e_x = e_x
        self.prev_e_y = e_y
        
        ax_t = self.kp * e_x + self.kd * de_x
        ay_t = self.kp * e_y + self.kd * de_y
        
        new_delta_x = self.combine_delta_and_error(self.attractive_dx, delta_x, ax_t)
        new_delta_y = self.combine_delta_and_error(self.attractive_dy, delta_y, ay_t)
        #new_delta_y = delta_y + ay_t

        #new_delta_x = self.kp * e_x + self.kd * de_x
        #new_delta_y = self.kp * e_y + self.kd * de_y

        #if tank.index == 0:
            #print "speed: %.2f" % self.speed
            #print "att. dx: %.2f ax_t: %.2f  new dx: %.2f  att. dy: %.2f  ay_t: %.2f  new dy: %.2f" % (self.attractive_dx, ax_t, new_delta_x, \
            #                                                                                                   self.attractive_dy, ay_t, new_delta_y)
 
#        if abs(e_x) >= 0.9 * self.infinity or abs(e_y) >= 0.9 * self.infinity:
#            self.speed = self.max_speed *.6
#        else:
#            self.speed = self.max_speed
         
                
        return (new_delta_x, new_delta_y)

    def combine_delta_and_error(self, target, delta, error):
        sign = lambda x : cmp(x, 0)
        
        target_sign = sign(target)
        error_sign =sign(error)
        
        new_delta = delta + error
        
#        if target_sign != error_sign:
#            f = -1 * (target * error)
#            if delta < f:
#                new_delta = delta - f
#            else:
#                new_delta = delta + f
#        else:
#            f = target * error
#            #new_delta = delta + (target * error)
#            if delta < f/2:
#                new_delta = delta - f
#            else:
#                new_delta = delta + f

        return new_delta

    def compute_attractive_x_and_y(self, flag, d, tank, r):
        if d == 0:
            d = math.sqrt((flag.x - tank.x)**2 + (flag.y-tank.y)**2)
        else:
            d = math.sqrt(d)
        
        if tank.status == "alive" and flag != None and flag.poss_color != self.constants['team']:
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
        
        self.attractive_dx = delta_x/self.attractive_s
        self.attractive_dy = delta_y/self.attractive_s
        
        return (delta_x, delta_y)

    def compute_attractive_vectors(self, tank):
        """ compute the strongest attractive vector and return the direction and the angle """
        
        min_d = self.infinity
        best_flag = None

        for flag in self.flags:
            if flag.poss_color != self.constants['team']:
                if flag.color == self.constants['team'] and (self.myflag.x == flag.x or self.myflag == flag.y):
                    continue
                else:
                    d = ((flag.x - tank.x)**2 + (flag.y - tank.y)**2) # get distance between tank and flag
                    if d < min_d:
                        min_d = d
                        best_flag = flag

        tank.goalx = flag.x    
        tank.goaly = flag.y
        
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
    
    def go_back(self, tank):
        """ go back to the base if tank has the flag """
        
        d = math.sqrt((self.myflag.x - tank.x)**2 + (self.myflag.y - tank.y)**2)
        
        if d <= self.base_radius:
            command = Command(tank.index, 0, 0, False)
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
            tank.goalx = self.myflag.y
            tank.goaly = self.myflag.x
            command = self.create_move_forward_command(tank, delta_x, delta_y)
            self.commands.append(command)

    def compute_repulsive_vectors(self, tank, rotation = False):
        """ computer the strongest repulsive vector and return the direction and the angle """
        delta_x = delta_y = 0
        theta = 0

        sign = lambda x : cmp(x, 0)
                    
        for obstacle in self.obstacles:
            ox = (obstacle[2][0]+ obstacle[0][0])/2
            oy = (obstacle[2][1]+ obstacle[0][1])/2
            r = math.sqrt((ox - obstacle[2][0])**2 + (oy - obstacle[2][1])**2)
            d = math.sqrt((ox - tank.x)**2 + (oy - tank.y)**2)

            if d < (self.repulsive_s + r):
                if rotation == False:
                    theta = math.atan2(oy-tank.y, ox-tank.x) # compute the angle between tank and flag
                else:
                    if self.tangential_clockwise == True:
                        theta = math.atan2(oy-tank.y, ox-tank.x) - math.pi / 2
                        self.tangential_clockwise = True#bool(random.getrandbits(1))
                    else:
                        theta = math.atan2(oy-tank.y, ox-tank.x) + math.pi / 2
                        self.tangential_clockwise = True#bool(random.getrandbits(1))
                if d < r:
                    delta_x -= sign(math.cos(theta)) * self.infinity
                    delta_y -= sign(math.sin(theta)) * self.infinity
                elif r <= d and d <= self.repulsive_s + r: 
                    delta_x -= self.repulsive_beta * (self.repulsive_s + r - d)* math.cos(theta)
                    delta_y -= self.repulsive_beta * (self.repulsive_s + r - d)*math.sin(theta)

        if abs(delta_x) > 10 and abs(delta_y) > 10:
            delta_x = sign(delta_x) * self.repulsive_max
            delta_y = sign(delta_y) * self.repulsive_max
        else:
            delta_x /= self.repulsive_s
            delta_y /= self.repulsive_s

        return (delta_x, delta_y)
                
    
    def compute_tangential_vectors(self, tank):
        """ computer tangential vectors based on repulsive vectors """
        delta_x, delta_y = self.compute_repulsive_vectors(tank, True)
        return (delta_x, delta_y)
        
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

