'''
Created on Sep 25, 2012

@author: hitokazu

Agent only with attractive potential fields
This will be merged with agents with other potential fields later
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
        self.repulsive_radius = 10
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
        
#        if shoot == True:
#            if angle == True:
#                for tank in mytanks:
#                    self.shoot(tank)
#                    self.change_angle(tank)
#            else:
#                for tank in mytanks:
#                    self.shoot(tank)
#                    self.move_forward(tank)
#        else:
#            if angle == True:
#                for tank in mytanks:
#                    self.change_angle(tank)
#            else:
#                for tank in mytanks:
#                    self.move_forward(tank)

        results = self.bzrc.do_commands(self.commands)
        
    def get_direction(self, tank):
        """ Get the moving direction based on the strongest attractive vector """
        angle, delta_x, delta_y = self.compute_attractive_vectors(tank) # compute the strongest attractive vector and the target flag
        relative_angle = self.normalize_angle(angle - tank.angle)
        print "relative angle: %f" % relative_angle
        command = Command(tank.index, 1, 2*relative_angle, False)
        self.commands.append(command)
        
    def compute_attractive_vectors(self, tank):
        """ computer the strongest attractive vector and return the direction and the angle """
        
        min_d = float("inf")
        best_flag = None

        for flag in self.flags:
            if flag.color != self.constants['team']:
                d = (flag.x - tank.x)**2 + (flag.x - tank.y)**2 # get distance between tank and flag
                if d < min_d:
                    min_d = d
                    best_flag = flag

        #print "color: %s \t d: %f" % (best_flag.color, d)
                    
        theta = math.atan2(best_flag.y-tank.y, best_flag.x-tank.x) # compute the angle between tank and flag
        delta_x = self.attractive_alpha * self.attractive_s * min_d * math.cos(theta)
        delta_y = self.attractive_alpha * self.attractive_s * min_d * math.sin(theta)
                
#        for flag in self.flags:
#            if flag.color != self.constants['team']:
#                d = math.sqrt((flag.x - tank.x)**2 + (flag.x - tank.y)**2) # get distance between tank and flag
#                if d == 0: # if tank reaches the flag
#                    delta_x = delta_y = 0
#                    break
#                else:              
#                    theta = math.atan2(flag.y-tank.y, flag.x-tank.x) # compute the angle between tank and flag
#                    cur_delta_x = d * math.cos(theta)
#                    cur_delta_y = d * math.sin(theta)
#                    cur_vector = cur_delta_x**2 + cur_delta_y ** 2
#                    print "color: %s \t cur_vector: %f" % (flag.color, cur_vector)
#                    if max_vector < cur_vector:
#                        delta_x = cur_delta_x
#                        delta_y = cur_delta_y
#                        best_theta = theta
#                        max_vector = cur_vector
#                        best_flag = flag
        
        #print "closest flag: %s" % best_flag.color
        return (theta, delta_x, delta_y)
            
    def shoot(self, tank):
        command = Command(tank.index, 0, 0, True)
        self.commands.append(command)

    def move_to_position(self, tank, target_x, target_y):
        """Set command to move to given coordinates."""
        target_angle = math.atan2(target_y - tank.y,
                                  target_x - tank.x)
        relative_angle = self.normalize_angle(target_angle - tank.angle)
        command = Command(tank.index, 1, 2 * relative_angle, True)
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

