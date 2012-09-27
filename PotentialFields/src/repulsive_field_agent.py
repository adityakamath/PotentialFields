'''
Created on Sep 25, 2012

@author: hitokazu

Agent only with attractive potential fields
This will be merged with agents with other potential fields later
'''
from __future__ import division
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
        self.s = 10
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
        angle, delta_x, delta_y = self.compute_repulsive_wall(tank) # compute the strongest attractive vector and the target flag
        relative_angle =0
        #print "Angle: %f" % angle
        if angle != 0:
            relative_angle = self.normalize_angle(tank.angle - angle)            
            #print "relative angle: %f" % relative_angle
        command = Command(tank.index, 1, relative_angle, False)
        self.commands.append(command)
        
    def compute_repulsive_vectors(self, tank):
        """ computer the strongest attractive vector and return the direction and the angle """
        
        delta_x = delta_y = 0
        min_vector = float("inf")
        theta = 0
        best_flag = None

        obstacles = self.bzrc.get_obstacles();
        for obstacle in obstacles:
            ox = (obstacle[2][0]+ obstacle[0][0])/2
            oy = (obstacle[2][1]+ obstacle[0][1])/2
            r = math.sqrt((obstacle[0][0] - obstacle[2][0])**2 + (obstacle[0][1] - obstacle[2][1])**2)/2
            d = math.sqrt((ox - tank.x)**2 + (oy - tank.y)**2)
#            print "obstacle: %s" % (obstacle)
            
            if d < (self.s + r):
                theta = math.atan2(oy-tank.y, ox-tank.x) # compute the angle between tank and flag
                print "Theta: %f" % theta
                if d < r:
                    delta_x = -math.cos(theta)*float("inf")
                    delta_y = -math.sin(theta)*float("inf")
                else:
                    delta_x = -self.repulsive_beta * (self.s + r - d)* math.cos(theta)
                    delta_y = -self.repulsive_beta * (self.s + r - d)*math.sin(theta)

        return (theta, delta_x, delta_y)
        #print "closest flag: %s" % best_flag.color
        #return (theta, math.sqrt(min_vector))
    def compute_repulsive_wall(self, tank):
        """ computer the strongest attractive vector and return the direction and the angle """
        divisions = 4
        obstacleDistance = 80
        repulseDistance = obstacleDistance/2
        delta_x = delta_y = 0
        min_vector = float("inf")
        theta = 0
        best_flag = None

        obstacles = self.bzrc.get_obstacles();
        for obstacle in obstacles:
            if self.close_to_points(tank,obstacle,obstacleDistance):
                repulsivePoints = self.get_field_points(obstacle,divisions)
                print "REPOINTS: %s" % repulsivePoints
                for repulsePoint in repulsivePoints:
                    d = math.sqrt((repulsePoint[0] - tank.x)**2 + (repulsePoint[1] - tank.y)**2)
                    if d < repulseDistance:
                        theta = math.atan2(repulsePoint[1]-tank.y, repulsePoint[0]-tank.x) # compute the angle between tank and flag
                        if d <=1:
                            delta_x = -math.cos(theta)*float("inf")
                            delta_y = -math.sin(theta)*float("inf")
                        else:        
                            delta_x = -self.repulsive_beta * (repulseDistance - d)* math.cos(theta)
                            delta_y = -self.repulsive_beta * (repulseDistance - d)*math.sin(theta)              
        return (theta, delta_x, delta_y)
    
    """Tells wether tank is within distance of any of the points"""
    def close_to_points(self,tank,points, distance):
        close = False
        for point in points:
            d = math.sqrt((point[0] - tank.x)**2 + (point[1] - tank.y)**2)
            if d < distance:
                close = True
        return close
    
    """Gives all the points neccessary for generating field for object"""
    def get_field_points(self,edgePoints, divisions):
        haveLast = False
        points = []
        lastPoint = [(0,0)]
        for curPoint in edgePoints:
            if haveLast:
                dx = (curPoint[0] - lastPoint[0])/divisions
                dy = (curPoint[1] - lastPoint[1])/divisions
                for i in xrange(divisions-1):
                    newPoint = [(lastPoint[0]-dx*i,lastPoint[i]-dy*i)]
                    points.append(newPoint)
            haveLast = True
            lastPoint = curPoint
        #from last point to first point
        dx = (edgePoints[0][0] - lastPoint[0])/divisions
        dy = (edgePoints[0][1] - lastPoint[1])/divisions
        for i in xrange(divisions-1):
            newPoint = (lastPoint[0]-dx*i,lastPoint[i]-dy*i)
            points.append(newPoint)
        return points
    
    
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

