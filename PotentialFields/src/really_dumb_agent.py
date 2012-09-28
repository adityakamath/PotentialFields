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
        self.speed = 0
        self.elapsed_time = 0
        self.moving_time = self.set_moving_time()
        self.shooting_time = self.set_shooting_time()
        
    def set_moving_time(self):
        """ Set time to move forward (3 to 8 sec.)"""
        return random.uniform(3,8) # sample a value between 3 and 8 from the uniform dist.

    def set_shooting_time(self):
        """ Set time to shoot (1.5 and 2.5 sec.) """
        return random.uniform(1.5, 2.5) # sample a value between 1.5 and 2.5 from the uniform dist.

    def tick(self, time_diff, angle, shoot):
        """Some time has passed; decide what to do next."""
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color !=
                        self.constants['team']]

        self.commands = []
        
        if shoot == True:
            if angle == True:
                for tank in mytanks:
                    self.shoot(tank)
                    self.change_angle(tank)
            else:
                for tank in mytanks:
                    self.shoot(tank)
                    self.move_forward(tank)
        else:
            if angle == True:
                for tank in mytanks:
                    self.change_angle(tank)
            else:
                for tank in mytanks:
                    self.move_forward(tank)

        results = self.bzrc.do_commands(self.commands)

    def move_forward(self, tank):
        """ Tanks move forward. """
        self.speed = 0.1 + random.uniform(0, 0.9) # choose speed value between 
        command = Command(tank.index, self.speed, 0, False)
        self.commands.append(command)

    def stop(self, tank):
        command = Command(tank.index, 0, 0, False)
        self.commands.append(command)

    def change_angle(self, tank):
        """ Turn left about 60 degrees """
        command = Command(tank.index, 1, self.get_60_degrees(), False)
        self.commands.append(command)
    
    def get_60_degrees(self):
        """ Rotate 60 degrees to the left """
        return 2 * self.normalize_angle(math.pi/3)
                
    def shoot(self, tank):
        command = Command(tank.index, 0, 0, True)
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

    agent.elapsed_time = prev_time = time.time()
    time_diff = 0

    print "Moving Time: %d" % agent.moving_time

    # Run the agent
    try:
        while True: 
            #print "Elapsed Time: %f" % time_diff
            time_diff = time.time() - prev_time
            if time.time() - agent.elapsed_time > agent.shooting_time:
                #print "Shoot!"
                agent.tick(time_diff, False, True)
                agent.shooting_time = agent.set_shooting_time()
                agent.elapsed_time = time.time()
            if time_diff > agent.moving_time:
                #print "Turning 60 degrees." 
                if time_diff < agent.moving_time + 1.5:
                    agent.tick(time_diff, True, False)
                else:
                    agent.moving_time = agent.set_moving_time()
                    prev_time = time.time()
                    #print "Moving Time: %d" % agent.moving_time
            else:
                agent.tick(time_diff, False, False)
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()

