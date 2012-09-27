#!/usr/bin/env python
'''This is a demo on how to use Gnuplot for potential fields.  We've
intentionally avoided "giving it all away."
'''

from __future__ import division
from itertools import cycle

try:
    from numpy import linspace
except ImportError:
    # This is stolen from numpy.  If numpy is installed, you don't
    # need this:
    def linspace(start, stop, num=50, endpoint=True, retstep=False):
        """Return evenly spaced numbers.

        Return num evenly spaced samples from start to stop.  If
        endpoint is True, the last sample is stop. If retstep is
        True then return the step value used.
        """
        num = int(num)
        if num <= 0:
            return []
        if endpoint:
            if num == 1:
                return [float(start)]
            step = (stop-start)/float((num-1))
            y = [x * step + start for x in xrange(0, num - 1)]
            y.append(stop)
        else:
            step = (stop-start)/float(num)
            y = [x * step + start for x in xrange(0, num)]
        if retstep:
            return y, step
        else:
            return y


########################################################################
# Constants

# Output file:
FILENAME = 'fields.gpi'
# Size of the world (one of the "constants" in bzflag):
WORLDSIZE = 800
# How many samples to take along each dimension:
SAMPLES = 50
# Change spacing by changing the relative length of the vectors.  It looks
# like scaling by 0.75 is pretty good, but this is adjustable:
VEC_LEN = 0.75 * WORLDSIZE / SAMPLES
# Animation parameters:
ANIMATION_MIN = 0
ANIMATION_MAX = 500
ANIMATION_FRAMES = 20 # this changes the speed of animation


########################################################################
# Field and Obstacle Definitions

# <this is the part we need to change according to our potential fields!!> #

# the first tank position: (364.000000, 2.000000)
# the closest flag position: (0.000000, 370.000000)

import math

OBSTACLES = [((150.0, 150.0), (150.0, 90.0), (90.0, 90.0), (90.0, 150.0)),
                ((150.0, 210.0), (150.0, 150.0), (90.0, 150.0), (90.0, 210.0)),
                ((210.0, 150.0), (210.0, 90.0), (150.0, 90.0), (150.0, 150.0)),
                ((150.0, -90.0), (150.0, -150.0), (90.0, -150.0), (90.0, -90.0)),
                ((210.0, -90.0), (210.0, -150.0), (150.0, -150.0), (150.0, -90.0)),
                ((150.0, -150.0), (150.0, -210.0), (90.0, -210.0), (90.0, -150.0)),
                ((-90.0, -90.0), (-90.0, -150.0), (-150.0, -150.0), (-150.0, -90.0)),
                ((-90.0, -150.0), (-90.0, -210.0), (-150.0, -210.0), (-150.0, -150.0)),
                ((-150.0, -90.0), (-150.0, -150.0), (-210.0, -150.0), (-210.0, -90.0)),
                ((-90.0, 150.0), (-90.0, 90.0), (-150.0, 90.0), (-150.0, 150.0)),
                ((-90.0, 210.0), (-90.0, 150.0), (-150.0, 150.0), (-150.0, 210.0)),
                ((-150.0, 150.0), (-150.0, 90.0), (-210.0, 90.0), (-210.0, 150.0)),
                ((10.0, 60.0), (10.0, -60.0), (-10.0, -60.0), (-10.0, 60.0))]


def generate_field_function(scale, fieldtype):
    
    def attractive(x, y):
        '''User-defined field function.'''

        closest_flag = (0.0, 370.0)
     
        delta_x = delta_y = 0
        
        s= 50
        alpha = 1
        
        d = (closest_flag[0] - x)**2 + (closest_flag[1] - y)**2 # get distance between tank and flag
        
        theta = math.atan2(closest_flag[1]- y, closest_flag[0]-x) # compute the angle between tank and flag
        if d >= 0 and d <= s:
            delta_x = alpha * s * d * math.cos(theta)
            delta_y = alpha * s * d * math.sin(theta)
        elif d > s:
            delta_x = alpha * s * math.cos(theta)
            delta_y = alpha * s * math.sin(theta)
        
        
            
        return delta_x/s, delta_y/s
    
    def repulsive(x, y):
        delta_x = delta_y = 0
        theta = 0
        s = 100
        beta = .5

        for obstacle in OBSTACLES:
            ox = (obstacle[2][0]+ obstacle[0][0])/2
            oy = (obstacle[2][1]+ obstacle[0][1])/2
            r = math.sqrt((obstacle[0][0] - obstacle[2][0])**2 + (obstacle[0][1] - obstacle[2][1])**2)/2
            d = math.sqrt((ox - x)**2 + (oy - y)**2)
            
            sign = lambda x : cmp(x, 0)
            
            if d < (s + r):
                theta = math.atan2(oy-y, ox-x) # compute the angle between tank and flag
                if d < r:
                    delta_x -= sign(math.cos(theta))*1000000
                    delta_y -= sign(math.sin(theta))*1000000
                else:
                    delta_x -= beta * (s + r - d)* math.cos(theta)
                    delta_y -= beta * (s + r - d)*math.sin(theta)

        return delta_x/s, delta_y/s
    
    
    def combination(x, y):
        
        attractive_x, attractive_y = attractive(x, y)
        repulsive_x, repulsive_y = repulsive(x, y)

        #print "attractive: dx: %f, dy: %f, vec: %f" % (attractive_x, attractive_y, math.sqrt(attractive_x**2 + attractive_y**2))
        #print "repulsive: dx: %f, dy: %f, vec: %f" % (repulsive_x, repulsive_y, math.sqrt(repulsive_x**2 + repulsive_y**2))
        #print "combination: dx: %f, dy: %f, vec: %f" % (attractive_x+repulsive_x, attractive_y+repulsive_y, math.sqrt((attractive_x+repulsive_x)**2 + (attractive_y+repulsive_y)**2))
        #print
        
        if abs(repulsive_x) > 500000 and abs(repulsive_y) > 500000:
            repulsive_x = 2
            repulsive_y = 2
        
        return attractive_x+repulsive_x, attractive_y+repulsive_y 
        
    
    
    if fieldtype == "attractive":
        function = attractive
    elif fieldtype == "repulsive":
        function = repulsive
#    elif fieldtype == "tangential":
#        function = tangential
    else:
        function = combination
                
    #        sqnorm = (x**2 + y**2)
    #        if sqnorm == 0.0:
    #            return 0, 0
    #        else:
    #            return x*scale/sqnorm, y*scale/sqnorm
    return function

# this is the four-ls world obstacles


type = "combination"



########################################################################
# Helper Functions

def gpi_point(x, y, vec_x, vec_y):
    '''Create the centered gpi data point (4-tuple) for a position and
    vector.  The vectors are expected to be less than 1 in magnitude,
    and larger values will be scaled down.'''
    
    r = (vec_x ** 2 + vec_y ** 2) ** 0.5
    if r > 1:
        vec_x /= r
        vec_y /= r
    return (x - vec_x * VEC_LEN / 2, y - vec_y * VEC_LEN / 2,
            vec_x * VEC_LEN, vec_y * VEC_LEN)

def gnuplot_header(minimum, maximum):
    '''Return a string that has all of the gnuplot sets and unsets.'''
    s = ''
    s += 'set xrange [%s: %s]\n' % (minimum, maximum)
    s += 'set yrange [%s: %s]\n' % (minimum, maximum)
    # The key is just clutter.  Get rid of it:
    s += 'unset key\n'
    # Make sure the figure is square since the world is square:
    s += 'set size square\n'
    # Add a pretty title (optional):
    #s += "set title 'Potential Fields'\n"
    return s

def draw_line(p1, p2):
    '''Return a string to tell Gnuplot to draw a line from point p1 to
    point p2 in the form of a set command.'''
    x1, y1 = p1
    x2, y2 = p2
    return 'set arrow from %s, %s to %s, %s nohead lt 3\n' % (x1, y1, x2, y2)

def draw_obstacles(obstacles):
    '''Return a string which tells Gnuplot to draw all of the obstacles.'''
    s = 'unset arrow\n'

    for obs in obstacles:
        last_point = obs[0]
        for cur_point in obs[1:]:
            s += draw_line(last_point, cur_point)
            last_point = cur_point
        s += draw_line(last_point, obs[0])
    return s

def plot_field(function):
    '''Return a Gnuplot command to plot a field.'''
    s = "plot '-' with vectors head\n"

    separation = WORLDSIZE / SAMPLES
    end = WORLDSIZE / 2 - separation / 2
    start = -end

    points = ((x, y) for x in linspace(start, end, SAMPLES)
                for y in linspace(start, end, SAMPLES))

    for x, y in points:
        f_x, f_y = function(x, y)
        plotvalues = gpi_point(x, y, f_x, f_y)
        if plotvalues is not None:
            x1, y1, x2, y2 = plotvalues
            s += '%s %s %s %s\n' % (x1, y1, x2, y2)
    s += 'e\n'
    return s


########################################################################
# Plot the potential fields to a file

outfile = open(FILENAME, 'w')
print >>outfile, gnuplot_header(-WORLDSIZE / 2, WORLDSIZE / 2)
print >>outfile, draw_obstacles(OBSTACLES)
field_function = generate_field_function(150, "repulsive")
print >>outfile, plot_field(field_function)


########################################################################
# Animate a changing field, if the Python Gnuplot library is present

try:
    from Gnuplot import GnuplotProcess
except ImportError:
    print "Sorry.  You don't have the Gnuplot module installed."
    import sys
    sys.exit(-1)

forward_list = list(linspace(ANIMATION_MIN, ANIMATION_MAX, ANIMATION_FRAMES/2))
backward_list = list(linspace(ANIMATION_MAX, ANIMATION_MIN, ANIMATION_FRAMES/2))
anim_points = forward_list + backward_list

gp = GnuplotProcess(persist=True)
gp.write(gnuplot_header(-WORLDSIZE / 2, WORLDSIZE / 2))
gp.write(draw_obstacles(OBSTACLES))

#for scale in cycle(anim_points):
#    field_function = generate_field_function(scale)
#    gp.write(plot_field(field_function))

field_function = generate_field_function(150, type)
gp.write(plot_field(field_function))

