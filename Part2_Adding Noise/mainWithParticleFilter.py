# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess 
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered 
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position. 
#
# ----------
# GRADING
# 
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random

# This is the function you have to write. Note that measurement is a 
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be 
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    # You must return xy_estimate (x, y), and OTHER (even if it is None) 
    # in this order for grading purposes.
    
    x_, y_ = measurement
    if not OTHER:
        OTHER = []
    OTHER.append(measurement)
    if len(OTHER) > 10:
        
        x_, y_ = particle_filter(OTHER)
      
    xy_estimate = x_, y_ 
    return xy_estimate, OTHER

def particle_filter(measurements, N=100): 
    # --------
    #
    # calculate average distance and angle
    # 
    # print('measurements sent to me')
    # print(measurements)
    avg_dist = 0
    avg_turn_angle = 0
    for i in range(len(measurements)-2):
        avg_dist+=distance_between(measurements[i], measurements[i+1])
        x1, y1 = measurements[i]
        x2, y2 = measurements[i+1]
        x3, y3 = measurements[i+2]
        dist1 = distance_between((x2, y2), (x3, y3))
        dist2 = distance_between((x1, y1), (x2, y2))
        a_x, a_y = x2-x1, y2-y1
        b_x, b_y = x3-x2, y3-y2
        avg_turn_angle+=acos((a_x*b_x + a_y*b_y)/(dist1*dist2))
    avg_dist=avg_dist/(len(measurements)-2)
    avg_turn_angle=avg_turn_angle/(len(measurements)-2)
    # print('avg_dist')
    # print(avg_dist)
    # print('avg_turn_angle')
    # print(avg_turn_angle)
    p = []
    x1, y1 = measurements[0]
    x2, y2 = measurements[1]
    init_heading = atan2(y2-y1, x2-x1)
    # --------
    #
    # Make particles
    # 
    for i in range(N):
        r = robot(random.gauss(x1, measurement_noise), random.gauss(y1, measurement_noise), random.gauss(init_heading, measurement_noise), random.gauss(avg_turn_angle, measurement_noise), avg_dist)
        r.set_noise(measurement_noise, measurement_noise, measurement_noise)
        p.append(r)
    # print('creating particles')
    # print(p)   
    # --------
    #
    # Update particles
    #     
    
    for t in range(len(measurements)-1):
        # motion update (prediction)
        p2 = []
        for i in range(N):
            p[i].move_in_circle()
            p2.append(p[i])
        p = p2
        # print('motion update (prediction)')
        # print(p)
        
        # measurement update
        w = []
        for i in range(N):
            # compute errors
            error = 1.0
            
            x = p[i].x
            y = p[i].y
            error_bearing = distance_between(measurements[t+1], (x, y))
            # print('error_bearing')
            # print(error_bearing)
            # update Gaussian 
            error = exp(- ((error_bearing) ** 2) / (measurement_noise ** 2) / 2.0) / sqrt(2.0 * pi * (measurement_noise ** 2))
            w.append(error)
        # print('compute errors with ', measurements[t+1])
        # print(w)
        # resampling
        p3 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        for i in range(N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            r = robot(p[index].x, p[index].y, p[index].heading, p[index].turning, avg_dist)
            r.set_noise(measurement_noise, measurement_noise, measurement_noise)
            p3.append(r)
        p = p3
        # print('resampling')
        # print(p)
    #  get final position
    x = 0.0
    y = 0.0
    heading = 0.0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
        heading += (((p[i].heading - p[0].heading + pi) % (2.0 * pi)) 
                        + p[0].heading - pi)
    pred_rob = robot(x / len(p), y / len(p), heading / len(p), avg_turn_angle, avg_dist)
    pred_rob.move_in_circle()
    return pred_rob.x, pred_rob.y 
    


# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any 
# information that you want. 
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        # print('measurement')
        # print(measurement)
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        # print('position_guess')
        # print(position_guess)
        # print('true_position')
        # print(true_position)
        # print(error)
        # print('---------------------------------------')
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
    return localized

# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER = None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that 
    position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER 
    return xy_estimate, OTHER

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

# demo_grading(naive_next_pos, test_target)
demo_grading(estimate_next_pos, test_target)
