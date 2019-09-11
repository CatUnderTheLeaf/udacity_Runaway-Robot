# ----------
# Part Four
#
# Again, you'll track down and recover the runaway Traxbot.
# But this time, your speed will be about the same as the runaway bot.
# This may require more careful planning than you used last time.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time.
#
# ----------
# GRADING
#
# Same as part 3. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random


def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER=None):
    # This function will be called after each time the target moves.

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.

    if not OTHER:  # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = []
        predicted_positions = []
        OTHER = (
        measurements, hunter_positions, hunter_headings, predicted_positions)  # now I can keep track of history
    else:  # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        measurements, hunter_positions, hunter_headings, predicted_positions = OTHER  # now I can always refer to these variables

    # print(target_measurement)
    # print (len(measurements))
    if len(measurements) > 20:

        new_positions = particle_filter(measurements)

        # print(predicted_positions)
        predicted_positions.append(new_positions)
    
        # calculate distances from hunter_position to all predicted positions
        # and divide it by max_distance to know how many steps we need to do to achieve all positions
        # select point which takes us the same amount of steps or less as the chased robot
        dist_to_pred_points = []
        min_point = 0, 0
        for i in range(len(predicted_positions[-1])):
            dist_to_pred_points.append(
                ceil(distance_between(hunter_position, predicted_positions[-1][i]) / max_distance))
            if dist_to_pred_points[i] <= i:
                min_point = predicted_positions[-1][i]
                break
        # print(dist_to_pred_points)
        # print(min_point)

        hunter_headings.append(min_point)

    if len(hunter_headings) > 2:
        xy_estimate = hunter_headings[-1]
    else:
        hunter_headings.append(measurements[-1])
        xy_estimate = measurements[-1]

    heading_to_target = get_heading(hunter_position, xy_estimate)
    # print(heading_to_target)

    heading_difference = angle_trunc(heading_to_target - hunter_heading)
    turning = heading_difference  # turn towards the target
    # print(turning)
    if distance_between(xy_estimate, hunter_position) < max_distance:
        distance = distance_between(xy_estimate, hunter_position)
    else:
        distance = max_distance  # full speed ahead!
    return turning, distance, OTHER


def particle_filter(measurements, N=500):
    # --------
    #
    # calculate average distance and angle
    #
    # print('measurements sent to me')
    # print(measurements)
    avg_dist = 0
    avg_turn_angle = 0
    for i in range(len(measurements) - 2):
        avg_dist += distance_between(measurements[i], measurements[i + 1])
        x1, y1 = measurements[i]
        x2, y2 = measurements[i + 1]
        x3, y3 = measurements[i + 2]
        dist1 = distance_between((x2, y2), (x3, y3))
        dist2 = distance_between((x1, y1), (x2, y2))
        a_x, a_y = x2 - x1, y2 - y1
        b_x, b_y = x3 - x2, y3 - y2
        avg_turn_angle += acos((a_x * b_x + a_y * b_y) / (dist1 * dist2))
    avg_dist = avg_dist / (len(measurements) - 2)
    avg_turn_angle = avg_turn_angle / (len(measurements) - 2)
    # print('avg_dist')
    # print(avg_dist)
    # print('avg_turn_angle')
    # print(avg_turn_angle)
    p = []
    x1, y1 = measurements[0]
    x2, y2 = measurements[1]
    init_heading = atan2(y2 - y1, x2 - x1)
    # --------
    #
    # Make particles
    #
    for i in range(N):
        r = robot(random.gauss(x1, measurement_noise), random.gauss(y1, measurement_noise),
                  random.gauss(init_heading, measurement_noise), random.gauss(avg_turn_angle, measurement_noise),
                  avg_dist)
        r.set_noise(measurement_noise, measurement_noise, measurement_noise)
        p.append(r)
    # print('creating particles')
    # print(p)
    # --------
    #
    # Update particles
    #

    for t in range(len(measurements) - 1):
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
            error_bearing = distance_between(measurements[t + 1], (x, y))
            # print('error_bearing')
            # print(error_bearing)
            # update Gaussian
            error = exp(- ((error_bearing) ** 2) / (measurement_noise ** 2) / 2.0) / sqrt(
                2.0 * pi * (measurement_noise ** 2))
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
    pred_rob = get_mean_pos(p, avg_turn_angle, avg_dist)

    #predicting new positions in circular movement
    whole_circle = False
    ctr = 0
    circle_tolerance = avg_dist
    new_positions = []
    # print(len(measurements))
    while not whole_circle and ctr < 100:
        # print('making circle')
        circle_arc = distance_between((pred_rob.x, pred_rob.y), measurements[-3])
        if circle_arc < circle_tolerance:
            whole_circle = True
        else:
            new_positions.append((pred_rob.x, pred_rob.y))
            pred_rob.move_in_circle()
        ctr += 1
    # print('new_positions')
    # print(new_positions)
    return new_positions

def get_mean_pos(particle_robots, angle, dist):
    x = 0.0
    y = 0.0
    heading = 0.0
    for i in range(len(particle_robots)):
        x += particle_robots[i].x
        y += particle_robots[i].y
        heading += (((particle_robots[i].heading - particle_robots[0].heading + pi) % (2.0 * pi))
                    + particle_robots[0].heading - pi)
    return robot(x / len(particle_robots), y / len(particle_robots), heading / len(particle_robots), angle, dist)


def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 500:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)
        
        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1            
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught



def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all 
    the target measurements, hunter positions, and hunter headings over time, but it doesn't 
    do anything with that information."""
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings) # now I can keep track of history
    else: # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER # now I can always refer to these variables
    
    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning =  heading_difference # turn towards the target
    distance = max_distance # full speed ahead!
    return turning, distance, OTHER

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = .05*target.distance
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

# print demo_grading(hunter, target, naive_next_move)
print demo_grading(hunter, target, next_move)
