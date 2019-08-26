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
import turtle
import random


def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER=None):
    # This function will be called after each time the target moves.

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.

    if not OTHER:  # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        # hunter_headings = [hunter_heading]
        hunter_headings = []
        predicted_positions = []
        OTHER = (measurements, hunter_positions, hunter_headings, predicted_positions)  # now I can keep track of history
    else:  # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        # OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings, predicted_positions = OTHER  # now I can always refer to these variables

    # if len(measurements) < 4:
    #     x_, y_ = target_measurement
    # print(target_measurement)
    # print (len(measurements))
    if len(measurements) > 2:
        avg_dist = 0
        for i in range(len(measurements) - 1):
            avg_dist += distance_between(measurements[i], measurements[i + 1])
        avg_dist = avg_dist / (len(measurements) - 1)
        # print(avg_dist)
        avg_turn_angle = 0
        heading = 0
        for i in range(len(measurements) - 2):
            x1, y1 = measurements[i]
            x2, y2 = measurements[i + 1]
            x3, y3 = measurements[i + 2]
            dist1 = distance_between(measurements[i + 1], measurements[i + 2])
            dist2 = distance_between(measurements[i], measurements[i + 1])
            a_x, a_y = x2 - x1, y2 - y1
            b_x, b_y = x3 - x2, y3 - y2
            avg_turn_angle += acos((a_x * b_x + a_y * b_y) / (dist1 * dist2))
            heading = atan2(b_y, b_x)
        avg_turn_angle = avg_turn_angle / (len(measurements) - 2)
        # print(avg_turn_angle)
        # x_ = x3 + avg_dist * cos(angle_trunc(heading + avg_turn_angle))
        # y_ = y3 + avg_dist * sin(angle_trunc(heading + avg_turn_angle))

        whole_circle = False
        ctr = 0
        circle_tolerance = avg_dist
        new_positions = []
        print(len(measurements))
        while not whole_circle and ctr < 100:
            print('making circle')
            x_ = x3 + avg_dist * cos(angle_trunc(heading + avg_turn_angle))
            y_ = y3 + avg_dist * sin(angle_trunc(heading + avg_turn_angle))
            circle_arc = distance_between((x_, y_), measurements[-2])
            if circle_arc < circle_tolerance:
                whole_circle = True
            else:
                new_positions.append((x_, y_))
                head_x, head_y = x_ - x3, y_ - y3
                heading = atan2(head_y, head_x)
                x3 = x_
                y3 = y_
            ctr += 1
        # print(predicted_positions)
        predicted_positions.append(new_positions)
        # mean_pred_positions = map(lambda x:sum(x)/float(len(x)), zip(*predicted_positions))
        # del hunter_headings[:]
        dist_to_pred_points = []
        min_point = x_, y_
        for i in range(len(predicted_positions[-1])):
            dist_to_pred_points.append(ceil(distance_between(hunter_position, predicted_positions[-1][i])/max_distance))
            if dist_to_pred_points[i] <= i:
                min_point = predicted_positions[-1][i]
                break
        # print(dist_to_pred_points)
        # print(min_point)
        # del dist_to_pred_points[:]
        hunter_headings.append(min_point)
        # x_, y_ = min_point

        # print('predicted heading')
        # print(heading)
        # print(turn_angle)
        # print(2*pi / 34.0)
        # print('new x, y')
        # print(x_, y_)

    if len(hunter_headings) > 2:# and (distance_between(hunter_position, measurements[-1])) > max_distance:
        # print('big dist')
        xy_estimate = hunter_headings[-1]
    else:
        # x_, y_ = target_measurement
        hunter_headings.append(measurements[-1])
        xy_estimate = measurements[-1]
        # print('small dist')
        # print(distance_between(hunter_position, measurements[-1]))
        # print(max_distance)
        # print('xy_estimate')
        # print(xy_estimate)


    # print('target_measurement')
    # print(target_measurement)
    # print('xy_estimate')
    # print(xy_estimate)
    # print('hunter_position')
    # print(hunter_position)
    # print(hunter_headings)

    # heading_to_target = get_heading(hunter_position, target_measurement)
    heading_to_target = get_heading(hunter_position, xy_estimate)
    # print(heading_to_target)

    heading_difference = angle_trunc(heading_to_target - hunter_heading)
    turning = heading_difference  # turn towards the target
    # print(turning)
    if distance_between(xy_estimate, hunter_position) < max_distance:
        distance = distance_between(xy_estimate, hunter_position)
    else:
        distance = max_distance  # full speed ahead!
    # print(distance)
    return turning, distance, OTHER

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


# def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER=None):
#     """Returns True if your next_move_fcn successfully guides the hunter_bot
#     to the target_bot. This function is here to help you understand how we
#     will grade your submission."""
#     max_distance = 0.98 * target_bot.distance  # 0.98 is an example. It will change.
#     separation_tolerance = 0.02 * target_bot.distance  # hunter must be within 0.02 step size to catch target
#     caught = False
#     ctr = 0
#
#     # We will use your next_move_fcn until we catch the target or time expires.
#     while not caught and ctr < 1000:
#
#         # Check to see if the hunter has caught the target.
#         hunter_position = (hunter_bot.x, hunter_bot.y)
#         target_position = (target_bot.x, target_bot.y)
#         separation = distance_between(hunter_position, target_position)
#         if separation < separation_tolerance:
#             print "You got it right! It took you ", ctr, " steps to catch the target."
#             caught = True
#
#         # The target broadcasts its noisy measurement
#         target_measurement = target_bot.sense()
#
#         # This is where YOUR function will be called.
#         turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance,
#                                                  OTHER)
#
#         # Don't try to move faster than allowed!
#         if distance > max_distance:
#             distance = max_distance
#
#         # We move the hunter according to your instructions
#         hunter_bot.move(turning, distance)
#
#         # The target continues its (nearly) circular motion.
#         target_bot.move_in_circle()
#
#         ctr += 1
#         if ctr >= 1000:
#             print "It took too many steps to catch the target."
#     return caught

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    # max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
    max_distance = 1.94 * target_bot.distance  # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    #For Visualization
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')
    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    size_multiplier = 15.0 #change size of animation
    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
    chaser_robot.showturtle()
    broken_robot.hideturtle()
    broken_robot.penup()
    broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
    broken_robot.showturtle()
    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
    broken_robot.pendown()
    chaser_robot.pendown()
    meet_point = turtle.Turtle()
    meet_point.shape('turtle')
    meet_point.color('red')
    meet_point.penup()
    meet_point.resizemode('user')
    meet_point.shapesize(1, 1, 1)
    meet_point.hideturtle()
    predicted = turtle.Turtle()
    predicted.shape('circle')
    predicted.color('black')
    predicted.penup()
    predicted.resizemode('user')
    predicted.shapesize(0.1, 0.1, 0.1)
    predicted.hideturtle()
    #End of Visualization
    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:
        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            meet_point.setpos(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
            meet_point.stamp()
            # caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        if len(OTHER[3]):
            for i in range(len(OTHER[3][-1])):
                x1, y1 = OTHER[3][-1][i]
                predicted.setpos(x1 * size_multiplier, y1 * size_multiplier - 100)
                predicted.stamp()
        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance
        # print('hunter pos before move')
        # print(hunter_bot.x, hunter_bot.y)
        # print('robot pos before move')
        # print(target_bot.x, target_bot.y)
        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)
        # print('hunter pos after move')
        # print(hunter_bot.x, hunter_bot.y)
        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()
        # print('robot pos after move')
        # print(target_bot.x, target_bot.y)
        #Visualize it
        measuredbroken_robot.setheading(target_bot.heading*180/pi)
        measuredbroken_robot.goto(target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-100)
        measuredbroken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        chaser_robot.setheading(hunter_bot.heading*180/pi)
        chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
        #End of visualization
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
    if not OTHER:  # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings)  # now I can keep track of history
    else:  # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER  # now I can always refer to these variables

    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference  # turn towards the target
    distance = max_distance  # full speed ahead!
    return turning, distance, OTHER

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = .05*target.distance
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

# print demo_grading(hunter, target, naive_next_move)
print demo_grading(hunter, target, next_move)





