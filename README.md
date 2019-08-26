Background

A robotics company named Trax has created a line of small self-driving robots 
designed to autonomously traverse desert environments in search of undiscovered
water deposits.

A Traxbot looks like a small tank. Each one is about half a meter long and drives
on two continuous metal tracks. In order to maneuver itself, a Traxbot can do one
of two things: it can drive in a straight line or it can turn. So to make a 
right turn, A Traxbot will drive forward, stop, turn 90 degrees, then continue
driving straight.

This series of questions involves the recovery of a rogue Traxbot. This bot has 
gotten lost somewhere in the desert and is now stuck driving in an almost-circle: it has
been repeatedly driving forward by some step size, stopping, turning a certain 
amount, and repeating this process... Luckily, the Traxbot is still sending all
of its sensor data back to headquarters.

In this project, we will start with a simple version of this problem and 
gradually add complexity. By the end, you will have a fully articulated
plan for recovering the lost Traxbot.

----------
PART ONE

Let's start by thinking about circular motion (well, really it's polygon motion
that is close to circular motion). Assume that Traxbot lives on 
an (x, y) coordinate plane and (for now) is sending you PERFECTLY ACCURATE sensor 
measurements. 

With a few measurements you should be able to figure out the step size and the 
turning angle that Traxbot is moving with.
With these two pieces of information, you should be able to 
write a function that can predict Traxbot's next location.

You can use the robot class that is already written to make your life easier. 
You should re-familiarize yourself with this class, since some of the details
have changed. 

YOUR JOB:
Complete the estimate_next_pos function. You will probably want to use
the OTHER variable to keep track of information about the runaway robot.

----------
PART TWO

Now we'll make the scenario a bit more realistic. Now Traxbot's
sensor measurements are a bit noisy (though its motions are still
completetly noise-free and it still moves in an almost-circle).
You'll have to write a function that takes as input the next
noisy (x, y) sensor measurement and outputs the best guess 
for the robot's next position.

YOUR JOB:
Complete the function estimate_next_pos. You will be considered 
correct if your estimate is within 0.01 stepsizes of Traxbot's next
true position. 

----------
PART THREE

Now you'll actually track down and recover the runaway Traxbot. 
In this step, your speed will be about twice as fast the runaway bot,
which means that your bot's distance parameter will be about twice that
of the runaway. You can move less than this parameter if you'd 
like to slow down your bot near the end of the chase. 

YOUR JOB:
Complete the next_move function. This function will give you access to 
the position and heading of your bot (the hunter); the most recent 
measurement received from the runaway bot (the target), the max distance
your bot can move in a given timestep, and another variable, called 
OTHER, which you can use to keep track of information.

Your function will return the amount you want your bot to turn, the 
distance you want your bot to move, and the OTHER variable, with any
information you want to keep track of.

----------
PART FOUR

Again, you'll track down and recover the runaway Traxbot. 
But this time, your speed will be about the same as the runaway bot. 
This may require more careful planning than you used last time.

YOUR JOB:
Complete the next_move function, similar to how you did last time. 

----------
PART FIVE

This time, the sensor measurements from the runaway Traxbot will be VERY 
noisy (about twice the target's stepsize). You will use this noisy stream
of measurements to localize and catch the target.

YOUR JOB:
Complete the next_move function, similar to how you did last time.
