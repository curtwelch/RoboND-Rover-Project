import numpy as np
import math

def decision_set_stop(Rover):
    print("SET MODE STOP")
    # Set mode to "stop" and hit the brakes!
    Rover.mode = 'stop'
    Rover.throttle = 0
    # Set brake to stored brake value
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    return Rover

def decision_set_forward(Rover):
    print("SET MODE FORWARD")
    # Just change to forward mode, we will decided what to
    # on next image frame
    Rover.mode = 'forward'
    # The rest will be set later
    Rover.throttle = 0
    Rover.steer = 0
    Rover.brake = 0
    return Rover

def decision_mode_forward(Rover):
    # print("MODE decision_mode_forward()")
    # Rover.mode is 'forward': 

    # Regulate speed to Rover.target_vel

    if Rover.near_sample:
        # stop and grab rock!!
        return decision_set_stop(Rover)

    if Rover.see_rock:
        # We wee a rock, but aren't close enough
        # Are we headed towareds it?  If so, keep going
        # if not, stop, and spin to turn towards it
        # We see a rock!!!  Stop and spin to point towards it
        print("SEE A ROCK SLOWDOWN")
        Rover.target_vel = max(Rover.target_vel, .999) # slowdown
        if abs(Rover.rock_angle) > 40: # Stop and spin
            print("SEE A ROCK STOP AND SPIN")
            return decision_set_stop(Rover)
        # Otherwise, keep going forward and turn

    if Rover.target_vel < 0.2:
        # this means no path forward, stop and do something else
        return decision_set_stop(Rover)

    if Rover.vel < Rover.target_vel:
        # Throttle UP
        Rover.throttle = Rover.throttle_set
        Rover.brake = 0
    elif Rover.vel > Rover.target_vel*1.5:
        # Throttle off, break
        Rover.throttle = 0
        Rover.brake = Rover.brake_set / 2.0 # light brake
    else: # coast
        Rover.throttle = 0
        Rover.brake = 0

    # Set steering
    # Use follow the right wall logic to expore the environment
    # Use the anverage angle for the right and forward vectors to set
    # angle.
    
    if Rover.see_rock:
        Rover.steer = np.clip(Rover.rock_angle/2, -15, 15)
        print("SEE A ROCK, steer to", Rover.steer)
    else:
        # Set steering to average angle clipped to the range +/- 15
        if len(Rover.nav_angles) > 0:
            # Biase 5 deg right to force wall hugging
            bias = -5.0
            bias = 0.0
            Rover.steer = np.clip(Rover.target_angle + bias , -15, 15)
        else:
            Rover.steer = 0

    return Rover

def decision_mode_stop(Rover):
    # print("MODE decision_mode_stop()")
    # If we're in stop mode but still moving keep braking
    if abs(Rover.vel) > 0.2:
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        Rover.steer = 0
        #print("STEER!  set to zero in stop mode")
        return Rover

    # else we're not moving (vel < 0.2) then do something else
    
    # Is there a rock to pick up?  Do it!

    if Rover.near_sample:
        # Keep asking until the simulator tells us it's picking up!
        # It might not pick up if we are still moving?  Ignore that
        # an keep asking.
        if not Rover.picking_up:
            Rover.send_pickup = True
            return Rover

        # Othherwise, it's picking up, so just do nothing an just
        # wait for it to get done.

        return Rover

    if Rover.see_rock:
        # We see a rock!
        print("SEE A ROCK IN STOP, angle={:6.3f} distance={:6.3f}".format(Rover.rock_angle, Rover.rock_dist))
        # Spin to face it if needed
        if abs(Rover.rock_angle) < 10:
            # We are pointed towards it sort of, move forward
            return decision_set_forward(Rover)
        else:
            if Rover.rock_angle > 0:
                a = 5   # Slow spin left
            else:
                a = -5  # Slow spin right
            return decision_set_spin(Rover, a);

    if Rover.target_vel < 0.2:
        # No path forward -- spin to look for options
        if Rover.target_angle > 0:
            a = 15
        else:
            a = -15
        return decision_set_spin(Rover, a)

    # If we're stopped but see sufficient navigable terrain in front then go!
    # We decided if it's safe to go based on target_vel calculated in percpetion

    if Rover.target_vel >= 0.2:
        return decision_set_forward(Rover)

    return Rover

def decision_set_spin(Rover, spin_angle):
    print("Set SPIN to", spin_angle)
    # - is clockwise to the right, + is counterclockwise to the left
    Rover.mode = 'spin'
    Rover.throttle = 0
    # Release the brake to allow turning
    Rover.brake = 0
    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
    Rover.spin_angle = spin_angle
    Rover.steer = Rover.spin_angle
    return Rover

# We are spinning looking for a better path
def decision_mode_spin(Rover):
    # print("MODE decision_mode_spin()")

    # Needs to be reset eaach time because it's updated from
    # server and can get reset to zero

    Rover.steer = Rover.spin_angle
    Rover.throttle = 0 # it gets reset at times by server also

    if Rover.near_sample:
        return decision_set_stop(Rover);

    if Rover.see_rock:
        if abs(Rover.rock_angle) <= 10:
            # stop spinning and go get it
            return decision_set_stop(Rover);
        # else keep spinning
    elif Rover.target_vel > 1.0:
        # If there is no rock, stop spinning and move forward
        # if the way forward is clear enough to allow 1.0 velocity
        return decision_set_stop(Rover);

    return Rover


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function

def decision_step(Rover):
    start_mode = Rover.mode

    r = decision_step2(Rover)

    print("         MODE {:7s}  ".format(start_mode), end='')
    print(" t:{:4.1f}   b:{:4.1f}   s:{:6.2f}".format(Rover.throttle, Rover.brake, Rover.steer), end='')
    if Rover.mode != start_mode:
        print(" -> ", Rover.mode, end='')
    print()

    return r

def decision_step2(Rover):

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:

        if Rover.mode == 'forward': 
            return decision_mode_forward(Rover)

        if Rover.mode == 'stop':
            return decision_mode_stop(Rover)

        if Rover.mode == 'spin':
            return decision_mode_spin(Rover)

        # No mode set?
        # Shoudn't happen, but just in case it doess..

        return decision_set_stop(Rover);

    # Just to make the rover do something 
    # even if no modifications have been made to the code

    Rover.throttle = Rover.throttle_set
    Rover.steer = 0
    Rover.brake = 0
    return Rover

