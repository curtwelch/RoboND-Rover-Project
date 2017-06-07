import numpy as np
import math
import random
import sys
import time

def decision_set_stuck(Rover, forward=False):
    # Oh crap -- try something random for a random bit of time

    Rover.mode = 'stuck'

    Rover.stuck_cnt = 0
    Rover.stuck_end = 10 + random.random() * 200

    Rover.brake = 0.0
    Rover.throttle = 0.0
    Rover.steer = 0.0

    # We set throttle_current to desired throttle and
    # and target_angle to desired steer and reset it on each call
    # just in case server overwrites them.

    if forward:
        # Special request to try and just drive forward (from spin)
        # Try just drivig forward for a while using current setting
        Rover.throttle_current = 1.0
        Rover.target_angle = 0.0
    elif random.random() > .5: # Try spinning
        Rover.throttle_current = 0
        if random.random() > .5:
            Rover.target_angle = -15.0
        else:
            Rover.target_angle = 15.0
    else:
        # Try driving
        max = 50 # blow it out of the water
        #Rover.throttle_current = np.clip(random.random() * Rover.throttle_max, 1.0, Rover.throttle_max)
        Rover.throttle_current = np.clip(random.random() * max, 1.0, max)
        if random.random() > 0.80: # 80% odds of trying reverse
            Rover.throttle_current = -Rover.throttle_current # Try reverse
        Rover.target_angle = np.clip(random.random() * 30 - 15, -15, 15)

    Rover.brake = 0.0
    Rover.throttle = Rover.throttle_current
    Rover.steer = Rover.target_angle

    return Rover

def decision_mode_stuck(Rover):

    Rover.brake = 0.0
    Rover.throttle = Rover.throttle_current
    Rover.steer = Rover.target_angle

    Rover.stuck_cnt += 1

    if Rover.stuck_cnt > Rover.stuck_end:
        return decision_set_stop(Rover)
        
    # Else keep on spinning the wheels

    # Might need a better apporach to understanding if we are unstuck.
    # Path forward doesn't work, beause we could have a wheel stuck on a rock
    # and the camara still looks good.
    # Maybe a distance moved test would be good -- if we are moving, that would
    # tell us we got unstuck sort of.  The fear is that we are really still
    # suck, but it will take the other code a long time to figure that
    # out and come back here.

    return Rover

def decision_set_forward(Rover):
    Rover.mode = 'forward'
    # Set small target_vel to prevent from being confused as hard break request!
    Rover.target_vel = 0.01  # Must be greater than one not to be confused with hard break request
    Rover.throttle_current = 0.0 # take the brake off
    Rover.forward_stuck_cnt = 0
    return Rover

def decision_mode_forward(Rover):

    if Rover.near_sample:
        # stop and grab rock!!
        return decision_set_stop(Rover)

    if Rover.saw_rock:
        # We saw or still see a rock, but aren't close enough to grab it.
        # Are we headed towareds it?  If so, keep going
        # if not, stop, and spin to turn towards it
        Rover.target_vel = 1.0 # Slow speed rock hunting
        if abs(Rover.rock_angle) > 40: # Stop and spin
            #print("SEE A ROCK STOP AND SPIN")
            return decision_set_stop(Rover)
        # Otherwise, keep going forward and turn
        # Try to go forward even if there's no path forward
        Rover.target_angle = Rover.rock_angle
    else:

        # Move forward at the perception recommended speed and direction

        Rover.target_vel = Rover.safe_vel
        Rover.target_angle = Rover.safe_angle

        if Rover.target_vel == 0.0:
            # means no path forward, set stop mode.
            # Let stop figure out what to do
            return decision_set_stop(Rover)

    if Rover.vel > Rover.target_vel or Rover.vel > 0.2:
        Rover.forward_stuck_cnt = 0     # not stuck
    else:
        Rover.forward_stuck_cnt += 1    # Might be stuck

    stuck_limit = 100
    if Rover.saw_rock:
        stuck_limit = 300 # give us more time before we give up if we are rock seeking

    if Rover.forward_stuck_cnt > stuck_limit:
        return decision_set_stuck(Rover)

    return Rover

def drive_rover(Rover):
    # Control the throttle and brake to regulate Rover.vel to Rover.target_vel
    # Steer towards Rover.target_angle
    # Basically a type of PID-like cotroller to create smoother driving behavior.

    # (does negative throttle work to brake?  Haven't tried it)
    # Yes it edoes. And oh crap, -50 or +50 throttle works! Don't know
    # where it's capped. But the rover loses control and spins out at this
    # high numbers.  Speed seems capped at 5 anyway.
    # 10000 works, but it comes back as 1,000.00 which breaks the decoding
    # code that trys to change , to . and float conversion blows up.  I'll just
    # ignore that bug since I have no needs for huge throttle seetings.
    # This code does not support backing up with negative target_vel.
    # But it could.  Should.

    # Warning Rover.vel can go negative when stopping and rocking

    # throttle_tarrget is set to one of -brake_max, 0, or throttle_max.
    # So it's either full speed ahead, or full speed stop.  But them
    # we only slowly adjust throttle/brake as needed to move towareds
    # that target.  The target jumps up and down so then the actual setting
    # stays somewhat stable.
    # Maybe it wouod be better to just code a true PID here???

    if Rover.mode == 'stuck':
        # we reset thsese for stuck mode to keep them from being reset
        Rover.brake = 0
        Rover.throttle = Rover.throttle_current
        Rover.steer = np.clip(Rover.target_angle, -15, 15)
        return Rover

    speed = 0.02

    if Rover.mode == 'spin':
        Rover.throttle_target = 0.0
        Rover.throttle_current = 0.0
    elif Rover.target_vel > 0.0:
        # optimize forward speed but no fear of sudden stop needed
        if Rover.vel < Rover.target_vel:
            # Throttle UP
            Rover.throttle_target = Rover.throttle_max
        else:
            Rover.throttle_target = -Rover.brake_max  # Try heaving braking
            Rover.throttle_target = -1.0  # (light braking possible)
    else:
        # Assume target velocity is zero.  If fact, we just force it since
        # we don't support negative values
        # Does this mean hard stop?  Fast?
        Rover.target_vel = 0.0
        # Well, hell, if we are moving, just slam on the brakes
        # Ok, if we want to slow to a stop, set target_vel to something small, not zero...
        if abs(Rover.vel) > 0.0:
            Rover.throttle_target = -Rover.brake_max
        else:
            # Else we can take the brake off
            # Hopefully we won't roll anywhere in this environment
            Rover.throttle_target = 0
        Rover.throttle_current = Rover.throttle_target # Make it so!

    # Move throttle_current towards throttle_target slowly according to speed

    Rover.throttle_current += (Rover.throttle_target - Rover.throttle_current) * speed

    if Rover.throttle_current == 0:
        Rover.throttle = 0.0
        Rover.brake = 0.0
    elif Rover.throttle_current >= 0:
        Rover.throttle = Rover.throttle_current
        Rover.brake = 0
    else: # < 0 so brake
        Rover.throttle = 0
        Rover.brake = -Rover.throttle_current

    #
    # Now set steering
    #

    # target_angle is where we want to go -- but we set steering
    # to half that and turns torwards in -- reduces osscilations.
    # When spinning, steer ends up controlling spin speed. (throttle is zero)

    if Rover.mode == 'spin':
        Rover.steer = np.clip(Rover.target_angle, -15, 15)
    else:
        # driving -- set to half of the angle
        Rover.steer = np.clip(Rover.target_angle/2.0, -15, 15)

    return Rover

def decision_set_stop(Rover):
    # print("SET MODE STOP")
    # Set mode to "stop" and set target speed to zero.
    # Will stop quickly and smoothly, but not the same as
    # slaming on brakes!
    # We stop so we can pick up a rock or spin to find a new direction

    Rover.mode = 'stop'
    Rover.target_vel = 0

    # Warning, rover starts up in Stop mode, but this setup routing
    # is never called.  Make sure mode_stop code works ok without thi
    # init.

    return Rover

def decision_mode_stop(Rover):

    # print("MODE decision_mode_stop()")
    # If we are still moving just wait for us to stop

    if abs(Rover.vel) > 0.1:
        # Wait for rover to stop.
        # Could be an issue if it's rocking on top of a rock or something.
        # May never stop.  Or .2 may be too high to start a spin?
        return Rover

    # Is there a rock to pick up?  Do it!

    if Rover.near_sample or Rover.picking_up:
        # Keep asking until the simulator tells us it's picking up!
        # It might not pick up if we are still moving?  Ignore that
        # an keep asking.

        if not Rover.picking_up:
            Rover.send_pickup = True
            Rover.rock_forget_time = time.time() # time to forget we ever saw it
            return Rover

        # Otherwise, it's picking up, so just wait.

        Rover.picked_up = True

        return Rover

    if Rover.picked_up:
        # All this just to not double count
        Rover.picked_up = False
        Rover.rock_cnt += 1
    
    if Rover.saw_rock:
        if abs(Rover.rock_angle) < 10:
            # We are pointed towards it sort of, move forward
            # print("SEE A ROCK IN STOP, angle={:6.3f} distance={:6.3f}".format(Rover.rock_angle, Rover.rock_dist))
            return decision_set_forward(Rover)

        # else Spin to try to point at it
        if Rover.rock_angle > 0:
            a = 5   # Slow spin left
        else:
            a = -5  # Slow spin right
        return decision_set_spin(Rover, a);

    good_go_velocity = 0.2

    if Rover.safe_vel < good_go_velocity:
        # No _good_ path forward -- spin to look for options
        if abs(Rover.target_angle) > 10:
        # Strong opinion, spin towards the better path
            if Rover.target_angle > 0:
                a = 15
            else:
                a = -15
        else:
            # Weak opinion, pick ranomly to explore 
            if random.random() > 0.5:
                a = 15
            else:
                a = -15
        return decision_set_spin(Rover, a)

    # forward path is better than 0.2 vel, so lets just try driving again

    return decision_set_forward(Rover)

def decision_set_spin(Rover, spin_angle):
    # - is clockwise to the right, + is counterclockwise to the left
    # In theory, we are already stopped when we try to start a spin
    # But the rover doesn't seem to want to spin if it's moving and I don't
    # Know what happens if it's moving and we set a high steering angle. Will
    # it slow to a stop, then auto fall into spin mode???  Or will it not start
    # a spin ever?  I could spend a few hours testing, but insted, I'll just guess.

    Rover.mode = 'spin'
    Rover.target_vel = 0.0
    # Take the brake off, don't wait -- we assume we are alrady stopped
    # Only the "stop" ommand waits for the rover to stop
    Rover.throttle_current = 0.0 # take the brake off

    # When stopped, setting steer will will induce 4-wheel turning
    Rover.target_angle = spin_angle ## aka spin speed and direction

    # Escape checking
    Rover.spin_best_angles = 0
    Rover.spin_cnt = 0

    return Rover

# We are spinning looking for a better path
def decision_mode_spin(Rover):
    # print("MODE decision_mode_spin()")

    # Spinning is controlled by the drive_rover() function
    # But here is where decided when to stop and what to do next.

    spin_exit_velocity = min(1.0, Rover.max_vel * 0.70)

    if Rover.near_sample:
        # Stop and pick up rock
        return decision_set_stop(Rover);

    if Rover.saw_rock:
        if abs(Rover.rock_angle) <= 10:
            # stop spinning and go get it
            return decision_set_stop(Rover);

        # Are we spinning the right way?  We might have just
        # seen the rock as we were spinning and then passed it.
        # Or it might have shown up for a moment on th right, when
        # we are spinnign left, etc.  So stop and turn back towards
        # it now that we have "rock memory" to remember where we saw it.
        if Rover.rock_angle < 0 and Rover.target_angle > 0 or \
                Rover.rock_angle > 0 and Rover.target_angle < 0:
            # We are spinning the wrong way, stop and reverse direction.
            # The stop code will make us spin the right way.
            # We could just reverse direction but looping through
            # the stop code allows for more advanced behavior in theory.
            return decision_set_stop(Rover);
    elif Rover.safe_vel > spin_exit_velocity:
        # Good sold path forward -- take it.  Stop spinning and drive.
        return decision_set_stop(Rover);

    # Escape code!

    # Track the quality of optional escape paths forward as we spin.

    Rover.spin_best_angles = max(Rover.spin_best_angles, len(Rover.nav_angles))
    Rover.spin_best_angles += (0 - Rover.spin_best_angles) * 0.01 # slow decay to forget what we saw
    Rover.spin_cnt += 1

    # print("spin cnt", Rover.spin_cnt, "angles", len(Rover.nav_angles), "best angles", Rover.spin_best_angles)
    
    if Rover.spin_cnt > 200:
       # We have to escape this spinning, I'm getting sick
       if Rover.spin_best_angles > 0: # we have seen something worth trying
           five_percent = Rover.spin_best_angles * 0.05
           if len(Rover.nav_angles) > Rover.spin_best_angles - five_percent:
               # Close enough, lets boogie
               # Must force an exit attempt using special stuck mode
               return decision_set_stuck(Rover, forward=True)
       else:
           # crap, no possible paths forward seen at all, we are stuck
           return decision_set_stuck(Rover)

    if Rover.spin_cnt > 400:
        # Oh my, we really are stuck, just bale
        return decision_set_stuck(Rover)
   
    return Rover

#
# All the decision logic to drive the rover starts here
#

def decision_step(Rover):
    start_mode = Rover.mode

    r = decision_work(Rover)    # Decide what to do
    r = drive_rover(r)          # do it

    print("         MODE {:7s}  ".format(start_mode), end='')
    print(" t:{:4.1f}   b:{:4.1f}   s:{:6.2f}".format(Rover.throttle, Rover.brake, Rover.steer), end='')
    print("   tv: {:5.2f}  real vel: {:5.2f}".format(Rover.target_vel, Rover.vel), end='')

    if Rover.mode != start_mode: # we decided to change modes
        print(" -> ", Rover.mode, end='')

    print()

    return r

def decision_work(Rover):

    Rover.update_rock() ## -- will update rock position if needed

    if Rover.mode == 'forward': 
        return decision_mode_forward(Rover)

    if Rover.mode == 'stop':
        return decision_mode_stop(Rover)

    if Rover.mode == 'spin':
        return decision_mode_spin(Rover)

    if Rover.mode == 'stuck':
        return decision_mode_stuck(Rover)

    # No mode set?
    # Shoudn't happen, but just in case it doess..

    return decision_set_stop(Rover);

    return Rover

