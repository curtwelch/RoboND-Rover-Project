import numpy as np

def decision_set_stop(Rover):
    # Set mode to "stop" and hit the brakes!
    Rover.mode = 'stop'
    Rover.throttle = 0
    # Set brake to stored brake value
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    Rover.target_vel = 0
    return Rover

def decision_forward(Rover):
    # Rover.mode is 'forward': 

    # Check the extent of navigable terrain
    if len(Rover.nav_angles) < Rover.stop_forward:
        return decision_set_stop(Rover)
            
    # Set target velocity
    min_dists = np.min(Rover.nav_dists)
    max_dists = np.max(Rover.nav_dists)
    mean_dists = np.mean(Rover.nav_dists)
    # print("distances, cnt={}, min={}, max={}, mean={}".format(len(Rover.nav_dists), min_dists, max_dists, mean_dists))

    mean_dists = np.clip(mean_dists-10, 0.0, 40.0)
    Rover.target_vel = Rover.max_vel*2.0 * mean_dists / 40.0
    Rover.target_vel = max(Rover.min_vel, Rover.target_vel)

    print("Target_vel ", Rover.target_vel, "mean", mean_dists)

    # Regulate speed to Rover.target_vel

    if Rover.vel < Rover.target_vel:
        # Throttle UP
        Rover.throttle = Rover.throttle_set * 2
        Rover.brake = 0
    elif Rover.vel > 2*Rover.target_vel:
        # Throttle off, break
        Rover.throttle = 0
        Rover.brake = 1.0 # light break
    else: # coast
        Rover.throttle = 0
        Rover.brake = 0

    # Set steering to average angle clipped to the range +/- 15
    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)

    return Rover

def decision_stop(Rover):
    # If we're in stop mode but still moving keep braking
    if Rover.vel > 0.2:
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        Rover.steer = 0
        return Rover

    # else we're not moving (vel < 0.2) then do something else

    # Now we're stopped and we have vision data to see if there's a path forward
    if len(Rover.nav_angles) < Rover.go_forward:
        # No path forward -- spin
        Rover.throttle = 0
        # Release the brake to allow turning
        Rover.brake = 0
        # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
        Rover.steer = -15 # Could be more clever here about which way to turn
        return Rover

    # If we're stopped but see sufficient navigable terrain in front then go!
    if len(Rover.nav_angles) >= Rover.go_forward:
        Rover.mode = 'forward'
        return decision_foward()

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is None:
        # Just to make the rover do something 
        # even if no modifications have been made to the code
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        return Rover

    if Rover.mode == 'forward': 
        return decision_forward(Rover)

    if Rover.mode == 'stop':
        return decision_stop(Rover)

    return Rover

