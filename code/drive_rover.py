import argparse
import shutil
import base64
from datetime import datetime
import os
import cv2
import numpy as np
import socketio
import eventlet
import eventlet.wsgi
from PIL import Image
from flask import Flask
from io import BytesIO, StringIO
import json
import pickle
import matplotlib.image as mpimg
import time
import sys

# Import functions for perception and decision making
from perception import perception_step
from decision import decision_step
from supporting_functions import update_rover, create_output_images
# Initialize socketio server and Flask application 
# (learn more at: https://python-socketio.readthedocs.io/en/latest/)
sio = socketio.Server()
app = Flask(__name__)

# Read in ground truth map and create 3-channel green version for overplotting
# NOTE: images are read in by default with the origin (0, 0) in the upper left
# and y-axis increasing downward.
ground_truth = mpimg.imread('../calibration_images/map_bw.png')
# This next line creates arrays of zeros in the red and blue channels
# and puts the map into the green channel.  This is why the underlying 
# map output looks green in the display image
ground_truth_3d = np.dstack((ground_truth*0, ground_truth*255, ground_truth*0)).astype(np.float)

# Define RoverState() class to retain rover state parameters
class RoverState():
    def __init__(self):
        self.start_time = None # To record the start time of navigation
        self.total_time = None # To record total duration of naviagation
        self.fps = 0
        self.img = None # Current camera image
        self.pos = None # Current position (x, y)
        self.yaw = None # Current yaw angle
        self.pitch = None # Current pitch angle
        self.roll = None # Current roll angle
        self.vel = None # Current velocity
        self.steer = 0 # Current steering angle
        self.throttle = 0 # Current throttle value
        self.brake = 0 # Current brake value
        self.picking_up = 0 # Will be set to telemetry value data["picking_up"]
        self.send_pickup = False # Set to True to trigger rock pickup

        self.ground_truth = ground_truth_3d # Ground truth worldmap

        #
        # Perception
        #
        
        self.nav_vecgtors = None        # list of vectors -- borken into angles and dists below
        self.nav_angles = None          # Angles of navigable terrain pixels
        self.nav_mean = None            # mean of nav_angles
        self.nav_dists = None           # Distances of navigable terrain pixels

        # Perception recomendations for vel and angle (output to decision code)
        # Must have max_vel less than the max of the simulator so
        # the over can go over the max and we learn to throttle back.
        # Simulator max is 5.0, so we set our target speed to 4.0

        self.max_vel = 4.0 # Maximum velocity (meters/second) that perception will recommend
        self.safe_vel = 0 # calculcated safe driving speed (0 when no path forward)
        self.safe_angle = 0 # percpetion 

        self.last_vel = None            # Tracking acceleration to detect collisions
        self.last_vel_time = None
        self.min_a_brake = 0.0
        self.min_a_nobrake = 0.0
        self.collision = 0              # Display OUCH

        # Rock tracking variables
        self.see_rock = False
        self.rock_pixels = 0 # Number of "rock" pixels seen in image
        self.rock_angle = 0
        self.rock_dist = 0
        self.saw_rock = False
        self.saw_time = time.time()
        self.rock_xpix = 0 # Rover centric coords
        self.rock_ypix = 0
        self.rock_xpix_world = 0
        self.rock_ypix_world = 0

        # Seen map is my private world map to track what "looks" to be good places to drive.

        self.seen_map = np.zeros((200, 200), dtype=np.float) # Tracks grid visits

        # Visit map the grids the rover has acutally driven on.  It's the ground truth
        # version of our belifs about where we can drive.

        self.visit_map = np.zeros((200, 200), dtype=np.float) # Tracks grid visits

        # Stuck map is a count of the times stuck at different grid locations.  We
        # Avoid these.

        self.stuck_map = np.zeros((200, 200), dtype=np.float) # Tracks grid places to avoid

        self.min_visit_x = 100 # experimental but not used currently 
        self.min_visit_y = 100

        #
        # Decision control
        #

        self.mode = 'stop' # Current mode (can be forward or stop or spin or stuck)

        self.spin_best_angles = 0.0
        self.spin_cnt = 0

        self.stuck_cnt = 0
        self.stuck_end = 0

        self.forward_stuck_cnt = 0

        self.throttle_max = 1.0 # max throttle setting
        self.brake_max = 10 # max brake setting when braking

        # speed and angle that control motion and steering
        self.target_vel = 0 # target speed for rover
        self.target_angle = 0 # target steering angle

        self.throttle_target = 0.0 # Negative means break
        self.throttle_current = 0.0 # (what we last set break/throttle to)

        self.throttle_PID_err = 0.0 # last error value

        self.throttle_PID_P = 0.5 
        self.throttle_PID_I = 0.0001
        self.throttle_PID_sum = 3000.0 # starting sum for the I term.
        self.throttle_PID_D = 1.0 # positive dampens change (to prevent oscillation), negative speeds up converging

        # throttle_current is like acceleration.  Postive means we set the throttle
        # to this value, and the break is off.  Negative means throttle is off, and
        # brake is on.  0 means throttle and break is off. (both zero)

        self.picked_up = False          # State to help count rocks
        self.rock_cnt = 0               # Number picked up this run
        self.fakesmell = False          # For Testing invisa-rock code

        # Image output from perception step
        # Update this image to display your intermediate analysis steps
        # on screen in autonomous mode
        self.vision_image = np.zeros((160, 320, 3), dtype=np.float) 

        # Worldmap
        # Update this image with the positions of navigable terrain
        # obstacles and rock samples
        self.worldmap = np.zeros((200, 200, 3), dtype=np.float) 
        self.samples_pos = None # To store the actual sample positions
        self.samples_to_find = 0 # To store the initial count of samples
        self.samples_found = 0 # To count the number of samples found
        self.near_sample = 0 # Will be set to telemetry value data["near_sample"]

    def update_rock(self):
        # Update memory of the rock we saw but no longer see.
        # If we are in "saw_rock" mode, update the rock angle and dist
        # from the saved world xpix and ypix given the current rover position
        # and yaw. (rock_xpix_world and rock ypix_world)
        # Also upate xpic and ypic (rover relative location of rock memory)
        # rock_dist is in relative pixels 10 per m
        # rock_angle is degrees +- 10
        # Rover.yaw is degrees 0 to 360

        if self.saw_rock and not self.see_rock:
           self.rock_dist = np.sqrt((self.pos[0] - self.rock_xpix_world)**2 +
                                (self.pos[1] - self.rock_ypix_world)**2) * 10.0 
           self.rock_angle = np.arctan2(self.rock_ypix_world - self.pos[1],
                                          self.rock_xpix_world - self.pos[0])
           self.rock_angle *= 180.0 / np.pi # rad to degrees
           # yaw is 0 to 360
           # we need rock_angle to be +- 180
           self.rock_angle = self.rock_angle - self.yaw
           if self.rock_angle > 180.0:
                self.rock_angle -= 360.0
           if self.rock_angle < -180.0:
                self.rock_angle += 360.0
           self.rock_ypix = self.rock_dist * np.sin(self.rock_angle * np.pi / 180)
           self.rock_xpix = self.rock_dist * np.cos(self.rock_angle * np.pi / 180)

        return

# Initialize our rover 
Rover = RoverState()

# Variables to track frames per second (FPS)
# Intitialize frame counter
frame_counter = 0
# Initalize second counter
second_counter = time.time()
fps = None


# Define telemetry function for what to do with incoming data
@sio.on('telemetry')
def telemetry(sid, data):

    global frame_counter, second_counter, fps
    frame_counter+=1
    # Do a rough calculation of frames per second (FPS)
    if (time.time() - second_counter) > 1:
        fps = frame_counter
        frame_counter = 0
        second_counter = time.time()
        #print("Current FPS: {}".format(fps))

    if data:
        global Rover
        # Initialize / update Rover with current telemetry
        Rover, image = update_rover(Rover, data)
        Rover.fps = fps # pass alog to perception

        if np.isfinite(Rover.vel):

            # Execute the perception and decision steps to update the Rover's state
            Rover = perception_step(Rover)
            Rover = decision_step(Rover)

            # Create output images to send to server
            out_image_string1, out_image_string2 = create_output_images(Rover)

            # The action step!  Send commands to the rover!
 
            # Don't send both of these, they both trigger the simulator
            # to send back new telemetry so we must only send one
            # back in respose to the current telemetry data.

            # If in a state where want to pickup a rock send pickup command
            if Rover.send_pickup and not Rover.picking_up:
                send_pickup()
                # Reset Rover flags
                Rover.send_pickup = False
            else:
                # Send commands to the rover!
                commands = (Rover.throttle, Rover.brake, Rover.steer)
                send_control(commands, out_image_string1, out_image_string2)

        # In case of invalid telemetry, send null commands
        else:
            # Send zeros for throttle, brake and steer and empty images
            send_control((0, 0, 0), '', '')

        # If you want to save camera images from autonomous driving specify a path
        # Example: $ python drive_rover.py image_folder_path
        # Conditional to save image frame if folder was specified
        if args.image_folder != '':
            timestamp = datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f')[:-3]
            image_filename = os.path.join(args.image_folder, timestamp)
            image.save('{}.jpg'.format(image_filename))

    else:
        sio.emit('manual', data={}, skip_sid=True)

@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    send_control((0, 0, 0), '', '')
    sample_data = {}
    sio.emit(
        "get_samples",
        sample_data,
        skip_sid=True)

def send_control(commands, image_string1, image_string2):
    # Define commands to be sent to the rover
    data={
        'throttle': commands[0].__str__(),
        'brake': commands[1].__str__(),
        'steering_angle': commands[2].__str__(),
        'inset_image1': image_string1,
        'inset_image2': image_string2,
        }
    # Send commands via socketIO server
    sio.emit(
        "data",
        data,
        skip_sid=True)
    eventlet.sleep(0)
# Define a function to send the "pickup" command 
def send_pickup():
    # print("Picking up")
    pickup = {}
    sio.emit(
        "pickup",
        pickup,
        skip_sid=True)
    eventlet.sleep(0)
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Remote Driving')
    parser.add_argument(
        'image_folder',
        type=str,
        nargs='?',
        default='',
        help='Path to image folder. This is where the images from the run will be saved.'
    )
    args = parser.parse_args()
    
    #os.system('rm -rf IMG_stream/*')
    if args.image_folder != '':
        print("Creating image folder at {}".format(args.image_folder))
        if not os.path.exists(args.image_folder):
            os.makedirs(args.image_folder)
        else:
            shutil.rmtree(args.image_folder)
            os.makedirs(args.image_folder)
        print("Recording this run ...")
    else:
        print("NOT recording this run ...")
    
    # wrap Flask application with socketio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
