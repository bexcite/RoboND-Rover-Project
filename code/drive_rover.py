# Do the necessary imports
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

# Import functions for perception and decision making
from perception import perception_step
from decision import decision_step
from control import control_step
from planning import planning_step
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

class History():
  def __init__(self, time_length = 2.0):
    self.hist = []
    self.time_len = time_length
  def add(self, dt, vel, throttle, mode):
    if dt > 1.0 or vel is None:
      return
    for h in self.hist:
      h["dt"] += dt
    while (len(self.hist) > 0 and self.hist[0]["dt"] > self.time_len ):
      del self.hist[0]
    self.hist.append({"dt":dt, "vel":vel, "throttle":throttle, "mode":mode})
  def avgSpeed(self, mode=None):
    if len(self.hist) == 0: return 0.0, 1
    if len(self.hist) > 0 and self.hist[0]["dt"] < self.time_len/2:
      return 0.0, 1
    # if len(self.hist) == 1:
    #   if mode is not None:
    #     if self.hist[0]["mode"] == mode:
    #       return self.hist[0]["vel"], 0
    #     else:
    #       return 0.0, 1
    #   else:
    #     return self.hist[0]["vel"], 0
    d = 0.0
    t = 0.0
    for idx in range(len(self.hist)-1):
      if mode is not None:
        if self.hist[idx]["mode"] == mode and self.hist[idx+1]["mode"] == mode:
          # print('vel = ', self.hist[idx]["vel"])
          # print('ddt = ', (self.hist[idx]["dt"] - self.hist[idx+1]["dt"]))
          d += self.hist[idx]["vel"] * (self.hist[idx]["dt"] - self.hist[idx+1]["dt"])
        else:
          return 0.0, 1
      else:
        d += self.hist[idx]["vel"] * (self.hist[idx]["dt"] - self.hist[idx+1]["dt"])
      t += self.hist[idx]["throttle"]
    print('d = ', d)
    print('avg_throttle = ', t / len(self.hist))
    if t / len(self.hist) > 0.1:
      return d / (self.hist[0]["dt"] - self.hist[len(self.hist)-1]["dt"]), 0
    else:
      return d / (self.hist[0]["dt"] - self.hist[len(self.hist)-1]["dt"]), 1
history = History(2.0)

# Define RoverState() class to retain rover state parameters
class RoverState():
    def __init__(self):
        self.start_time = None # To record the start time of navigation
        self.total_time = None # To record total duration of naviagation
        self.img = None # Current camera image
        self.pos = None # Current position (x, y)
        self.yaw = None # Current yaw angle
        self.pitch = None # Current pitch angle
        self.roll = None # Current roll angle
        self.vel = None # Current velocity
        self.steer = 0 # Current steering angle
        self.throttle = 0 # Current throttle value
        self.brake = 0 # Current brake value
        self.nav_angles = None # Angles of navigable terrain pixels
        self.nav_dists = None # Distances of navigable terrain pixels
        self.ground_truth = ground_truth_3d # Ground truth worldmap
        self.mode = 'start' # Current mode (can be forward or stop)
        self.throttle_set = 0.2 # Throttle setting when accelerating
        self.brake_set = 10 # Brake setting when braking
        # The stop_forward and go_forward fields below represent total count
        # of navigable terrain pixels.  This is a very crude form of knowing
        # when you can keep going and when you should stop.  Feel free to
        # get creative in adding new fields or modifying these!
        self.stop_forward = 50 # Threshold to initiate stopping
        self.go_forward = 500 # Threshold to go forward again
        self.max_vel = 2 # Maximum velocity (meters/second)
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
        self.picking_up = 0 # Will be set to telemetry value data["picking_up"]
        self.send_pickup = False # Set to True to trigger rock pickup

        self.view_mask = None
        self.nav_mask = None

        self.rock_angles = []
        self.rock_dists = []
        self.rock_pos = []
        self.rock_ttl = 0

        self.histAvgSpeed = 0.0
        self.histAvgSpeedErr = 1

        self.stuck = False

        self.targetPos = None
        self.targetYaw = None
        self.dt = 0.0

        # Rotate mode
        self.rotStartYaw = 0.0
        self.rotStartTime = 0.0

        self.prev_target_yaw = 0

        self.s_cte_prev = 0.0
        self.s_cte_sum = 0.0

        self.t_cte_prev = 0.0
        self.t_cte_sum = 0.0

        self.initialize = False
# Initialize our rover
Rover = RoverState()

# Variables to track frames per second (FPS)
# Intitialize frame counter
frame_counter = 0
# Initalize second counter
second_counter = time.time()
prev_counter = 0
fps = None


# Define telemetry function for what to do with incoming data
@sio.on('telemetry')
def telemetry(sid, data):

    global frame_counter, second_counter, fps, prev_counter, history, Rover

    dt = time.time() - prev_counter
    print("DT = ", dt, ", VEL = ", Rover.vel)
    Rover.dt = dt
    prev_counter = time.time()

    # Initialize first time
    if not Rover.initialize:
      Rover.initialize = True
      # Send zeros for throttle, brake and steer and empty images
      send_control((0, 0, 0), '', '')
      return


    # print("hist = ", history.hist)
    avSpeed, avS_err = history.avgSpeed() # mode='forward'
    print("avgSpeed = ", avSpeed, ", err = ", avS_err)
    Rover.histAvgSpeed = avSpeed
    Rover.histAvgSpeedErr = avS_err

    frame_counter+=1
    # Do a rough calculation of frames per second (FPS)
    if (time.time() - second_counter) > 1:
        fps = frame_counter
        frame_counter = 0
        second_counter = time.time()
    print("Current FPS: {}".format(fps))

    if data:
        # global Rover
        # Initialize / update Rover with current telemetry
        Rover, image = update_rover(Rover, data)

        if np.isfinite(Rover.vel):

            # Execute the perception and decision steps to update the Rover's state

            Rover = perception_step(Rover)


            Rover = decision_step(Rover)

            Rover = planning_step(Rover)


            Rover = control_step(Rover)

            # Fix thottle (bug)
            if Rover.throttle < 0.01 and Rover.throttle > -0.01:
              Rover.throttle = 0.0


            history.add(dt, Rover.vel, Rover.throttle, Rover.mode)


            # Create output images to send to server
            out_image_string1, out_image_string2 = create_output_images(Rover)

            # The action step!  Send commands to the rover!
            commands = (Rover.throttle, Rover.brake, Rover.steer)
            send_control(commands, out_image_string1, out_image_string2)

            # If in a state where want to pickup a rock send pickup command
            if Rover.send_pickup and not Rover.picking_up:
                send_pickup()
                # Reset Rover flags
                Rover.send_pickup = False
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
    print("Picking up")
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
