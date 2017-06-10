import numpy as np
import time

from scipy.ndimage.measurements import label

def get_rocks_on_map(worldmap):
    rocks_map = worldmap[:, :, 1]
    labels = label(rocks_map)
    print('labels[num] = ', labels[1])

    rocks = []
    positions = []
    for rock in range(1, labels[1] + 1):
      ypos, xpos = (labels[0] == rock).nonzero()
      rock_pxs = rocks_map[ypos,xpos]
      # print("rock_pxs = ", rock_pxs)
      rock_pxs /= np.max(rock_pxs)
      xpos_m = np.sum(np.multiply(xpos, rock_pxs))/np.sum(rock_pxs)
      ypos_m = np.sum(np.multiply(ypos, rock_pxs))/np.sum(rock_pxs)
      # print('ypos = ', ypos)
      # print('xpos = ', xpos)
      # print('xpos_m = ', xpos_m)
      # print('ypos_m = ', ypos_m)
      # print("rock_pxs_norm = ", rock_pxs)
      rocks.append((xpos_m, ypos_m))
      positions.append((xpos, ypos))
    return rocks, positions

def get_closest_rock_idx(rocks, rover_pos, limit = 200):
  if len(rocks) == 0:
    return -1
  dist_min = 200
  idx_min = -1
  for idx, r in enumerate(rocks):
    d = np.sqrt((rover_pos[0] - r[0]) ** 2 + (rover_pos[1] - r[1]) ** 2)
    if d < dist_min and d < limit:
      dist_min = d
      idx_min = idx
    print(idx, ' : ', r)
  return idx_min


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # steer = np.mean(Rover.nav_angles * 180/np.pi)
    # dists = np.mean(Rover.nav_dists)
    # print('mean steer = ', steer)
    # print('mean dists = ', dists)

    rocks, positions = get_rocks_on_map(Rover.worldmap)

    # Clean picked up rock from the map
    if Rover.picking_up:
      # Get Closest rock in dist < 4
      # Clean all pixels from the map for this rock
      # Rover.worldmap[y-5:y+5, x-5:x+5, 1] = 0
      if len(rocks) > 0:
        idx_min = get_closest_rock_idx(rocks, Rover.pos, limit = 16)
        if idx_min >= 0:
          Rover.worldmap[positions[idx_min][1], positions[idx_min][0], 1] = 0
      return Rover


    if len(Rover.rock_angles) > 10:
      print('ROCK VISIBLE! set targetPos to ', Rover.rock_pos)
      Rover.targetPos = Rover.rock_pos
      Rover.mode = 'forward_stop'
      return Rover




    # TODO: Select closest rock as target
    if len(rocks) > 0:
      idx_min = get_closest_rock_idx(rocks, Rover.pos)
      print('ROCK ON MAP! set targetPos to ', rocks[idx_min])
      Rover.targetPos = rocks[idx_min]
      Rover.mode = 'forward_stop'
      return Rover

    # Do nothing and just explore
    print('MODE before rotate check = ', Rover.mode)
    Rover.targetPos = None
    if Rover.mode != 'rotate' and Rover.mode != 'no_target':
      Rover.mode = 'rotate'
      print("MODE = rotate")
      Rover.rotStartYaw = Rover.yaw
      Rover.rotStartTime = time.time()
    elif Rover.mode == 'rotate':
      delta_t = time.time() - Rover.rotStartTime
      delta_yaw = np.absolute(Rover.yaw - Rover.rotStartYaw)
      print("MODE = rotate cont")
      print('delta_t = ', delta_t)
      print('delta_yaw = ', delta_yaw)
      if delta_t > 1.0 and delta_yaw < 10:
        # We did about full lap then stop
        Rover.mode = 'no_target'
        print("MODE = no_target")




    # Plan the route and select the next point


    # targetX = 93.3 # Rover.pos[0] + 10
    # targetY = 78.4 # Rover.pos[1] + 10

    # targetX = 97
    # targetY = 73


    # targetX = np.random.choice([107, 106])
    # targetY = 66

    # targetYaw = 90

    # print('deltaYaw = ', deltaYaw)

    # Rover.targetYaw = targetYaw
    # Rover.targetPos = (targetX, targetY)
    # Rover.mode = 'forward_stop'
    # Rover.mode = 'forward_stop'
    # Rover.mode = 'forward_pickup'

    return Rover

'''
    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':

            # If if stuck somewhere - moving forward without getting speed
            if Rover.histAvgSpeedErr == 0 and Rover.histAvgSpeed < 0.1:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
                Rover.stuck = True
                print("D: STOP - stuck")
                return Rover

            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                print("D: FORWARD - cont")
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
                    print("D: STOP - not good nav")

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                print("D: STOP - continue stop")
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward or Rover.stuck:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                    Rover.stuck = False
                    print("D: STOP/TURN - looking around")
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
                    print("D: FORWARD - return to forward")
    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        print("D: DEFAULT")
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        print("D: PICK SAMPLE")
        Rover.send_pickup = True

    return Rover
'''
