import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    steer = np.mean(Rover.nav_angles * 180/np.pi)
    dists = np.mean(Rover.nav_dists)
    print('mean steer = ', steer)
    print('mean dists = ', dists)


    if Rover.rock_pos is not None:
      Rover.targetPos = Rover.rock_pos
      Rover.mode = 'forward_stop'
      return Rover

    '''
    # Look at map and select the best way point
    # 1. Is there diamonds on map?
    print('Look for rocks')
    ypos, xpos = Rover.worldmap[:, :, 1].nonzero()
    if len(ypos) > 0:
      print('ROCKS!!!! Num pix = ', len(ypos))
      print('ypos =', ypos)
      print('xpos =', xpos)
      Rover.targetPos = (xpos[0], ypos[0])
      Rover.mode = 'forward_stop'
      return Rover
    else:
      print('NO ROCKS')
    '''


    # Plan the route and select the next point


    # targetX = 93.3 # Rover.pos[0] + 10
    # targetY = 78.4 # Rover.pos[1] + 10

    targetX = 97
    targetY = 73


    # targetX = np.random.choice([107, 106])
    # targetY = 66

    targetYaw = 90

    # print('deltaYaw = ', deltaYaw)

    Rover.targetYaw = targetYaw
    Rover.targetPos = (targetX, targetY)
    # Rover.mode = 'forward_stop'
    Rover.mode = 'forward_stop'
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
