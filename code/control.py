import numpy as np
import math
import time
from robo_functions import *



def send_stop(Rover):
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    # Rover.mode = 'stop'
    print("CMD: send_stop")

def get_target_vel(Rover, s_cte):
    targetDist = np.sqrt((Rover.targetPos[0] - Rover.pos[0]) ** 2 + (Rover.targetPos[1] - Rover.pos[1]) ** 2)
    print('TARGET_DIST = ', targetDist)

    # if s_cte > 10:
    #   return 0.0

    if Rover.mode == 'forward':
      if targetDist > 1:
        return Rover.max_vel
      else:
        return 0.0

    targetVel = 0
    if Rover.mode == 'forward_stop':
        if targetDist > 15 + 2.5 * Rover.vel:
            targetVel = Rover.max_vel
        elif targetDist > 7 + 3.5 * Rover.vel:
            targetVel = Rover.max_vel / 2.0
        elif targetDist > 0.01: # 0.5
            targetVel = Rover.max_vel / 2.0 # / 4.0
        else:
            targetVel = 0.0

    return targetVel

def set_steer(Rover, deltaYaw):
    sP = 0.5 # 0.5
    sD = 0.35 # 0.35
    sI = 0.0

    deltaYaw = normalize_angle(deltaYaw * 0.3 + Rover.prev_target_yaw * 0.7)
    Rover.prev_target_yaw = deltaYaw

    targetSteer = deltaYaw

    s_cte = targetSteer

    sDpart = (s_cte - Rover.s_cte_prev) / Rover.dt

    Rover.s_cte_sum += s_cte * Rover.dt

    Rover.steer = np.clip(sP * s_cte + sD * sDpart + sI * Rover.s_cte_sum, -15, 15)

    print('s_cte = ', s_cte)
    print('sDpart = ', sDpart)
    print('s_cte_sum = ', Rover.s_cte_sum)
    print('STEER = ', Rover.steer)

    Rover.s_cte_prev = s_cte


def control_step(Rover):

    # Make throttle, steer, brake
    tP = 2.5
    tD = 1.5
    tI = 0.0

    # sP = 0.3
    # sD = 0.12
    # sI = 0.0

    # sP = 0.1
    # sD = 0.05
    # sI = 0.0

    if Rover.mode == 'rotate_left':
      if Rover.vel > 0.2:
        send_stop(Rover)
        return Rover
      Rover.steer = 15
      Rover.brake = 0
      Rover.throttle = 0.0
      print('MODE = rotate_left')
      return Rover

    if Rover.mode == 'rotate_right':
      if Rover.vel > 0.2:
        send_stop(Rover)
        return Rover
      Rover.steer = -15
      Rover.brake = 0
      Rover.throttle = 0.0
      print('MODE = rotate_right')
      return Rover

    if Rover.near_sample and not Rover.picking_up:
      print('ROVER NEAR SAMPLE!')
      if Rover.vel > 0.01:
        send_stop(Rover)
        return Rover
      else:
        Rover.brake = 0
        Rover.send_pickup = True
        return Rover


    if Rover.mode == 'follow_wall':
      if len(Rover.nav_angles) > 200:
        Rover.brake = 0
        deltaYaw = np.mean(Rover.nav_angles * 180 / np.pi)
        deltaYaw_min = np.min(Rover.nav_angles * 180 / np.pi)
        deltaYaw_max = np.max(Rover.nav_angles * 180 / np.pi)
        set_steer(Rover, normalize_angle(deltaYaw * 0.8 + deltaYaw_max * 0.2)) # -15
        # Rover.steer = np.clip(deltaYaw-8, -15, 15)
        Rover.throttle = Rover.throttle_set
        print('MODE = follow_wall')
        return Rover


    print('targetPos = ', Rover.targetPos)

    # ALways release brakes if they were set
    Rover.brake = 0

    # targetDist = Rover.targetPos[1]


    if Rover.mode == 'forward_stop':
      if len(Rover.rock_angles) > 10:
        deltaYaw = np.mean(Rover.rock_angles * 180 / np.pi)
        set_steer(Rover, deltaYaw)
        if np.absolute(deltaYaw) > 10 and Rover.vel > 0.1:
          # Rover.brake = Rover.brake_set
          # Rover.thottle = 0.0
          print('deltaYaw is big and vel is 0.1 plus - so stop.')
          send_stop(Rover)
          return Rover
        # k = 3
        # coeff = np.exp(-(deltaYaw**2/(2*k*k)))
        coeff = 1
        if np.absolute(deltaYaw) < 10:
          Rover.brake = 0.0
          Rover.throttle = coeff * Rover.throttle_set
        return Rover



    '''
    if Rover.targetPos is None:

      if Rover.mode == 'rotate':
        delta_t = time.time() - Rover.rotStartTime
        delta_yaw = np.absolute(Rover.yaw - Rover.rotStartYaw)
        print("MODE = rotate cont")
        print('delta_t = ', delta_t)
        print('delta_yaw = ', delta_yaw)
        if delta_t > 4.0 and delta_yaw < 10:
          # We did about full lap then stop
          Rover.mode = 'no_target'
          print("MODE = no_target")
        elif Rover.vel > 0.2:
          Rover.brake = Rover.brake_set
          print("BRAKE!!!!!!!! on rotate")
        else:
          print('CONTROL = rotate')
          Rover.brake = 0
          Rover.steer = -10
        return Rover


      send_stop(Rover)
      return Rover
    '''



    '''
    targetYaw = np.arctan2((Rover.targetPos[1] - Rover.pos[1]), (Rover.targetPos[0] - Rover.pos[0]))
    # print('targetYaw deg = ', targetYaw)
    targetYaw = targetYaw * 180 / math.pi
    print('targetYaw in = ', targetYaw)

    if len(Rover.rock_angles) > 10 and Rover.mode == 'forward_stop':
      # targetYaw = np.mean(Rover.rock_angles * 180 / np.pi) + Rover.yaw
      deltaYaw = np.mean(Rover.rock_angles * 180 / np.pi)
      targetYaw = normalize_angle(deltaYaw + Rover.yaw)
      print('targetYaw to rock = ', deltaYaw + Rover.yaw)
      # print('deltaYaw to rock = ', deltaYaw)
    elif len(Rover.nav_angles) > 10 and Rover.mode == 'forward':
      deltaYaw = np.mean(Rover.nav_angles * 180 / np.pi)
      targetYawNew = normalize_angle(deltaYaw + Rover.yaw)
      diff = np.absolute(normalize_angle(targetYawNew - targetYaw))
      print('targetYawNew to nav space = ', deltaYaw + Rover.yaw, ', diff =', diff)
      if diff < 45.0:
        targetYaw = targetYawNew
        print('targetYaw to nav space = ', targetYaw, ', diff =', diff)


    # Smooth with prev
    targetYaw = normalize_angle(targetYaw * 0.1 + Rover.prev_target_yaw * 0.9)
    Rover.prev_target_yaw = targetYaw

    print('targetYaw smooth = ', targetYaw)

    # ===== Control to the yaw

    # deltaYaw = (Rover.targetYaw - Rover.yaw)
    deltaYaw = normalize_angle(targetYaw - Rover.yaw)

    # s_cte = deltaYaw
    #
    # set_steer(Rover, deltaYaw)


    print('deltaYaw = ', deltaYaw)

    targetSteer = deltaYaw

    s_cte = targetSteer

    sDpart = (s_cte - Rover.s_cte_prev) / Rover.dt

    Rover.s_cte_sum += s_cte * Rover.dt

    Rover.steer = np.clip(sP * s_cte + sD * sDpart + sI * Rover.s_cte_sum, -15, 15)

    print('s_cte = ', s_cte)
    print('sDpart = ', sDpart)
    print('s_cte_sum = ', Rover.s_cte_sum)
    print('STEER = ', Rover.steer)

    Rover.s_cte_prev = s_cte


    # ====== Throttle control

    if Rover.mode == 'forward_stop' and np.absolute(s_cte) > 10 and Rover.vel > 0.1:
      send_stop(Rover)
      print('BRAKE due to s_cte %f and vel %f params' % (s_cte, Rover.vel))
      return Rover


    k = 10
    if Rover.mode == 'forward_stop':
      k = 3
      if distance(Rover.pos, Rover.targetPos) < 3:
        k = 0.5
    coeff = np.exp(-(s_cte**2/(2*k*k)))


    # coeff = 1.0
    # if s_cte > 10:
    #   coeff = 0.0

    print('thrott_coeff = ', coeff)


    targetVel = get_target_vel(Rover, s_cte)

    print('targetVel = ', targetVel, ', mode = ', Rover.mode)

    t_cte = targetVel - Rover.vel
    tDpart = (t_cte - Rover.t_cte_prev) / Rover.dt
    Rover.t_cte_sum += t_cte * Rover.dt

    # if targetVel == 0.0:
    #   coeff = 1.0

    print('coeff final = ', coeff)
    Rover.throttle = coeff * np.clip(tP * t_cte + tD * tDpart + tI * Rover.t_cte_sum, -0.4, 0.2)


    if targetVel == 0.0 and Rover.vel > 0.1: #  and Rover.vel > 0.1 and s_cte < 3
      Rover.brake = Rover.brake_set
      print("BRAKE!!!!!!!!")
    else:
      Rover.brake = 0



    print('t_cte = ', t_cte)
    print('tDpart = ', tDpart)
    print('t_cte_sum = ', Rover.t_cte_sum)


    Rover.t_cte_prev = t_cte

    # this prevents rover to stale on one place, I think it's due to very small number of throttle
    # and the way simulator process it.
    if Rover.throttle < 0.01 and Rover.throttle > -0.01:
      Rover.throttle = 0.0

    print('THROTTLE = ', Rover.throttle)

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel < 0.1 and not Rover.picking_up:
        print("D: PICK SAMPLE, near_sample = ", Rover.near_sample)
        Rover.send_pickup = True
    '''


    return Rover
