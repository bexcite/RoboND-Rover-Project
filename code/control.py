import numpy as np
import math

def distance(pos1, pos2):
  return np.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos2[1]) ** 2)

def normalize_angle(a):
  while a > 180.0:
    a -= 360.0
  while a < - 180:
    a += 360
  return a

def send_stop(Rover):
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    # Rover.mode = 'stop'
    print("CMD: send_stop")

def get_target_vel(Rover):
    targetDist = np.sqrt((Rover.targetPos[0] - Rover.pos[0]) ** 2 + (Rover.targetPos[1] - Rover.pos[1]) ** 2)
    print('TARGET_DIST = ', targetDist)

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
        elif targetDist > 0.5:
            targetVel = Rover.max_vel / 4.0
        else:
            targetVel = 0.0

    return targetVel

def control_step(Rover):

    # Make throttle, steer, brake
    tP = 2.5
    tD = 1.5
    tI = 0.0

    # sP = 0.3
    # sD = 0.12
    # sI = 0.0

    sP = 0.1
    sD = 0.05
    sI = 0.0

    if Rover.targetPos is None:

      if Rover.mode == 'rotate':
        if Rover.vel > 0.2:
          Rover.brake = Rover.brake_set
          print("BRAKE!!!!!!!! on rotate")
        else:
          print('CONTROL = rotate')
          Rover.brake = 0
          Rover.steer = -10
        return Rover


      send_stop(Rover)
      return Rover

    print('targetPos = ', Rover.targetPos)

    # ALways release brakes if they were set
    Rover.brake = 0

    # targetDist = Rover.targetPos[1]




    targetYaw = np.arctan2((Rover.targetPos[1] - Rover.pos[1]), (Rover.targetPos[0] - Rover.pos[0]))
    # print('targetYaw deg = ', targetYaw)
    targetYaw = targetYaw * 180 / math.pi
    print('targetYaw in = ', targetYaw)

    if len(Rover.rock_angles) > 10:
      # targetYaw = np.mean(Rover.rock_angles * 180 / np.pi) + Rover.yaw
      deltaYaw = np.mean(Rover.rock_angles * 180 / np.pi)
      targetYaw = normalize_angle(deltaYaw + Rover.yaw)
      print('targetYaw to rock = ', deltaYaw + Rover.yaw)
      # print('deltaYaw to rock = ', deltaYaw)


    # Smooth with prev
    targetYaw = normalize_angle(targetYaw * 0.1 + Rover.prev_target_yaw * 0.9)
    Rover.prev_target_yaw = targetYaw

    print('targetYaw smooth = ', targetYaw)

    # ===== Control to the yaw

    # deltaYaw = (Rover.targetYaw - Rover.yaw)
    deltaYaw = normalize_angle(targetYaw - Rover.yaw)

    '''
    if len(Rover.rock_angles) > 20:
      # targetYaw = np.mean(Rover.rock_angles * 180 / np.pi) + Rover.yaw
      deltaYaw = np.mean(Rover.rock_angles * 180 / np.pi)
      print('targetYaw to rock = ', deltaYaw + Rover.yaw)
      print('deltaYaw to rock = ', deltaYaw)

    while deltaYaw > 180.0:
      deltaYaw -= 360.0
    while deltaYaw < - 180:
      deltaYaw += 360
    '''

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

    k = 3
    if distance(Rover.pos, Rover.targetPos) < 3:
      k = 0.5

    coeff = np.exp(-(s_cte**2/(2*k*k)))
    print('thrott_coeff = ', coeff)

    '''
    targetDist = np.sqrt((Rover.targetPos[0] - Rover.pos[0]) ** 2 + (Rover.targetPos[1] - Rover.pos[1]) ** 2)
    targetVel = 0
    if targetDist > 15 + 2.5 * Rover.vel:
      targetVel = Rover.max_vel
    elif targetDist > 7 + 3.5 * Rover.vel:
      targetVel = Rover.max_vel / 2.0
    elif targetDist > 2:
      targetVel = Rover.max_vel / 4.0
    else:
      targetVel = 0.0
      coeff = 1.0
    '''

    targetVel = get_target_vel(Rover)

    print('targetVel = ', targetVel, ', mode = ', Rover.mode)

    t_cte = targetVel - Rover.vel
    tDpart = (t_cte - Rover.t_cte_prev) / Rover.dt
    Rover.t_cte_sum += t_cte * Rover.dt

    if targetVel == 0.0:
      coeff = 1.0

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
    if Rover.throttle < 0.1 and Rover.throttle > -0.1:
      Rover.throttle = 0.0

    print('THROTTLE = ', Rover.throttle)

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        print("D: PICK SAMPLE, near_sample = ", Rover.near_sample)
        Rover.send_pickup = True


    return Rover
