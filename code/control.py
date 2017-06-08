import numpy as np
import math

def send_stop(Rover):
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    Rover.mode = 'stop'
    print("CMD: send_stop")

def get_target_vel(Rover):
    targetDist = np.sqrt((Rover.targetPos[0] - Rover.pos[0]) ** 2 + (Rover.targetPos[1] - Rover.pos[1]) ** 2)
    print('TARGET_DIST = ', targetDist)

    if Rover.mode == 'forward':
      if targetDist > 3:
        return Rover.max_vel
      else:
        return 0.0

    targetVel = 0
    if Rover.mode == 'forward_stop':
        if targetDist > 15 + 2.5 * Rover.vel:
            targetVel = Rover.max_vel
        elif targetDist > 7 + 3.5 * Rover.vel:
            targetVel = Rover.max_vel / 2.0
        elif targetDist > 1:
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
      send_stop(Rover)
      return Rover

    print('targetPos = ', Rover.targetPos)

    # targetDist = Rover.targetPos[1]




    targetYaw = np.arctan2((Rover.targetPos[1] - Rover.pos[1]), (Rover.targetPos[0] - Rover.pos[0]))
    print('targetYaw deg = ', targetYaw)
    targetYaw = targetYaw * 180 / math.pi
    print('targetYaw = ', targetYaw)



    # ===== Control to the yaw

    # deltaYaw = (Rover.targetYaw - Rover.yaw)
    deltaYaw = targetYaw - Rover.yaw


    if Rover.rock_pos is not None:
      # targetYaw = np.mean(Rover.rock_angles * 180 / np.pi) + Rover.yaw
      deltaYaw = np.mean(Rover.rock_angles * 180 / np.pi)
      print('targetYaw to rock = ', deltaYaw + Rover.yaw)
      print('deltaYaw to rock = ', deltaYaw)

    while deltaYaw > 180.0:
      deltaYaw -= 360.0
    while deltaYaw < - 180:
      deltaYaw += 360

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


    coeff = np.exp(-(s_cte**2/(2*3*3)))
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

    '''
    if targetVel == 0.0 and s_cte < 3: #  and Rover.vel > 0.1
      Rover.brake = Rover.brake_set
      print("BRAKE!!!!!!!!")
    else:
      Rover.brake = 0
    '''


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
