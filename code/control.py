import numpy as np
import math

def send_stop(Rover):
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    Rover.mode = 'stop'
    print("CMD: send_stop")


def control_step(Rover):

    # Make throttle, steer, brake
    tP = 0.7
    tD = 0.2
    tI = 0.0

    sP = 0.1
    sD = 0.07
    sI = 0.0

    if Rover.targetPos is None:
      send_stop(Rover)
      return Rover


    # targetDist = Rover.targetPos[1]


    targetYaw = np.arctan2((Rover.targetPos[1] - Rover.pos[1]), (Rover.targetPos[0] - Rover.pos[0]))
    targetYaw = targetYaw * 180 / math.pi
    print('targetYaw = ', targetYaw)

    # Control to the yaw

    # deltaYaw = (Rover.targetYaw - Rover.yaw)
    deltaYaw = targetYaw - Rover.yaw
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
    print('steer = ', Rover.steer)

    Rover.s_cte_prev = s_cte

    # ====== Throttle control

    coeff = np.exp(-(s_cte**2/(2*3*3)))
    print('thrott_coeff = ', coeff)


    t_cte = np.sqrt((Rover.targetPos[0] - Rover.pos[0]) ** 2 + (Rover.targetPos[1] - Rover.pos[1]) ** 2)
    t_cte = t_cte - 3
    tDpart = (t_cte - Rover.t_cte_prev) / Rover.dt
    Rover.t_cte_sum += t_cte * Rover.dt

    Rover.throttle = coeff * np.clip(tP * t_cte + tD * tDpart + tI * Rover.t_cte_sum, -0.2, 0.2)

    print('t_cte = ', t_cte)
    print('tDpart = ', tDpart)
    print('t_cte_sum = ', Rover.t_cte_sum)
    print('throttle = ', Rover.throttle)

    Rover.t_cte_prev = t_cte

    if Rover.throttle < 0.1:
      Rover.throttle = 0.0

    return Rover
