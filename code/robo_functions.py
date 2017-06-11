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

def pos_int(pos):
  return (int(pos[0]), int(pos[1]))

def direction_to_pos(rover_pos, rover_yaw, pos):
  targetYaw = np.arctan2((pos[1] - rover_pos[1]), (pos[0] - rover_pos[0]))
  targetYaw = targetYaw * 180 / math.pi
  deltaYaw = normalize_angle(targetYaw - rover_yaw)
  return deltaYaw
