from transformation_helpers import *
import numpy as np

def deg2rad(v):
  return v / 180.0 * n.pi

def get_T_B_LB():
  x_m = 0.035 
  y_m = 0.002 
  z_m = −1.23
  roll_rad = deg2rad(−179.93)
  pitch_rad = deg2rad(−0.23)
  yaw_rad = deg2rad(0.5)

  T_B_LB = getTransformation(roll_rad, pitch_rad, yaw_rad, x_m, y_m, z_m)
  return T_B_LB

def get_T_B_V():
  x_m = 0.002
  y_m = −0.004
  z_m = −0.957
  roll_rad = deg2rad(0.807)
  pitch_rad = deg2rad(0.166)
  yaw_rad = deg2rad(−90.703)

  T_B_V = getTransformation(roll_rad, pitch_rad, yaw_rad, x_m, y_m, z_m)
  return T_B_V

def get_T_B_I():
  x_m = −0.11
  y_m = −0.18
  z_m = −0.71
  roll_rad = 0.0
  pitch_rad = 0.0
  yaw_rad = 0.0

  T_B_I = getTransformation(roll_rad, pitch_rad, yaw_rad, x_m, y_m, z_m)
  return T_B_I

def get_T_B_GPS():
  x_m = 0 
  y_m = −0.25 
  z_m = −0.51
  roll_rad = 0.0
  pitch_rad = 0.0
  yaw_rad = 0.0

  T_B_GPS = getTransformation(roll_rad, pitch_rad, yaw_rad, x_m, y_m, z_m)
  return T_B_GPS

def get_T_B_RTK():
  x_m = −0.24 
  y_m = 0 
  z_m = −1.24
  roll_rad = 0.0
  pitch_rad = 0.0
  yaw_rad = 0.0

  T_B_RTK = getTransformation(roll_rad, pitch_rad, yaw_rad, x_m, y_m, z_m)
  return T_B_RTK

def get_T_LB_C0():
  x_m = 0.000920 
  y_m = −0.000569 
  z_m = 0.062413 
  roll_rad = −0.028132 
  pitch_rad = 0.196467
  yaw_rad = 0.248664

  T_LB_C0 = getTransformation(roll_rad, pitch_rad, yaw_rad, x_m, y_m, z_m)
  return T_LB_C0

def get_T_LB_C1():
  x_m = 0.014543 
  y_m = 0.039337 
  z_m = 0.000398 
  roll_rad = −138.449751 
  pitch_rad = 89.703877 
  yaw_rad = −66.518051

  T_LB_C1 = getTransformation(roll_rad, pitch_rad, yaw_rad, x_m, y_m, z_m)
  return T_LB_C1

def get_T_LB_C2():
  x_m = −0.032674 
  y_m = 0.025928 
  z_m = 0.000176 
  roll_rad = 160.101024 
  pitch_rad = 89.836345 
  yaw_rad = −56.101163

  T_LB_C2 = getTransformation(roll_rad, pitch_rad, yaw_rad, x_m, y_m, z_m)
  return T_LB_C2

def get_T_LB_C3():
  x_m = −0.034969 
  y_m = −0.022993 
  z_m = 0.000030 
  roll_rad = 95.603967 
  pitch_rad = 89.724274 
  yaw_rad = −48.640335

  T_LB_C3 = getTransformation(roll_rad, pitch_rad, yaw_rad, x_m, y_m, z_m)
  return T_LB_C3

def get_T_LB_C4():
  x_m = 0.011238 
  y_m = −0.040367 
  z_m = −0.000393 
  roll_rad = −160.239278 
  pitch_rad = 89.812338 
  yaw_rad = 127.472911

  T_LB_C4 = getTransformation(roll_rad, pitch_rad, yaw_rad, x_m, y_m, z_m)
  return T_LB_C4

def get_T_LB_C5():
  x_m = 0.041862 
  y_m = −0.001905 
  z_m = −0.000212 
  roll_rad = 160.868615 
  pitch_rad = 89.914152 
  yaw_rad = 160.619894

  T_LB_C5 = getTransformation(roll_rad, pitch_rad, yaw_rad, x_m, y_m, z_m)
  return T_LB_C5

# fx, fy, cx, cy
def getC0Intrinsics():
  return np.array([410.777504, 410.777504, 805.879152, 613.277472])

def getC1Intrinsics():
  return np.array([409.719024, 409.719024, 813.947840, 624.237344])

def getC2Intrinsics():
  return np.array([408.385824, 408.385824, 793.959536, 623.058320])

def getC3Intrinsics():
  return np.array([406.802144, 406.802144, 810.874208, 617.352928])

def getC4Intrinsics():
  return np.array([403.920816, 403.920816, 823.270544, 601.929520])

def getC5Intrinsics():
  return np.array([399.433184, 399.433184, 826.361952, 621.668624])


