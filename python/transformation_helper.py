import minkindr as mk
import numpy as np
import IPython

def getRotationMatrixFromEulerAnglesRollPitchYaw(roll_rad, pitch_rad, yaw_rad):
  R_roll = np.array([[np.cos(roll_rad), -np.sin(roll_rad), 0.0], 
                     [np.sin(roll_rad), np.cos(roll_rad), 0.0], 
                     [0.0, 0.0, 1.0]])

  R_pitch = np.array([[np.cos(pitch_rad), 0.0, np.sin(pitch_rad)], 
                     [0.0, 1.0, 0.0], 
                     [-np.sin(pitch_rad), 0.0, np.cos(pitch_rad)]])

  R_yaw = np.array([[1.0, 0.0, 0.0], 
                     [0.0, np.cos(yaw_rad), -np.sin(yaw_rad)], 
                     [0.0, np.sin(yaw_rad), np.cos(yaw_rad)]])

  R = R_roll * R_pitch * R_yaw
  return R

def getQuaternionFromFromEulerAnglesRollPitchYaw(roll_rad, pitch_rad, yaw_rad):
  R = getRotationMatrixFromEulerAnglesRollPitchYaw(roll_rad, pitch_rad, yaw_rad)
  q = mk.createQuaternionFromApproximateRotationMatrix(R)

  return q

def getTransformation(roll_rad, pitch_rad, yaw_rad, x_m, y_m, z_m):
  q = getQuaternionFromFromEulerAnglesRollPitchYaw(roll_rad, pitch_rad, yaw_rad)
  p = np.array([x_m, y_m, z_m])
  T = mk.Transformation(q, p)

  return T
