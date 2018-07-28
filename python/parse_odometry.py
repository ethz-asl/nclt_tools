import os
import IPython
import numpy as np
import minkindr as mk

from transformation_helpers import *

def parse_synchronized_odometry(odometry_file, odometry_cov_file):
  assert(os.path.exists(odometry_file))
  assert(os.path.exists(odometry_cov_file))

  odom = np.loadtxt(odometry_file, delimiter = ",")
  odom_cov = np.loadtxt(odometry_cov_file, delimiter = ",")

  num_measurements = len(odom)
  assert(num_measurements == len(odom_cov))

  T_O_Bkm1 = mk.Transformation()

  time_us_T_O_Bks_with_covs = []

  print 'Parsing odometry...'
  for i in range(num_measurements):
    utime = odom[i, 0]
    x_m = odom[i, 1]
    y_m = odom[i, 2]
    z_m = odom[i, 3]

    roll_rad = odom[i, 4]
    pitch_rad = odom[i, 5]
    yaw_rad = odom[i, 6]

    T_Bkm1_Bk = getTransformation(roll_rad, pitch_rad, yaw_rad, x_m, y_m, z_m)
    T_O_Bk = T_O_Bkm1 * T_Bkm1_Bk

    cov = np.array([
      [odom_cov[i, 1], odom_cov[i, 2],  odom_cov[i, 3],  odom_cov[i, 4],  odom_cov[i, 5], odom_cov[i, 6]], 
      [odom_cov[i, 2], odom_cov[i, 7],  odom_cov[i, 8],  odom_cov[i, 9],  odom_cov[i, 10], odom_cov[i, 11]], 
      [odom_cov[i, 3], odom_cov[i, 8],  odom_cov[i, 12], odom_cov[i, 13], odom_cov[i, 14], odom_cov[i, 15]], 
      [odom_cov[i, 4], odom_cov[i, 9],  odom_cov[i, 13], odom_cov[i, 16], odom_cov[i, 17], odom_cov[i, 18]], 
      [odom_cov[i, 5], odom_cov[i, 10], odom_cov[i, 14], odom_cov[i, 17], odom_cov[i, 19], odom_cov[i, 20]], 
      [odom_cov[i, 6], odom_cov[i, 11], odom_cov[i, 15], odom_cov[i, 17], odom_cov[i, 20], odom_cov[i, 21]]])

    time_us_T_O_Bks_with_covs.append((utime, T_O_Bk, cov))

  print 'done'
  return time_us_T_O_Bks_with_covs
