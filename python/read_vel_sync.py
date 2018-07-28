# !/usr/bin/python
#
# Example code to read a velodyne_sync/[utime].bin file
# Plots the point cloud using matplotlib. Also converts
# to a CSV if desired.
#
# To call:
#
#   python read_vel_sync.py velodyne.bin [out.csv]
#

import sys
import struct
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import IPython
import os
import time

def xyz_array_to_pointcloud2(points, stamp, frame_id):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    '''
    msg = PointCloud2()

    num_values = points.shape[0]
    assert(num_values > 0)

    NUM_FIELDS = 5
    assert(np.mod(num_values, NUM_FIELDS) == 0)
    
    num_points = num_values / NUM_FIELDS

    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id

    assert(len(points.shape) == 1)
    msg.height = 1

    FLOAT_SIZE_BYTES = 4
    msg.width = num_values * FLOAT_SIZE_BYTES

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('i', 12, PointField.FLOAT32, 1),
        PointField('l', 16, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = NUM_FIELDS * FLOAT_SIZE_BYTES

    msg.row_step = msg.point_step * num_points
    msg.is_dense = False #int(np.isfinite(points).all())

    msg.width = num_points
    msg.data = np.asarray(points, np.float32).tostring()

    return msg

def convert(x_s, y_s, z_s):

    scaling = 0.005 # 5 mm
    offset = -100.0

    x = x_s * scaling + offset
    y = y_s * scaling + offset
    z = z_s * scaling + offset

    return x, y, z

def parse_vel_sync_csv(filepath, stamp, frame_id):
    assert(os.path.exists(filepath))
  
    f_bin = open(filepath, "r")

    hits = []

    for line in f_bin:
      values = line.split(',')
      assert(len(values) == 5)
      
      x = float(values[0])
      y = float(values[1])
      z = float(values[2])
      i = float(values[3])
      l = float(values[4])

      #x, y, z = convert(x, y, z)

      hits += [x, y, z, i, l]

    f_bin.close()
    hits = np.asarray(hits)

    pc2_msg = xyz_array_to_pointcloud2(hits, stamp, frame_id)

    return pc2_msg

def parse_vel_sync(filepath, stamp, frame_id):
    assert(os.path.exists(filepath))
  
    start = time.time()

    f_bin = open(filepath, "r")

    end = time.time()
    print 'Opening file took: ',  end - start, 's.'

    hits = []

    start = time.time()
    while True:
        x_str = f_bin.read(2)
        if x_str == '': # eof
            break


        x = struct.unpack('<H', x_str)[0]
        y = struct.unpack('<H', f_bin.read(2))[0]
        z = struct.unpack('<H', f_bin.read(2))[0]
        i = struct.unpack('B', f_bin.read(1))[0]
        l = struct.unpack('B', f_bin.read(1))[0]

        x, y, z = convert(x, y, z)

        s = "%5.3f, %5.3f, %5.3f, %d, %d" % (x, y, z, i, l)

        hits += [x, y, z, i, l]

    end = time.time()
    print 'Unpacking took: ',  end - start, 's.'

    start = time.time()
    f_bin.close()
    hits = np.asarray(hits)

    pc2_msg = xyz_array_to_pointcloud2(hits, stamp, frame_id)
    end = time.time()
    print 'PC2 conversion took: ',  end - start, 's.'

    return pc2_msg

def main(args):

    if len(sys.argv) < 2:
        print 'Please specify velodyne file'
        return 1

    f_bin = open(sys.argv[1], "r")

    if len(sys.argv) >= 3:
        print 'Writing to ', sys.argv[2]
        f_csv = open(sys.argv[2], "w")
    else:
        f_csv = None

    hits = []

    while True:

        x_str = f_bin.read(2)
        if x_str == '': # eof
            break

        x = struct.unpack('<H', x_str)[0]
        y = struct.unpack('<H', f_bin.read(2))[0]
        z = struct.unpack('<H', f_bin.read(2))[0]
        i = struct.unpack('B', f_bin.read(1))[0]
        l = struct.unpack('B', f_bin.read(1))[0]

        x, y, z = convert(x, y, z)

        s = "%5.3f, %5.3f, %5.3f, %d, %d" % (x, y, z, i, l)

        if f_csv:
            f_csv.write('%s\n' % s)

        hits += [[x, y, z]]

    f_bin.close()

    if f_csv:
        f_csv.close()

    hits = np.asarray(hits)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(hits[:, 0], hits[:, 1], -hits[:, 2], c=-hits[:, 2], s=5, linewidths=0)
    plt.show()

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv))
