# !/usr/bin/python
#
# Convert the sensor data files in the given directory to a single rosbag.
#
# To call:
#
#   python sensordata_to_rosbag.py 2012-01-08/ 2012-01-08.bag
#

import rosbag, rospy
from std_msgs.msg import Float64, UInt16, Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import NavSatStatus, NavSatFix, PointCloud2, PointField, Image, Imu, MagneticField
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from transformation_helper import *
from read_vel_sync import *
from scipy import misc
from parse_odometry import *
import sys
import numpy as np
import struct
import IPython
import os
import time

#def xyz_array_to_pointcloud2(points, stamp=None, frame_id=None):
#    '''
#    Create a sensor_msgs.PointCloud2 from an array
#    of points.
#    '''
#    msg = PointCloud2()
#
#    num_values = points.shape[0]
#    assert(num_values > 0)
#
#    NUM_FIELDS = 5
#    assert(np.mod(num_values, NUM_FIELDS) == 0)
#    
#    num_points = num_values / NUM_FIELDS
#
#    if stamp:
#        msg.header.stamp = stamp
#    if frame_id:
#        msg.header.frame_id = frame_id
#
#    assert(len(points.shape) == 1)
#    msg.height = 1
#
#    FLOAT_SIZE_BYTES = 4
#    msg.width = num_values * FLOAT_SIZE_BYTES
#
#    msg.fields = [
#        PointField('x', 0, PointField.FLOAT32, 1),
#        PointField('y', 4, PointField.FLOAT32, 1),
#        PointField('z', 8, PointField.FLOAT32, 1),
#        PointField('i', 12, PointField.FLOAT32, 1),
#        PointField('l', 16, PointField.FLOAT32, 1)]
#    msg.is_bigendian = False
#    msg.point_step = NUM_FIELDS * FLOAT_SIZE_BYTES
#
#    msg.row_step = msg.point_step * num_points
#    msg.is_dense = False #int(np.isfinite(points).all())
#
#    msg.width = num_points
#    msg.data = np.asarray(points, np.float32).tostring()
#
#    return msg

def write_gps(gps, i, bag):

    utime = gps[i, 0]
    mode = gps[i, 1]

    lat = gps[i, 3]
    lng = gps[i, 4]
    alt = gps[i, 5]

    timestamp = rospy.Time.from_sec(utime/1e6)

    status = NavSatStatus()

    if mode==0 or mode==1:
        status.status = NavSatStatus.STATUS_NO_FIX
    else:
        status.status = NavSatStatus.STATUS_FIX

    status.service = NavSatStatus.SERVICE_GPS

    num_sats = UInt16()
    num_sats.data = gps[i, 2]

    fix = NavSatFix()
    fix.status = status

    fix.latitude = np.rad2deg(lat)
    fix.longitude = np.rad2deg(lng)
    fix.altitude = alt

    track = Float64()
    track.data = gps[i, 6]

    speed = Float64()
    speed.data = gps[i, 7]

    bag.write('gps_fix', fix, t=timestamp)
    bag.write('gps_track', track, t=timestamp)
    bag.write('gps_speed', speed, t=timestamp)

def write_gps_rtk(gps, i, bag):

    utime = gps[i, 0]
    mode = gps[i, 1]

    lat = gps[i, 3]
    lng = gps[i, 4]
    alt = gps[i, 5]

    timestamp = rospy.Time.from_sec(utime/1e6)

    status = NavSatStatus()

    if mode==0 or mode==1:
        status.status = NavSatStatus.STATUS_NO_FIX
    else:
        status.status = NavSatStatus.STATUS_FIX

    status.service = NavSatStatus.SERVICE_GPS

    num_sats = UInt16()
    num_sats.data = gps[i, 2]

    fix = NavSatFix()
    fix.status = status

    fix.latitude = np.rad2deg(lat)
    fix.longitude = np.rad2deg(lng)
    fix.altitude = alt

    track = Float64()
    track.data = gps[i, 6]

    speed = Float64()
    speed.data = gps[i, 7]

    bag.write('gps_rtk_fix', fix, t=timestamp)
    bag.write('gps_rtk_track', track, t=timestamp)
    bag.write('gps_rtk_speed', speed, t=timestamp)

def write_odom(time_us_T_O_Bks_with_covs, i, bag):

    utime = time_us_T_O_Bks_with_covs[i][0]

    T_O_B = time_us_T_O_Bks_with_covs[i][1]
    cov = time_us_T_O_Bks_with_covs[i][2]

    timestamp = rospy.Time.from_sec(utime/1e6)

    rospose = Odometry()
    rospose.child_frame_id = "Odometry"
    rospose.header.stamp = timestamp
    rospose.pose.pose.position.x = T_O_B.getPosition()[0]
    rospose.pose.pose.position.y = T_O_B.getPosition()[1]
    rospose.pose.pose.position.z = T_O_B.getPosition()[2]
    rospose.pose.pose.orientation.x = T_O_B.getRotation().x()
    rospose.pose.pose.orientation.y = T_O_B.getRotation().y()
    rospose.pose.pose.orientation.z = T_O_B.getRotation().z()
    rospose.pose.pose.orientation.w = T_O_B.getRotation().w()
    rospose.twist.twist.linear.x = 0.0
    rospose.twist.twist.linear.y = 0.0
    rospose.twist.twist.linear.z = 0.0
    rospose.twist.twist.angular.x = 0.0
    rospose.twist.twist.angular.y = 0.0
    rospose.twist.twist.angular.z = 0.0

    bag.write('odometry', rospose, t=timestamp)

def write_ms25(ms25, i, bag):

    utime = ms25[i, 0]

    mag_x = ms25[i, 1]
    mag_y = ms25[i, 2]
    mag_z = ms25[i, 3]

    accel_x = ms25[i, 4]
    accel_y = ms25[i, 5]
    accel_z = ms25[i, 6]

    rot_r = ms25[i, 7]
    rot_p = ms25[i, 8]
    rot_h = ms25[i, 9]

    timestamp = rospy.Time.from_sec(utime/1e6)

    rosimu = Imu()
    rosimu.header.stamp = timestamp
    rosimu.angular_velocity.x = float(rot_r)
    rosimu.angular_velocity.y = float(rot_p)
    rosimu.angular_velocity.z = float(rot_h)
    rosimu.linear_acceleration.x = float(accel_x)
    rosimu.linear_acceleration.y = float(accel_y)
    rosimu.linear_acceleration.z = float(accel_z)
    bag.write('ms25_imu', rosimu, t=timestamp)

    rosmag = MagneticField()
    rosmag.header.stamp = timestamp
    rosmag.magnetic_field.x = mag_x
    rosmag.magnetic_field.y = mag_y
    rosmag.magnetic_field.z = mag_z
    bag.write('ms25_mag', rosmag, t=timestamp)

def write_ms25_euler(ms25_euler, i, bag):

    utime = ms25_euler[i, 0]

    r = ms25_euler[i, 1]
    p = ms25_euler[i, 2]
    h = ms25_euler[i, 3]

    timestamp = rospy.Time.from_sec(utime/1e6)

    rosquat = QuaternionStamped()
    rosquat.header.stamp = timestamp

    q = getQuaternionFromFromEulerAnglesRollPitchYaw(r, p, h)

    rosquat.quaternion.x = q.x()
    rosquat.quaternion.y = q.y()    
    rosquat.quaternion.z = q.z()   
    rosquat.quaternion.w = q.w()

    bag.write('ms25_orientation', rosquat, t=timestamp)

def convert_vel(x_s, y_s, z_s):

    scaling = 0.005 # 5 mm
    offset = -100.0

    x = x_s * scaling + offset
    y = y_s * scaling + offset
    z = z_s * scaling + offset

    return x, y, z

def verify_magic(s):

    magic = 44444

    m = struct.unpack('<HHHH', s)

    return len(m)>=3 and m[0] == magic and m[1] == magic and m[2] == magic and m[3] == magic

def read_next_vel_packet(f_vel):

    try:
        magic = f_vel.read(8)
        if magic == '': # eof
            return -1, None

        if not verify_magic(magic):
            print "Could not verify magic"
            return -1, None

        num_hits = struct.unpack('<I', f_vel.read(4))[0]
        utime = struct.unpack('<Q', f_vel.read(8))[0]

        f_vel.read(4) # padding

        # Read all hits
        data = []
        for i in range(num_hits):

            x = struct.unpack('<H', f_vel.read(2))[0]
            y = struct.unpack('<H', f_vel.read(2))[0]
            z = struct.unpack('<H', f_vel.read(2))[0]
            i = struct.unpack('B', f_vel.read(1))[0]
            l = struct.unpack('B', f_vel.read(1))[0]

            x, y, z = convert_vel(x, y, z)

            data += [x, y, z, float(i), float(l)]

        return utime, data, num_hits
    except Exception:
        pass

    return -1, None

def write_vel(vel_data, utime, num_hits, bag):

    timestamp = rospy.Time.from_sec(utime/1e6)

    layout = MultiArrayLayout()
    layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
    layout.dim[0].label = "hits"
    layout.dim[0].size = num_hits
    layout.dim[0].stride = 5
    layout.dim[1].label = "xyzil"
    layout.dim[1].size = 5
    layout.dim[1].stride = 1

    vel = Float64MultiArray()
    vel.data = vel_data
    vel.layout = layout

    pc2_msg = xyz_array_to_pointcloud2(np.array(vel_data), timestamp, 'velodyne')

    bag.write('velodyne_packet', pc2_msg, t=timestamp)

def write_vel_sync(utime, velodyne_sync_folder, bag):
    csv_filepath = os.path.join(velodyne_sync_folder, str(utime) + '.bin.csv')
    assert(os.path.exists(csv_filepath))

    timestamp = rospy.Time.from_sec(utime/1e6)

    FRAME_ID = 'velodyne'
    pc2_msg = parse_vel_sync_csv(csv_filepath, timestamp, FRAME_ID)

    bag.write('velodyne_full_scan', pc2_msg, t=timestamp)
    end = time.time()

def write_images(utime, images_folder, bag):
    timestamp = rospy.Time.from_sec(utime/1e6)

    NUM_CAMERAS = 5
    num_images_added = 0
    for camera_idx in range(NUM_CAMERAS):
        cam_filepath = os.path.join(images_folder, 'Cam' + str(camera_idx), str(utime) + '.png')
        if os.path.exists(cam_filepath):
              img_array = misc.imread(cam_filepath)
              assert(img_array.dtype == 'uint8')

              rosimage = Image()
              rosimage.data = img_array.tostring()
              rosimage.step = img_array.shape[1] #only with mono8! (step = width * byteperpixel * numChannels)
              rosimage.encoding = "mono8"
              rosimage.height = img_array.shape[0]
              rosimage.width = img_array.shape[1]
              rosimage.header.stamp = timestamp

              bag.write('cam' + str(camera_idx) + '/image_raw', rosimage, t=timestamp)

              num_images_added += 1
            
    print 'Added ', num_images_added, ' to the bag.'

def convert_hokuyo(x_s):

    scaling = 0.005 # 5 mm
    offset = -100.0

    x = x_s * scaling + offset

    return x

def read_next_hokuyo_30m_packet(f_bin):

    try:

        # hokuyo_30m always has 1081 hits
        num_hits = 1081

        # Read timestamp
        utime = struct.unpack('<Q', f_bin.read(8))[0]

        r = np.zeros(num_hits)

        for i in range(num_hits):
            s = struct.unpack('<H', f_bin.read(2))[0]
            r[i] = convert_hokuyo(s)

        return utime, r

    except Exception:
        pass

    return -1, None

def write_hokuyo_30m_packet(hok_30m, utime, bag):

    # hokuyo_30m always has 1081 hits
    num_hits = 1081

    timestamp = rospy.Time.from_sec(utime/1e6)

    layout = MultiArrayLayout()
    layout.dim = [MultiArrayDimension()]
    layout.dim[0].label = "r"
    layout.dim[0].size = num_hits
    layout.dim[0].stride = 1

    hits = Float64MultiArray()
    hits.data = hok_30m
    hits.layout = layout

    bag.write('hokuyo_30m_packet', hits, t=timestamp)

def read_next_hokuyo_4m_packet(f_bin):

    try:

        # hokuyo_4m always has 726 hits
        num_hits = 726

        # Read timestamp
        utime_str = f_bin.read(8)
        if utime_str == '': #EOF
            return -1, None

        #utime = struct.unpack('<Q', f_bin.read(8))[0]
        utime = struct.unpack('<Q', utime_str)[0]

        r = np.zeros(num_hits)

        for i in range(num_hits):
            s = struct.unpack('<H', f_bin.read(2))[0]
            r[i] = convert_hokuyo(s)

        return utime, r

    except Exception as e:
        print utime_str
        print len(utime_str)
        print i
        print r
        raise e
        pass

    return -1, None

def write_hokuyo_4m_packet(hok_4m, utime, bag):

    # hokuyo_4m always has 726 hits
    num_hits = 726

    timestamp = rospy.Time.from_sec(utime/1e6)

    layout = MultiArrayLayout()
    layout.dim = [MultiArrayDimension()]
    layout.dim[0].label = "r"
    layout.dim[0].size = num_hits
    layout.dim[0].stride = 1

    hits = Float64MultiArray()
    hits.data = hok_4m
    hits.layout = layout

    bag.write('hokuyo_4m_packet', hits, t=timestamp)

def get_velodyne_sync_timestamps(velodyne_sync_folder):
    assert(os.path.exists(velodyne_sync_folder))

    files = os.listdir(velodyne_sync_folder)

    timestamps_mikroseconds = sorted([long(os.path.splitext(os.path.splitext(f)[0])[0]) for f in files])
    
    return timestamps_mikroseconds

def get_image_timestamps(images_folder):
    assert(os.path.exists(images_folder))

    dirs = os.listdir(images_folder)
    assert(len(dirs) > 0)
    
    for d in dirs:
      assert(os.path.isdir(os.path.join(images_folder, d)))
      if d == 'Cam0':
        image_filenames = os.listdir(os.path.join(images_folder, d))
        
        timestamps_mikroseconds = sorted([long(os.path.splitext(f)[0]) for f in image_filenames])

      
    assert(len(timestamps_mikroseconds) > 0)
    return timestamps_mikroseconds

def main(args):

    if len(sys.argv) < 2:
        print 'Please specify sensor data directory file'
        return 1

    if len(sys.argv) < 3:
        print 'Please specify output rosbag file'
        return 1

    bag = rosbag.Bag(sys.argv[2], 'w')

    gps = np.loadtxt(sys.argv[1] + "gps.csv", delimiter = ",")
    gps_rtk = np.loadtxt(sys.argv[1] + "gps_rtk.csv", delimiter = ",")
    ms25 = np.loadtxt(sys.argv[1] + "ms25.csv", delimiter = ",")
    ms25_euler = np.loadtxt(sys.argv[1] + "ms25_euler.csv", delimiter = ",")

    time_us_T_O_Bks_with_covs = parse_synchronized_odometry(sys.argv[1] + "odometry_mu.csv", sys.argv[1] + "odometry_cov.csv")

    i_gps = 0
    i_gps_rtk = 0
    i_ms25 = 0
    i_ms25_euler = 0
    i_odom = 0
    i_vel = 0
    i_img = 0

    vel_sync_folder = os.path.join(sys.argv[1], 'velodyne_sync', 'csv')
    vel_sync_timestamps_microseconds = get_velodyne_sync_timestamps(vel_sync_folder)
    assert(len(vel_sync_timestamps_microseconds) > 0)

    images_folder = os.path.join(sys.argv[1], 'images', 'Undistorted-Downscaled-All')
    images_timestamps_microseconds = get_image_timestamps(images_folder)
    assert(len(images_timestamps_microseconds) > 0)

    #f_vel = open(sys.argv[1] + "velodyne_hits.bin", "r")
    #f_hok_30 = open(sys.argv[1] + "hokuyo_30m.bin", "r")
    #f_hok_4 = open(sys.argv[1] + "hokuyo_4m.bin", "r")

    #utime_vel, vel_data, num_hits = read_next_vel_packet(f_vel)
    #utime_hok30, hok30_data = read_next_hokuyo_30m_packet(f_hok_30)
    #utime_hok4, hok4_data = read_next_hokuyo_4m_packet(f_hok_4)

    print 'Loaded data, writing ROSbag...'

    max_num_messages = 20000
    num_messages = 0
    while True:

        # Figure out next packet in time
        next_packet = "done"
        next_utime = -1

        if i_gps < len(gps) and (gps[i_gps, 0] < next_utime or next_utime < 0):
            next_utime = gps[i_gps, 0]
            next_packet = "gps"

        if i_gps_rtk < len(gps_rtk) and (gps_rtk[i_gps_rtk, 0] < next_utime or next_utime < 0):
            next_utime = gps_rtk[i_gps_rtk, 0]
            next_packet = "gps_rtk"

        if i_ms25 < len(ms25) and (ms25[i_ms25, 0] < next_utime or next_utime < 0):
            next_utime = ms25[i_ms25, 0]
            next_packet = "ms25"

        if i_ms25_euler < len(ms25_euler) and (ms25_euler[i_ms25_euler, 0] < next_utime or next_utime < 0):
            next_utime = ms25_euler[i_ms25_euler, 0]
            next_packet = "ms25_euler"

        if i_odom < len(time_us_T_O_Bks_with_covs) and (time_us_T_O_Bks_with_covs[i_odom][0] < next_utime or next_utime < 0):
            next_utime = time_us_T_O_Bks_with_covs[i_odom][0]
            next_packet = "odom"

        if i_vel < len(vel_sync_timestamps_microseconds) and (vel_sync_timestamps_microseconds[i_vel] < next_utime or next_utime < 0):
            next_utime = vel_sync_timestamps_microseconds[i_vel]
            next_packet = "vel_sync"

        if i_img < len(images_timestamps_microseconds) and (images_timestamps_microseconds[i_img] < next_utime or next_utime < 0):
            next_utime = images_timestamps_microseconds[i_img]
            next_packet = "img"

        #if utime_vel>0 and (utime_vel<next_utime or next_utime<0):
        #    next_packet = "vel"

        #if utime_hok30>0 and (utime_hok30<next_utime or next_utime<0):
        #    next_packet = "hok30"

        #if utime_hok4>0 and (utime_hok4<next_utime or next_utime<0):
        #    next_packet = "hok4"

        print 'next packet: ', next_packet

        # Now deal with the next packet
        if next_packet == "done":
            break
        elif next_packet == "gps":
            write_gps(gps, i_gps, bag)
            i_gps = i_gps + 1
        elif next_packet == "gps_rtk":
            write_gps_rtk(gps_rtk, i_gps_rtk, bag)
            i_gps_rtk = i_gps_rtk + 1
        elif next_packet == "ms25":
            write_ms25(ms25, i_ms25, bag)
            i_ms25 = i_ms25 + 1
        elif next_packet == "ms25_euler":
            write_ms25_euler(ms25_euler, i_ms25_euler, bag)
            i_ms25_euler = i_ms25_euler + 1
        elif next_packet == "odom":
            write_odom(time_us_T_O_Bks_with_covs, i_odom, bag)
            i_odom += 1
        elif next_packet == "vel":
            write_vel(vel_data, utime_vel, num_hits, bag)
            utime_vel, vel_data, num_hits = read_next_vel_packet(f_vel)
        elif next_packet == "vel_sync":
            write_vel_sync(next_utime, vel_sync_folder, bag)
            i_vel += 1
        elif next_packet == "hok30":
            write_hokuyo_30m_packet(hok30_data, utime_hok30, bag)
            utime_hok30, hok30_data = read_next_hokuyo_30m_packet(f_hok_30)
        elif next_packet == "hok4":
            write_hokuyo_4m_packet(hok4_data, utime_hok4, bag)
            utime_hok4, hok4_data = read_next_hokuyo_4m_packet(f_hok_4)
        elif next_packet == "img":
            write_images(next_utime, images_folder, bag)
            i_img += 1
        else:
            print "Unknown packet type"

        num_messages += 1
        print 'Num message: ', num_messages
        if num_messages >= max_num_messages:
          break

    #f_vel.close()
    #f_hok_30.close()
    #f_hok_4.close()
    bag.close()

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv))
