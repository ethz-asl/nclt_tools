import rospy

def microseconds_to_ros_timestamp(time_us):
  timestamp_nanoseconds_str = str(long(long(time_us) * long(1000)))
  timestamp = rospy.Time(
    int(timestamp_nanoseconds_str[0:-9]), int(timestamp_nanoseconds_str[-9:]))
  return timestamp
