#!/usr/bin/env python3

import math

import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import NavSatFix, NavSatStatus


EARTH_RADIUS_M = 6378137.0


def quaternion_to_rotation(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def transform_offset(pose, offset):
    rot = quaternion_to_rotation(pose.orientation)
    ox, oy, oz = offset
    return (
        pose.position.x + rot[0][0] * ox + rot[0][1] * oy + rot[0][2] * oz,
        pose.position.y + rot[1][0] * ox + rot[1][1] * oy + rot[1][2] * oz,
        pose.position.z + rot[2][0] * ox + rot[2][1] * oy + rot[2][2] * oz,
    )


class DualRtkPublisher:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "ackermann_car")
        self.origin_lat = math.radians(rospy.get_param("~origin_latitude", 31.230416))
        self.origin_lon = math.radians(rospy.get_param("~origin_longitude", 121.473701))
        self.origin_alt = rospy.get_param("~origin_altitude", 0.0)
        self.cos_origin_lat = math.cos(self.origin_lat)

        self.front_offset = (
            rospy.get_param("~front_offset_x", 0.30),
            rospy.get_param("~front_offset_y", 0.0),
            rospy.get_param("~front_offset_z", 0.30),
        )
        self.rear_offset = (
            rospy.get_param("~rear_offset_x", -0.30),
            rospy.get_param("~rear_offset_y", 0.0),
            rospy.get_param("~rear_offset_z", 0.30),
        )

        self.front_frame_id = rospy.get_param("~front_frame_id", "rtk_front_link")
        self.rear_frame_id = rospy.get_param("~rear_frame_id", "rtk_rear_link")
        front_topic = rospy.get_param("~front_topic", "rtk_front/fix")
        rear_topic = rospy.get_param("~rear_topic", "rtk_rear/fix")

        self.front_pub = rospy.Publisher(front_topic, NavSatFix, queue_size=1)
        self.rear_pub = rospy.Publisher(rear_topic, NavSatFix, queue_size=1)
        self.subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback, queue_size=1)

    def model_states_callback(self, msg):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            rospy.logwarn_throttle(5.0, "Model '%s' not found in /gazebo/model_states", self.model_name)
            return

        pose = msg.pose[idx]
        stamp = rospy.Time.now()
        self.front_pub.publish(self.make_fix(transform_offset(pose, self.front_offset), self.front_frame_id, stamp))
        self.rear_pub.publish(self.make_fix(transform_offset(pose, self.rear_offset), self.rear_frame_id, stamp))

    def make_fix(self, xyz, frame_id, stamp):
        x, y, z = xyz
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = math.degrees(self.origin_lat + y / EARTH_RADIUS_M)
        msg.longitude = math.degrees(self.origin_lon + x / (EARTH_RADIUS_M * self.cos_origin_lat))
        msg.altitude = self.origin_alt + z
        msg.position_covariance = [
            0.0004, 0.0, 0.0,
            0.0, 0.0004, 0.0,
            0.0, 0.0, 0.0009,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return msg


if __name__ == "__main__":
    rospy.init_node("dual_rtk_publisher")
    DualRtkPublisher()
    rospy.spin()
