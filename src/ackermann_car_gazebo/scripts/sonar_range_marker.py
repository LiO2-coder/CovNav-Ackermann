#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker


class SonarRangeMarker:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "sonar/range")
        marker_topic = rospy.get_param("~marker_topic", "sonar/marker")
        self.fallback_fov = float(rospy.get_param("~fallback_fov", math.radians(30.0)))
        self.segments = max(2, int(rospy.get_param("~segments", 24)))
        self.line_width = float(rospy.get_param("~line_width", 0.03))
        self.alpha = float(rospy.get_param("~alpha", 0.95))

        self.publisher = rospy.Publisher(marker_topic, Marker, queue_size=1)
        self.subscriber = rospy.Subscriber(input_topic, Range, self.range_callback, queue_size=1)

    def range_callback(self, msg):
        fov = msg.field_of_view
        if not math.isfinite(fov) or fov <= 1.0e-6:
            fov = self.fallback_fov
        fov = max(0.0, fov)

        distance = msg.range
        if not math.isfinite(distance) or distance < msg.min_range or distance > msg.max_range:
            distance = msg.max_range
        distance = max(msg.min_range, min(msg.max_range, distance))

        marker = Marker()
        marker.header.frame_id = msg.header.frame_id or "front_sonar_link"
        marker.header.stamp = rospy.Time(0)
        marker.ns = "horizontal_sonar_fov"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 1.0
        marker.color.a = self.alpha
        marker.lifetime = rospy.Duration(0.2)

        self.fill_horizontal_fan(marker, fov, distance)
        self.publisher.publish(marker)

    def fill_horizontal_fan(self, marker, fov, distance):
        origin = Point()
        start = -0.5 * fov
        step = fov / float(self.segments) if self.segments else 0.0

        endpoints = []
        for i in range(self.segments + 1):
            angle = start + step * i
            point = Point()
            point.x = distance * math.cos(angle)
            point.y = distance * math.sin(angle)
            point.z = 0.0
            endpoints.append(point)

        # Arc outline.
        for left, right in zip(endpoints[:-1], endpoints[1:]):
            marker.points.append(left)
            marker.points.append(right)

        # Boundary and a few interior rays make a very narrow FOV still visible.
        stride = max(1, self.segments // 4)
        ray_indices = sorted(set([0, self.segments] + list(range(0, self.segments + 1, stride))))
        for index in ray_indices:
            marker.points.append(origin)
            marker.points.append(endpoints[index])


if __name__ == "__main__":
    rospy.init_node("sonar_range_marker")
    SonarRangeMarker()
    rospy.spin()
