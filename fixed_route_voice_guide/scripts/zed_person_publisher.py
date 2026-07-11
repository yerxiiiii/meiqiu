#!/usr/bin/env python3
"""
ZED mini -> person 3D positions -> /zed/person_positions

Uses the ZED SDK's object detection module directly (pyzed), not the
official zed-ros-wrapper package (not installed on this robot; SDK +
object detection AI models are already present at /usr/local/zed).

Coordinate system is set to RIGHT_HANDED_Z_UP (X=right, Y=forward, Z=up)
in meters, to match uwb_publisher.py's convention (X=right-positive,
Y=forward-positive) as closely as possible given the camera and UWB
base station are mounted at different physical locations on the robot
(this is a rough, uncalibrated match — see person_match_node.py).

Publishes geometry_msgs/PoseArray on /zed/person_positions: one Pose
per currently-detected person, position only (orientation left at
identity, ZED doesn't give a meaningful body orientation by default).

Usage:
  python3 zed_person_publisher.py
  python3 zed_person_publisher.py --min-confidence 50
"""

from __future__ import annotations

import argparse

import pyzed.sl as sl
import rospy
from geometry_msgs.msg import Pose, PoseArray


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="ZED person detection -> ROS")
    p.add_argument("--min-confidence", type=float, default=40.0)
    p.add_argument("--topic", default="/zed/person_positions")
    p.add_argument("--rate", type=float, default=15.0, help="max publish rate (Hz)")
    return p


def main() -> None:
    args = build_parser().parse_args()

    rospy.init_node("zed_person_publisher", anonymous=False)
    pub = rospy.Publisher(args.topic, PoseArray, queue_size=1)

    zed = sl.Camera()
    init = sl.InitParameters()
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
    init.depth_mode = sl.DEPTH_MODE.NEURAL

    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        rospy.logerr(f"ZED open failed: {status}")
        raise SystemExit(1)

    status = zed.enable_positional_tracking(sl.PositionalTrackingParameters())
    if status != sl.ERROR_CODE.SUCCESS:
        rospy.logerr(f"enable_positional_tracking failed: {status}")
        raise SystemExit(1)

    obj_param = sl.ObjectDetectionParameters()
    obj_param.enable_tracking = True
    obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_FAST
    status = zed.enable_object_detection(obj_param)
    if status != sl.ERROR_CODE.SUCCESS:
        rospy.logerr(f"enable_object_detection failed: {status}")
        raise SystemExit(1)

    runtime = sl.ObjectDetectionRuntimeParameters()
    runtime.detection_confidence_threshold = args.min_confidence
    runtime.object_class_filter = [sl.OBJECT_CLASS.PERSON]

    objects = sl.Objects()
    grab_rt = sl.RuntimeParameters()

    rospy.loginfo(f"ZED person publisher running, publishing to {args.topic}")
    rate = rospy.Rate(args.rate)
    try:
        while not rospy.is_shutdown():
            if zed.grab(grab_rt) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_objects(objects, runtime)

                msg = PoseArray()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "zed_camera"
                for obj in objects.object_list:
                    pos = obj.position
                    pose = Pose()
                    pose.position.x = float(pos[0])
                    pose.position.y = float(pos[1])
                    pose.position.z = float(pos[2])
                    pose.orientation.w = 1.0
                    msg.poses.append(pose)
                pub.publish(msg)
            rate.sleep()
    finally:
        zed.disable_object_detection()
        zed.disable_positional_tracking()
        zed.close()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
