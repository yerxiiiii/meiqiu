#!/usr/bin/env python3
"""
Match ZED-detected people against the UWB tag position.

Detection-and-logging only — no /cmd_vel, no motion commands of any kind.

Subscribes:
  /zed/person_positions  (geometry_msgs/PoseArray)  - from zed_person_publisher.py
  /uwb/tag_position       (geometry_msgs/PointStamped) - from uwb_publisher.py

For each ZED-detected person, computes straight-line 3D distance to the
current UWB tag position and logs the closest match (if within threshold)
or reports how many unmatched people are in view otherwise.

Note: ZED and UWB base station are mounted at different physical
locations on the robot and this does no extrinsic calibration between
them — it directly compares their two "rough" coordinate frames
(both set to X=right, Y=forward, in meters). Distances will have some
fixed offset error from that until a proper calibration is done.

Usage:
  python3 person_match_node.py
  python3 person_match_node.py --threshold 0.75
"""

from __future__ import annotations

import argparse
import math

import rospy
from geometry_msgs.msg import PointStamped, PoseArray


class PersonMatcher:
    def __init__(self, args: argparse.Namespace):
        self.threshold = args.threshold
        self.uwb_timeout = args.uwb_timeout

        self.uwb_point = None
        self.uwb_stamp = None

        rospy.init_node("person_match_node", anonymous=False)
        rospy.Subscriber(args.uwb_topic, PointStamped, self._on_uwb)
        rospy.Subscriber(args.zed_topic, PoseArray, self._on_zed)

        rospy.loginfo(
            f"person_match_node running: zed={args.zed_topic} uwb={args.uwb_topic} "
            f"threshold={self.threshold:.2f}m"
        )

    def _on_uwb(self, msg: PointStamped) -> None:
        self.uwb_point = (msg.point.x, msg.point.y, msg.point.z)
        self.uwb_stamp = rospy.Time.now()

    def _on_zed(self, msg: PoseArray) -> None:
        if self.uwb_point is None:
            rospy.loginfo_throttle(2.0, "NO UWB DATA YET: waiting for /uwb/tag_position")
            return

        stale = (rospy.Time.now() - self.uwb_stamp).to_sec() > self.uwb_timeout
        if stale:
            rospy.logwarn_throttle(
                2.0, f"UWB DATA STALE (>{self.uwb_timeout:.1f}s old): skipping match"
            )
            return

        if not msg.poses:
            rospy.loginfo("NO PEOPLE DETECTED by ZED this frame")
            return

        ux, uy, uz = self.uwb_point
        best_dist = None
        best_pos = None
        for pose in msg.poses:
            px, py, pz = pose.position.x, pose.position.y, pose.position.z
            d = math.sqrt((px - ux) ** 2 + (py - uy) ** 2 + (pz - uz) ** 2)
            if best_dist is None or d < best_dist:
                best_dist = d
                best_pos = (px, py, pz)

        if best_dist is not None and best_dist <= self.threshold:
            rospy.loginfo(
                f"MATCHED: tracked person at ZED position "
                f"({best_pos[0]:.2f}, {best_pos[1]:.2f}, {best_pos[2]:.2f}), "
                f"distance from UWB estimate: {best_dist:.2f}m"
            )
        else:
            others = len(msg.poses)
            rospy.loginfo(
                f"UNMATCHED: {others} other people detected, none within "
                f"{self.threshold:.2f}m threshold (closest: {best_dist:.2f}m)"
            )


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Match ZED person detections to UWB tag")
    p.add_argument("--threshold", type=float, default=0.75, help="match distance threshold (m)")
    p.add_argument("--uwb-timeout", type=float, default=1.0, help="UWB staleness cutoff (s)")
    p.add_argument("--zed-topic", default="/zed/person_positions")
    p.add_argument("--uwb-topic", default="/uwb/tag_position")
    return p


def main() -> None:
    args = build_parser().parse_args()
    PersonMatcher(args)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
