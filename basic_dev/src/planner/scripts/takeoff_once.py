#!/usr/bin/env python3

import rospy
from airsim_ros.srv import Takeoff, TakeoffRequest


def main():
    rospy.init_node("takeoff_once", anonymous=True)
    service_name = rospy.get_param("~service_name", "/airsim_node/drone_1/takeoff")
    wait_on_last_task = rospy.get_param("~wait_on_last_task", True)
    startup_delay = rospy.get_param("~startup_delay", 1.0)

    rospy.sleep(startup_delay)
    rospy.loginfo("Waiting for takeoff service: %s", service_name)
    rospy.wait_for_service(service_name, timeout=10.0)
    takeoff = rospy.ServiceProxy(service_name, Takeoff)
    req = TakeoffRequest()
    req.waitOnLastTask = wait_on_last_task
    resp = takeoff(req)
    rospy.loginfo("Takeoff service called. success=%s", getattr(resp, "success", "unknown"))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("Takeoff call failed: %s", exc)
