#!/usr/bin/env python
from autoware_config_msgs.msg import ConfigWaypointReplanner
import rospy

def lane_change():
    rospy.init_node("change_lane", anonymous=True)

    pub = rospy.Publisher("/config/waypoint_replanner", ConfigWaypointReplanner, queue_size=1)

    rospy.sleep(0.5)

    # while not rospy.is_shutdown():
    msg = ConfigWaypointReplanner()

    msg.multi_lane_csv = "/temp/driving_lane_change.csv"
    msg.replanning_mode = False
    msg.use_decision_maker = False
    msg.velocity_max = 20.0
    msg.velocity_min = 4.0
    msg.accel_limit = 0.5
    msg.decel_limit = 0.30000011921
    msg.radius_thresh = 20.0
    msg.radius_min = 6.0
    msg.resample_mode = True
    msg.resample_interval = 1.0
    msg.velocity_offset = 4
    msg.end_point_offset = 1
    msg.braking_distance = 5
    msg.replan_curve_mode = False
    msg.replan_endpoint_mode = True
    msg.overwrite_vmax_mode = False
    msg.realtime_tuning_mode = False

    pub.publish(msg)

if __name__ == "__main__":
    try:
        lane_change()

    except rospy.ROSInterruptException:
        pass
