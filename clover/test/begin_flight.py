#! /usr/bin/env python3

import threading
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_srvs.srv import SetBool, SetBoolResponse
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

takeoff_alt = 1.5 # m

setpoint_pose = None
offboard_control_enabled = True

current_state = State()
current_pose = PoseStamped()

home_lock = threading.Lock()
setpoint_lock = threading.Lock()

def state_cb(msg):
    global current_state
    current_state = msg

def setpoint_cb(msg):
    global setpoint_pose
    with setpoint_lock:
        setpoint_pose = msg

def dronePose_cb(msg):
    global current_pose
    with home_lock:
        current_pose = msg

def offboard_control_cb(req):
    global offboard_control_enabled

    offboard_control_enabled = req.data

    if offboard_control_enabled:
        rospy.loginfo("OFFBOARD mode control enabled")
    else:
        rospy.loginfo(
            "OFFBOARD mode control disabled - external mode change allowed"
        )

    return SetBoolResponse(
        success=True,
        message=f"offboard_control_enabled={offboard_control_enabled}"
    )

def set_home():
    global home_pose

    with home_lock:
        home_pose = current_pose

    home_pose.header.stamp = rospy.Time.now()
    home_pose.header.frame_id = "map"

    home_pub.publish(home_pose)

    rospy.loginfo(
        "Home set at (%.2f, %.2f, %.2f)",
        home_pose.pose.position.x,
        home_pose.pose.position.y,
        home_pose.pose.position.z,
    )

def begin_flight():
    alt_flag = False

    takeoff_pose = PoseStamped()

    takeoff_pose.pose.position.x = home_pose.pose.position.x
    takeoff_pose.pose.position.y = home_pose.pose.position.y
    takeoff_pose.pose.position.z = takeoff_alt

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(takeoff_pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while not rospy.is_shutdown():

        # Only force OFFBOARD while this node owns mode control.
        if offboard_control_enabled:

            if (
                current_state.mode != "OFFBOARD"
                and (rospy.Time.now() - last_req) > rospy.Duration(5.0)
            ):
                if set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()

            elif (
                not current_state.armed
                and (rospy.Time.now() - last_req) > rospy.Duration(5.0)
            ):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        # Continue publishing setpoints regardless of mode-control ownership.
        if not alt_flag:
            local_pos_pub.publish(takeoff_pose)

            if (
                current_pose.pose.position.z > 0.9 * takeoff_alt
                and setpoint_pose is not None
            ):
                alt_flag = True

        else:
            rospy.loginfo_once("Tracking Trajectory...")

            if setpoint_pose is not None:
                local_pos_pub.publish(setpoint_pose)

        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    offboard_control_srv = rospy.Service(
        "/clover1/offboard_control",
        SetBool,
        offboard_control_cb
    )

    setpoint_sub = rospy.Subscriber(
        "/clover1/target_pose",
        PoseStamped,
        callback=setpoint_cb,
        queue_size=1
    )

    home_pub = rospy.Publisher(
        "/clover1/home_pose",
        PoseStamped,
        queue_size=1,
        latch=True
    )

    dronePose_sub = rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, callback = dronePose_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rospy.wait_for_message("/mavros/vision_pose/pose", PoseStamped, timeout=30)

    set_home()

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    begin_flight()