#!/usr/bin/env python

import tf
import rospy
import storm32
import numpy as np
import serial
import diagnostic_updater
import diagnostic_msgs.msg
from storm32_gimbal.msg import GimbalOrientation
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import QuaternionStamped, Quaternion


def restart_shutdown_callback(event):
    """Restart callback used to make sure service is able to respond before
    shutting down.

    Args:
        event: Timer callback event.
    """
    rospy.logwarn("Gimbal restarted, node shutting down!")
    rospy.signal_shutdown("Gimbal restarted, node shutting down!")


def encode_angle(x):
    """Convert radian angle into degree within +-180 degree.

    Args:
        x: Radian angle to be converted.

    Returns: Degree anagle within +-180 degree.
    """
    while x > np.pi:
        x -= 2 * np.pi
    while x < -np.pi:
        x += 2 * np.pi
    return x * 180 / np.pi


def gimbal_quaternion_callback(msg):
    """Callback function for setting target_orientation. This function will
    terminate the ROS node if it encounters a SerialException.

    Arg:
        msg: target_orientation message.
    """
    quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z,
                  msg.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion, axes="syxz")
    euler = list(map(encode_angle, euler))
    rospy.logdebug_throttle(
        1.0, "set_angles : pitch={0}, roll={1}, yaw={2}, unlimited={3}".format(
            euler[0], euler[1], euler[2], msg.unlimited))
    try:
        response = gimbal.set_angles(*euler, unlimited=msg.unlimited)
        if response != [1, 150, 0]:
            rospy.logerr("STorM32: set_pitch_roll_yaw:" +
                         " response error : {0}:s".format(response))
    except serial.serialutil.SerialException as e:
        rospy.logfatal(e)
        rospy.signal_shutdown("SerialError: {0:s}".format(e))


def get_diagnostics_status(stat):
    """Callback function for periodically update diagnostics. This function
    will terminate the ROS node if it encounters a SerialException.

    Arg:
        stat: Status object required for diaganostic_updater.

    Returns: Updated status object.
    """
    # Get the status
    status = gimbal.get_status()

    # Put all status into stat object
    if status:
        for k, v in status.items():
            stat.add(k, v)
        rospy.loginfo("Gimbal States: {0}".format(status["State"]))
        if status["Battery Connected"]:
            rospy.loginfo("VBAT={0}, Voltage {1}".format(
                status["VBAT"], "low" if status["Bat Voltage Low"] else "OK"))
        else:
            rospy.logwarn("Battery disconnected!")
        # Put the state of the gimbal into diagnostic summary
        state = status["State"]
        if state == "Normal":
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, state)
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, state)
    return stat


def restart_controller(req):
    """Callback function to restart the gimbal controller. This function will
    terminate the ROS node if it encounters a SerialException.

    Arg:
        req: Request object for the ROS Service.

    Returns: Service Response, "True" for success, "False" for error.
    """
    try:
        response = TriggerResponse()
        success = gimbal.restart_controller()
        response.success = success
        if success:
            response.message = "Gimbal restarted successfully!"

            # Shutdown the node on success restart
            sub.unregister()
            pub_timer.shutdown()
            camera_pub.unregister()
            controller_pub.unregister()
            rospy.Timer(rospy.Duration(0.1), restart_shutdown_callback, True)
        else:
            response.message = "Gimbal restart failed!"
        return response

    except serial.serialutil.SerialException as e:
        rospy.logfatal(e)
        rospy.signal_shutdown("SerialError: {0:s}".format(e))


def pub_timer_callback(event):
    """Periodic callback function to publish the current orientation of the
    gimbal. This function  will terminate the ROS node if it ecounters a
    SerialException.

    Arg:
        event: ROS event object for the periodic callback.
    """
    # Create new message and header
    msg = QuaternionStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.get_rostime()

    try:
        imu1 = (gimbal.get_imu1_angles(), camera_pub, "imu1")
        imu2 = (gimbal.get_imu2_angles(), controller_pub, "imu2")
        for euler, pub, name in (imu1, imu2):
            if euler:
                rospy.logdebug_throttle(
                    1.0,
                    "get_{0}_angles: pitch={1:.2f}, roll={2:.2f}, yaw={3:.2f}".
                    format(name, *euler))
                euler = list(map(np.radians, euler))
                q = tf.transformations.quaternion_from_euler(
                    *euler, axes="syxz")
                msg.quaternion.x = q[0]
                msg.quaternion.y = q[1]
                msg.quaternion.z = q[2]
                msg.quaternion.w = q[3]
                pub.publish(msg)

        # diagnostic update is called here because this run at a high rate
        updater.update()
    except serial.serialutil.SerialException as e:
        rospy.logfatal(e)
        rospy.signal_shutdown("SerialError: {0:s}".format(e))


if __name__ == "__main__":
    # Init the ROS node
    rospy.init_node("~control")

    # Getting ROS params
    port = rospy.get_param("~port", "/dev/ttyACM0")
    frame_id = rospy.get_param("~frame_id", "gimbal_ref")

    # Start gimbal oject
    gimbal = storm32.Storm32(port=port)
    version = gimbal.get_version()

    # Print out some diagnostic info
    if version:
        rospy.loginfo("Gimbal found at {0}".format(port))
        for k, v in version.items():
            rospy.loginfo("{0}: {1}".format(k, v))

        # Setup the diagnostic updater
        updater = diagnostic_updater.Updater()
        updater.setHardwareID(frame_id)
        updater.add("Gimbal Diagnostics", get_diagnostics_status)

        # Setup subscriber and publisher
        sub = rospy.Subscriber(
            "~target_orientation",
            GimbalOrientation,
            gimbal_quaternion_callback,
            queue_size=1)
        camera_pub = rospy.Publisher(
            "~camera_orientation", QuaternionStamped, queue_size=1)
        controller_pub = rospy.Publisher(
            "~controller_orientation", QuaternionStamped, queue_size=1)
        restart_srv = rospy.Service("~restart", Trigger, restart_controller)

        # Setup periodic callback for orientation publisher
        pub_timer = rospy.Timer(rospy.Duration(0.01), pub_timer_callback)

        # Spin forever
        rospy.spin()
    else:
        rospy.logfatal("Gimbal unresponsive at {0}".format(port))
