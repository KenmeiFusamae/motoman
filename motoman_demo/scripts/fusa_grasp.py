#!/usr/bin/env python

import math
import time
import moveit_commander
import moveit_msgs.msg
import rospy
import geometry_msgs.msg
import copy
import tf2_ros
import tf
from std_msgs.msg import Int32
from ar_pose.msg import ARMarker


def callback(message):
    print message.pose.pose.position.x

def main():
    #======== init node ========#
    rospy.init_node('ar_lisner')


    sub = rospy.Subscriber('/ar_pose_marker',ARMarker,callback)

    #======== moveit init ========#
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm_initial_pose = arm.get_current_pose().pose
    target_pose = geometry_msgs.msg.Pose()
    #arm.set_planner_id('RRTConnectkConfigDefault')

    #======== tf lisner ========#
    tf_buffer = tf2_ros.Buffer()
    tf_listner = tf2_ros.TransformListener(tf_buffer)
    get_tf_flg = False

    count = 0

    while not get_tf_flg :
        try :
            start = time.time()
            print "count = " , count
            count = count +1

            trans = tf_buffer.lookup_transform('world', 'ar_marker', rospy.Time(0),rospy.Duration(10))
            print "world -> ar_marker"
            print trans.transform


            target_pose.position.x = trans.transform.translation.x
            target_pose.position.y = trans.transform.translation.y
            target_pose.position.z = trans.transform.translation.z
            q = (trans.transform.rotation.x,
                 trans.transform.rotation.y,
                 trans.transform.rotation.z,
                 trans.transform.rotation.w)
            (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(q)
            # roll -= math.pi/6.0
            pitch += math.pi/2.0
            # yaw += math.pi/4.0
            tar_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            target_pose.orientation.x = tar_q[0]
            target_pose.orientation.y = tar_q[1]
            target_pose.orientation.z = tar_q[2]
            target_pose.orientation.w = tar_q[3]

            # arm.set_pose_target(target_pose)
            # print "====== arm go ====== "
            # arm.go()
            # arm.clear_pose_targets()
            #
            # elapsed_time = time.time() - start
            # print("elapsed_time:{0}".format(elapsed_time))
            # print "====== arm clear ====== "

#compute_cartesian_path
            waypoints =[]
            waypoints.append(arm.get_current_pose().pose)

            wpose = geometry_msgs.msg.Pose()
            wpose.position.x = target_pose.orientation.x
            wpose.position.y = target_pose.orientation.y -0.15
            wpose.position.z = target_pose.orientation.z
            wpose.orientation.w = target_pose.orientation.w
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.z -= 0.1
            waypoints.append(copy.deepcopy(wpose))

            (plan, fraction) = arm.compute_cartesian_path(waypoints, 0.01, 0.0)
            arm.set_pose_target(plan)
            arm.go()
            rospy.sleep(5)


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) :
            continue


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
