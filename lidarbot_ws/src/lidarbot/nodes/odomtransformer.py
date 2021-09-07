#!/usr/bin/env python
import rospy, tf, tf2_ros, geometry_msgs.msg, nav_msgs.msg

def callback(data, args):
	bc = tf2_ros.TransformBroadcaster()
	t = geometry_msgs.msg.TransformStamped()
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = args[0]
	t.child_frame_id = args[1]
	t.transform.translation = data.pose.pose.position
	t.transform.rotation = data.pose.pose.orientation
	bc.sendTransform(t)
	# bc.sendTransform((data.pose.pose.position.x, data.pose.pose.position.y, 0) , (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w), rospy.Time.now(), args[1], args[0])

if __name__ == "__main__":
	rospy.init_node("odomtransformer")
	odomInput = rospy.get_param("~odom_input")
	tfOutput  = rospy.get_param("~tf_output")
	rospy.Subscriber(odomInput, nav_msgs.msg.Odometry, callback, [odomInput, tfOutput])
	rospy.spin()