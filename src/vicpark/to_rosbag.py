import scipy.io as sio

dr = sio.loadmat('./aa3_dr.mat')
lsr = sio.loadmat('./aa3_lsr2.mat')
gps = sio.loadmat('./aa3_gpsx.mat')
tr =  sio.loadmat('./trees.mat')

import rosbag
import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from vicpark_msgs.msg import TreesStamped, Tree
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

with rosbag.Bag('test.bag', 'w') as outbag:

	dr_time = dr['time']
	dr_speed = dr['speed']
	dr_steering = dr['steering']

	for i in range(len(dr_time)):
		msg = AckermannDriveStamped()
		msg.drive.steering_angle = dr_steering[i][0]
		msg.drive.speed = dr_speed[i][0]
		msg.header.stamp = rospy.Time(dr_time[i][0]/1000.)

		outbag.write('/dead_reckoning', msg, msg.header.stamp)

	tr_trees = tr['trees'][0]
	tr_time = tr['time']

	for i in range(len(tr_time)):
		msg = TreesStamped()

		for t in tr_trees[i].transpose():

			tree = Tree()
			tree.distance = t[0]
			tree.angle = t[1]
			tree.diameter = t[2]

			msg.trees.append(tree)

		msg.header.stamp = rospy.Time(tr_time[i][0])

		outbag.write('/trees', msg, msg.header.stamp)

	gps_x = gps['Lo_m']
	gps_y = gps['La_m']
	gps_t = gps['timeGps']

	msg = Path()
	msg.header.stamp = rospy.Time(gps_t[0]/1000.+5)
	msg.header.frame_id = 'world'

	for i in range(len(gps_t)):
		odom = Odometry()
		odom.header.frame_id = 'world'
		odom.child_frame_id = 'odom'



		ps = PoseStamped()
		ps.header.frame_id = 'world'

		ps.pose.position.x = gps_x[i]
		ps.pose.position.y = gps_y[i]
		ps.header.stamp = rospy.Time(gps_t[i]/1000.)

		odom.pose.pose = ps.pose
		odom.header.stamp = ps.header.stamp

		outbag.write('/ground_truth_odom', odom, odom.header.stamp)

		msg.poses.append(ps)
		




	# print msg

	outbag.write('/ground_truth_path', msg, msg.header.stamp)

import subprocess, yaml

info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', 'test.bag'], stdout=subprocess.PIPE).communicate()[0])
print info_dict