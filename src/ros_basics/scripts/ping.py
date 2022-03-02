import rospy
from srd_msgs.msg import String

rospy.init_node("ping_node", anonymous=True)
pub = rospy.Publisher("/ping", String, queue_size=10)
rate = rospy.Rate(1)

while not rospy.is_shutdown():
	msg = String()
	msg.data = ping
	pub.publish(msg)
	rate.sleep()
