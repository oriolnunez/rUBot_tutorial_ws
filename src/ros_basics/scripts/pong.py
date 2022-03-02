import rospy
from std_msgs.msg import String

text = pong

def callback_string(msg):
	global text
	text = msg.data
	new_msg = string
	pub.publish(new_msg)
	rospy.loginfo("%s", text)

rospy.init_node('pong_node')
pub = rospy.Publisher("/pong", String, queue_size=10)
sub = rospy.Subscriber("/ping", String, callback_string)
rospy.spin()
