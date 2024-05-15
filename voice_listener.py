import rospy
from std_msgs.msg import String

def listen_for_command():
    rospy.init_node('voice_command_listener')
    pub = rospy.Publisher('voice_commands', String, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        command = "jack"  # Replace with actual voice command logic
        rospy.loginfo(f"Publishing command: {command}")
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        listen_for_command()
    except rospy.ROSInterruptException:
        pass