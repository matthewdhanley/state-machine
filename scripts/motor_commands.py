import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray


class Pyteleop:
    def __init__(self, publish_topic):
        # add publishers here
        self.pub = rospy.Publisher(publish_topic, Twist, queue_size = 10)

        # shared input arguements
        self.publish_topic = publish_topic

        # tuning the stuff
        self.max_scalar = 2

        # init node
        rospy.init_node('pyteleop')

        # info
        rospy.loginfo("started pyteleop")

    def forward(self, speed):
        cmd_vel = Twist()
        if speed > 100*self.max_scalar:
            print("Speed out of range, clipping.")
            speed = 100*self.max_scalar
        if speed < -100*self.max_scalar:
            print("Speed out of range, clipping.")
            speed = -100*self.max_scalar
        cmd_vel.linear.x = speed/100.0*self.max_scalar
        self.pub.publish(cmd_vel)

    def turn(self, direction):
        cmd_vel = Twist()
        if direction > 100*self.max_scalar:
            print("Speed out of range, clipping.")
            direction = 100*self.max_scalar
        if direction < -100*self.max_scalar:
            print("Speed out of range, clipping.")
            direction = -100*self.max_scalar

        cmd_vel.angular.x = direction/100.0*self.max_scalar
        self.pub.publish(cmd_vel)


def approach_mining(data, robot):
    print("here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!S")
    try:
        y = data.poses[0].position.z
        rospy.loginfo(y)

    except:
        robot.forward(0)
        rospy.loginfo("lost track")
        return

    if y < 4:
        robot.forward((4-y) * 15.0)
    else:
        robot.forward(0)
        rospy.loginfo('Reached Target!  ')
        rospy.signal_shutdown("Found Target.")


def approach_mining_start():
    start()


def start():
    # add subscribers here
    robot = Pyteleop('cmd_vel')
    sub = None
    sub = rospy.Subscriber('/tag_detections_pose', PoseArray, approach_mining,robot)
    rospy.loginfo('approach script started...')
    rospy.spin()
    sub.unregister()
    print("done spinning")


if __name__=="__main__":
    start()

