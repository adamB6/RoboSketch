#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, pi, degrees, radians
import smach
import tf

class Forward(smach.State):
    def __init__(self, distance, rate):
        smach.State.__init__(self,
                             outcomes=["move_fwd", "do_turn", "do_stop"])
        self._distance = distance
        self._rate = rate
        self._pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._sub = rospy.Subscriber("/odom", Odometry, self.process_odom)
        self._position = None
        self._initialPosition = None

    def process_odom(self, data):
        self._position = (data.pose.pose.position.x, data.pose.pose.position.y)

    def execute(self, ud):
        if self._position is None:
            rospy.loginfo("Waiting for position")
            return 'move_fwd'

        position = tuple(self._position)

        rospy.loginfo(self._position)
        if self._initialPosition is None:
            self._initialPosition = position

        distance = sqrt(pow(position[0] - self._initialPosition[0], 2) + pow(position[1] - self._initialPosition[1], 2))
        rospy.loginfo(distance)
        twist = Twist()
        if distance < self._distance:
            twist.linear.x = .25
            transition = "move_fwd"
        else:
            twist.linear.x = 0
            self._initialPosition = None
            transition = "do_turn"
        self._pub.publish(twist)
        self._rate.sleep()
        return transition


class Turn(smach.State):
    def __init__(self, rate):
        smach.State.__init__(self,
                             outcomes=["start_fwd", "do_stop", "do_turn"])
        self._rate = rate
        self._pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._sub = rospy.Subscriber("/odom", Odometry, self.process_odom)
        self._yaw = None
        self._start_yaw = None

    def process_odom(self, data):
        pose = data.pose.pose

        quaternion = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w)

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self._yaw = yaw

    def execute(self, ud):
        if self._yaw is None:
            rospy.loginfo("Waiting for yaw")
            return "do_turn"

        if self._start_yaw is None:
            self._start_yaw = self._yaw

        current_yaw = degrees(self._yaw)
        goal_yaw = degrees(self._start_yaw + 90) % 180

        twist = Twist()
        if abs(current_yaw - goal_yaw) < 3:
            twist.angular.z = 0
            self._start_yaw = None
            transition = "start_fwd"
        else:
            twist.angular.z = .1
            transition = "do_turn"

        self._pub.publish(twist)
        self._rate.sleep()
        return transition


class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=["exit_sm"])

    def execute(self, ud):
        pass


def main():
    rospy.init_node("move_robot_smach", anonymous=True)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(2)
    distance = 2

    sm = smach.StateMachine(outcomes=["exit_sm"])

    with sm:
        smach.StateMachine.add("FORWARD", Forward(distance, rate),
                               transitions={"move_fwd": "FORWARD",
                                            "do_turn": "TURN",
                                            "do_stop": "STOP"
                                            })
        smach.StateMachine.add("TURN", Turn(rate),
                               transitions={"start_fwd": "FORWARD",
                                            "do_turn": "TURN",
                                            "do_stop": "STOP"
                                            })
        smach.StateMachine.add("STOP", Stop(),
                               transitions={"exit_sm": "exit_sm"
                                            })
    sm.execute()


if __name__ == '__main__':
    main()
