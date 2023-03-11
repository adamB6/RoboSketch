#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, pi, degrees, radians
import math
import smach
import tf
import numpy as np

index = 0
class Forward(smach.State):
    def __init__(self, coord_list, rate):
        smach.State.__init__(self,
                             outcomes=["move_fwd", "do_turn", "do_stop"])
        self._pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._sub = rospy.Subscriber("/odom", Odometry, self.process_odom)
        self._coord_list = coord_list
        self._rate = rate
        self._position = None
        self._initialPosition = None

    def move_distance(self):
        global index
        x1 = self._coord_list[index][0]
        x2 = self._coord_list[index + 1][0]
        y1 = self._coord_list[index][1]
        y2 = self._coord_list[index + 1][1]

        delta_y = abs(y2 - y1)
        print(f'delta y = {delta_y}')
        delta_x = abs(x2 - x1)
        print(f'delta x = {delta_x}')

        if delta_y > delta_x:
            move_distance = delta_y
        elif delta_y < delta_x:
            move_distance = delta_x
        else:
            move_distance = 0
        print(f'move_distance = {move_distance}')
        return abs(move_distance)


    def process_odom(self, data):
        self._position = (data.pose.pose.position.x, data.pose.pose.position.y)

    def execute(self, ud):
        global index
        if index >= len(self._coord_list):
            return "exit_sm"
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
        print(f'distance = {distance}')
        print(f'self.move_distance() = {self.move_distance()}')

        if distance < self.move_distance():
            twist.linear.x = .1
            transition = "move_fwd"
        else:
            twist.linear.x = 0
            self._initialPosition = None
            index += 1
            transition = "do_turn"
        self._pub.publish(twist)
        self._rate.sleep()
        return transition


class Turn(smach.State):
    def __init__(self, coord_list, rate):
        smach.State.__init__(self,
                             outcomes=["start_fwd", "do_stop", "do_turn"])
        self._coord_list = coord_list
        self._rate = rate
        self._pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._sub = rospy.Subscriber("/odom", Odometry, self.process_odom)
        self._yaw = None
        self._start_yaw = None

    def turn_distance(self):
        global index

        print(f"index = {index}")
        x1 = self._coord_list[index][0]
        x2 = self._coord_list[index + 1][0]
        y1 = self._coord_list[index][1]
        y2 = self._coord_list[index + 1][1]

        delta_y = y2 - y1
        delta_x = x2 - x1

        if delta_y == 0:
            turn_distance = math.acos(delta_x)
        elif delta_x == 0:
            turn_distance = math.asin(delta_y)
        else:
            turn_distance = math.atan2(delta_y, delta_x)
        return turn_distance

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
        global index

        if index + 1 >= len(self._coord_list):
            return 'do_stop'
        if self._yaw is None:
            rospy.loginfo("Waiting for yaw")
            return "do_turn"

        if self._start_yaw is None:
            self._start_yaw = self._yaw

        current_yaw = self._yaw
        goal_yaw = self.turn_distance()
        print(f'goal yaw= {goal_yaw}')
        print(f'current yaw = {current_yaw}')

        twist = Twist()
        if abs(goal_yaw - current_yaw) < .005:
            twist.angular.z = 0
            self._start_yaw = None
            transition = "start_fwd"
        else:
            twist.angular.z = 2 * (goal_yaw - current_yaw)
            transition = "do_turn"

        self._pub.publish(twist)
        self._rate.sleep()
        return transition


class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=["exit_sm"])

    def execute(self, ud):
        rospy.signal_shutdown("all done")
        return "exit_sm"

'''
def get_rotation(msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
    print(f'yaw= {yaw}')
    return yaw
'''
def main():
    rospy.init_node("move_robot_smach", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)


    # Filler for a function that will feed coordinates from the UI
    coord_list = [(0,0), (1,0), (1,1), (0,1), (0,0)]


    sm = smach.StateMachine(outcomes=["exit_sm"])

    with sm:
        smach.StateMachine.add("FORWARD", Forward(coord_list, rate),
                               transitions={"move_fwd": "FORWARD",
                                            "do_turn": "TURN",
                                            "do_stop": "STOP"
                                            })
        smach.StateMachine.add("TURN", Turn(coord_list, rate),
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

    def process_odom(self, data):
        self._position = (data.pose.pose.position.x, data.pose.pose.position.y)

    def execute(self, ud):
        global index
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
        if distance < self.determine_distance():
            twist.linear.x = .1
            transition = "move_fwd"
        else:
            twist.linear.x = 0
            self._initialPosition = None
            index += 1
            transition = "do_turn"
        self._pub.publish(twist)
        self._rate.sleep()
        return transition


class Turn(smach.State):
    def __init__(self, coord_list, rate):
        smach.State.__init__(self,
                             outcomes=["start_fwd", "do_stop", "do_turn"])
        self._coord_list = coord_list
        self._rate = rate
        self._pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._sub = rospy.Subscriber("/odom", Odometry, self.process_odom)
        self._yaw = None
        self._start_yaw = None

    def determine_distance(self):
        global index
        print(f"index = {index}")
        x1 = self._coord_list[index][0]
        x2 = self._coord_list[index + 1][0]
        y1 = self._coord_list[index][1]
        y2 = self._coord_list[index + 1][1]

        delta_y = y2 - y1
        delta_x = x2 - x1

        if delta_y == 0:
            turn_distance = math.acos(delta_x)
        elif delta_x == 0:
            turn_distance = math.asin(delta_y)
        else:
            turn_distance = math.atan2(delta_y, delta_x)
        return turn_distance

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
        global index
        if self._yaw is None:
            rospy.loginfo("Waiting for yaw")
            return "do_turn"

        if self._start_yaw is None:
            self._start_yaw = self._yaw

        current_yaw = self._yaw
        goal_yaw = self.determine_distance()
        print(f'goal yaw= {goal_yaw}')
        print(f'current yaw = {current_yaw}')

        twist = Twist()
        if goal_yaw - current_yaw < .001:
            twist.angular.z = 0
            self._start_yaw = None
            transition = "start_fwd"
        else:
            twist.angular.z = 1 * (goal_yaw - current_yaw)
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
    rate = rospy.Rate(20)


    # Filler for a function that will feed coordinates from the UI
    coord_list = [(0,0), (1,0), (1,1), (0,1), (0,0)]


    sm = smach.StateMachine(outcomes=["exit_sm"])

    with sm:
        smach.StateMachine.add("FORWARD", Forward(coord_list, rate),
                               transitions={"move_fwd": "FORWARD",
                                            "do_turn": "TURN",
                                            "do_stop": "STOP"
                                            })
        smach.StateMachine.add("TURN", Turn(coord_list, rate),
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
    rate = rospy.Rate(20)
    queue = []
    coord_list = (0, 0, 0, 1, 0, 2, 0, 3)
    for x in coord_list:
        queue.append(x)


    sm = smach.StateMachine(outcomes=["exit_sm"])

    with sm:
        smach.StateMachine.add("FORWARD", Forward(y_coords[1] - y_coords[0], rate),
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
