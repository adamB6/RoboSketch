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
        self._initialPosition2 = None

    def move_distance(self):
        global index
        x1 = self._coord_list[index][0]
        x2 = self._coord_list[index + 1][0]
        y1 = self._coord_list[index][1]
        y2 = self._coord_list[index + 1][1]

        delta_y = abs(y2 - y1)
        print(f'delta y = {delta_y}', end=' ')
        delta_x = abs(x2 - x1)
        print(f'delta x = {delta_x}')

        move_distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        print(f'move_distance = {move_distance}')
        return abs(move_distance)

    def process_odom(self, data):
        self._position = (data.pose.pose.position.x, data.pose.pose.position.y)

    def execute(self, ud):
        global index

        print(f'****************************************************index = {index}')
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
        #print(f'distance = {distance}')
        #print(f'self.move_distance() = {self.move_distance()}')

        # Move the robot according to the coordinates
        if distance < self.move_distance():
            twist.linear.x = .1
            transition = "move_fwd"
        else:            
            # After move between coordinates, pick up pen and move center of robot over final coordinate
            # initialPosition2 is used to store the robots position after moving distance between points

            position = tuple(self._position)
            if self._initialPosition2 is None:
                self._initialPosition2 = position
            move_distance = 0.04
            distance = sqrt(pow(position[0] - self._initialPosition2[0], 2) + pow(position[1] - self._initialPosition2[1], 2))
            print(f'*********move_distance = {move_distance}, distance = {distance}')
            if distance < move_distance:               
                twist.linear.x = 0.005
                transition = "move_fwd"
            else:
                self._initialPosition2 = None
                self._initialPosition = None
                index += 1
                transition = "do_turn"
        self._pub.publish(twist)
        self._rate.sleep()
        return transition


class Back(smach.State):
    def __init__(self, distance, rate):
        smach.State.__init__(self, outcomes=["move_back", "move_fwd"])
        self._pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._sub = rospy.Subscriber("/odom", Odometry, self.process_odom)
        self._rate = rate
        self._distance = distance
        self._position = None
        self._initialPosition = None

    def process_odom(self, data):
        self._position = (data.pose.pose.position.x, data.pose.pose.position.y)

    def execute(self, ud):
        if self._position is None:
            rospy.loginfo("Waiting for position")
            return 'move_back'

        position = tuple(self._position)

        rospy.loginfo(self._position)
        if self._initialPosition is None:
            self._initialPosition = position

        distance = sqrt(pow(position[0] - self._initialPosition[0], 2) + pow(position[1] - self._initialPosition[1], 2))
        rospy.loginfo(distance)
        twist = Twist()

        print(f'**************************distance = {distance}')
        if distance < self._distance:
            twist.linear.x = -.005
            transition = "move_back"
        else:
            twist.linear.x = 0
            self._position = None
            self._initialPosition = None
            transition = "move_fwd"
        self._pub.publish(twist)
        self._rate.sleep()
        return transition


class Turn(smach.State):
    def __init__(self, coord_list, rate):
        smach.State.__init__(self,
                             outcomes=["move_fwd", "move_back", "do_stop", "do_turn"])
        self._coord_list = coord_list
        self._rate = rate
        self._pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._sub = rospy.Subscriber("/odom", Odometry, self.process_odom)
        self._init_yaw = None
        self._yaw = None
        self._start_yaw = None

    def get_target_yaw(self):
        global index
        pi = 3.14159

        print(f"index = {index}")
        x1 = self._coord_list[index][0]
        x2 = self._coord_list[index + 1][0]
        y1 = self._coord_list[index][1]
        y2 = self._coord_list[index + 1][1]

        delta_y = y2 - y1
        delta_x = x2 - x1

        print(f'x2 = {x2}, x1 = {x1}')
        print(f'y2 = {y2}, y1 = {y1}')


        if delta_y == 0:
            desired_yaw = math.acos(delta_x)
        elif delta_x == 0:
            desired_yaw = math.asin(delta_y)
        else:
            desired_yaw = math.atan2(delta_y, delta_x)

        # print(f'************************* {turn_distance}')

        return desired_yaw

    def process_odom(self, data):
        pose = data.pose.pose

        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self._yaw = yaw

        if self._init_yaw is None:
            self._init_yaw = yaw

    def execute(self, ud):
        global index
        pi = 3.14159

        if index + 1 >= len(self._coord_list):
            return 'do_stop'
        
        if self._yaw is None:
            rospy.loginfo("Waiting for yaw")
            return "do_turn"

        if self._start_yaw is None:
            self._start_yaw = self._yaw

        if self._init_yaw > 0:
            current_yaw = self._yaw - self._init_yaw
        elif self._init_yaw < 0:
            current_yaw = self._yaw + self._init_yaw

        goal_yaw = self.get_target_yaw()

        twist = Twist()

        # Ensure that yaw is within -180 and 180
        if current_yaw < -pi:
            current_yaw = current_yaw + (pi * 2)
        elif current_yaw > pi:
            current_yaw = current_yaw - (pi * 2)

        print(f'goal yaw= {goal_yaw}', end=' ')
        print(f'current yaw = {current_yaw}')

        # If within plus or minus .005 radians, stop
        if -.005 < (goal_yaw - current_yaw) < .005:
            twist.angular.z = 0
            self._start_yaw = None
            if index == 0:
                return "move_fwd"
            transition = "move_back"

        else:
            twist.angular.z = max(abs(.5 * (goal_yaw - current_yaw)), .025) 
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


def main():
    rospy.init_node("move_robot_smach", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(10)
    img_height_width = 800

    # Square
    square_list = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]

    # Triangle
    triangle_list = [(0, 0), (1, 0), (.5, .5), (0, 0)]

    # Straight Line
    straight_line_read = open("StarTest.txt", "r")
    straight_line = eval(straight_line_read.read())

    # Convert origin from top left (image origin) to standard bottom left
    new_straight_line = []
    for x in straight_line:
        temp = x[0] / (800*2)
        temp2 = (img_height_width - x[1] + 1) / (800*2)
        temp_tuple = (temp, temp2)
        new_straight_line.append(temp_tuple)

    new_straight_line.insert(0, (0,0))

    sm = smach.StateMachine(outcomes=["exit_sm"])

    with sm:
        smach.StateMachine.add("TURN", Turn(new_straight_line, rate),
                                transitions={"move_fwd": "FORWARD",
                                             "do_turn": "TURN",
                                             "do_stop": "STOP",
                                             "move_back": "BACKWARD"
                                            })
        smach.StateMachine.add("FORWARD", Forward(new_straight_line, rate),
                               transitions={"move_fwd": "FORWARD",
                                            "do_turn": "TURN",
                                            "do_stop": "STOP"
                                            })
        smach.StateMachine.add("BACKWARD", Back(.0345, rate),
                               transitions={"move_back": "BACKWARD",
                                            "move_fwd": "FORWARD"
                                            })
        smach.StateMachine.add("STOP", Stop(),
                               transitions={"exit_sm": "exit_sm"
                                            })
    sm.execute()


if __name__ == '__main__':
    main()
