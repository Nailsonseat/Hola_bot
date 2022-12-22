#!/usr/bin/env python3

from turtle import st
import rospy

# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry

# for finding sin() cos()
import math

# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion


from geometry_msgs.msg import PoseArray


pi = math.pi
hola_x = 0
hola_y = 0
roll = 0
pitch = 0
yaw = 0
pub = 0

x_goals = [1, -1, -1, 1, 0]
y_goals = [1, 1, -1, -1, 0]
theta_goals = [pi/4, 3*pi/4, -3*pi/4, -pi/4, 0]


def task1_goals_Cb(msg):
    global x_goals, y_goals, theta_goals

    x_goals.clear()
    y_goals.clear()
    theta_goals.clear()

    for waypoint_pose in msg.poses:
        x_goals.append(waypoint_pose.position.x)
        y_goals.append(waypoint_pose.position.y)

        orientation_q = waypoint_pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        theta_goal = euler_from_quaternion(orientation_list)[2]
        theta_goals.append(theta_goal)


def odometryCb(msg):
    global hola_x, hola_y, roll, pitch, yaw
    hola_x = msg.pose.pose.position.x
    hola_y = msg.pose.pose.position.y

    orientation = msg.pose.pose.orientation
    orientation_l = [orientation.x, orientation.y,
                     orientation.z, orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_l)
    # Write your code to take the msg and update the three variables


def main():
    global pub
    global x_goals
    global y_goals
    global theta_goals
    # Initialze Node
    # We'll leave this for you to figure out the syntax for
    # initialising node named "controller"
    rospy.init_node("controller", anonymous=True)

    # Initialze Publisher and Subscriber
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odometryCb)
    #rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)
    # We'll leave this for you to figure out the syntax for
    # initialising publisher and subscriber of cmd_vel and odom respectively

    # Declare a Twist message
    vel = Twist()
    # Initialise the required variables to 0
    vel.linear.x = 0
    vel.linear.y = 0
    # <This is explained below>

    # For maintaining control loop rate.
    rate = rospy.Rate(100)

    # Initialise variables that may be needed for the control loop
    # For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
    # and also Kp values for the P Controller
    #
    #
    # Control Loop goes here
    index = 0
    while not rospy.is_shutdown():

        # Find error (in x, y and theta) in global frame
        # the /odom topic is giving pose of the robot in global frame
        # the desired pose is declared above and defined by you in global frame
        # therefore calculate error in global frame\
        print('x : '+str(hola_x))
        print('y : '+str(hola_y))
        print('O : '+str(yaw))

        vel_x = 0
        vel_y = 0
        vel_z = 0

        moveto(x_goals[index], y_goals[index], theta_goals[index], index)
        index += 1

        #rotate(0.7, 5, 0)
        print('x : '+str(hola_x))
        print('y : '+str(hola_y))
        print('O : '+str(yaw))
        # (Calculate error in body frame)
        # But for Controller outputs robot velocity in robot_body frame,
        # i.e. velocity are define is in x, y of the robot frame,
        # Notice: the direction of z axis says the same in global and body frame
        # therefore the errors will have have to be calculated in body frame.
        #
        # This is probably the crux of Task 1, figure this out and rest should be fine.

        # Finally implement a P controller
        # to react to the error with velocities in x, y and theta.

        # Safety Check
        # make sure the velocities are within a range.
        # for now since we are in a simulator and we are not dealing with actual physical limits on the system
        # we may get away with skipping this step. But it will be very necessary in the long run.

        vel.linear.x = vel_x
        vel.linear.y = vel_y
        vel.angular.z = vel_z

        pub.publish(vel)
        rate.sleep()
        if index == len(x_goals):
            break
    #
    #


def rotatedCoordinates(x, y, i):
    global yaw
    x_t = x
    y_t = y
    ret = [x*math.cos(yaw)+y*math.sin(yaw), -x*math.sin(yaw)+y*math.cos(yaw)]
    print('coord to modify '+str(x)+'and'+str(y))
    print('x at end '+str(ret[0]))
    print('y at end '+str(ret[1]))
    return ret


def moveto(x, y, angle, i):
    global pub
    global theta_goals
    global x_goals
    global y_goals

    x_start, y_start = rotatedCoordinates(x_goals[i-1], y_goals[i-1], i)

    x, y = rotatedCoordinates(x, y, i)

    clockwise = False
    """ x = x_t*math.cos(yaw/2)+y_t*math.sin(yaw/2)
    y = -x_t*math.sin(yaw/2)+y_t*math.cos(yaw/2) """

    vel_msg = Twist()

    kp = 3
    #target = angle*math.pi/180
    target = abs(angle)

    if angle >= 0:
        clockwise = True

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0

    distance_final = abs(math.sqrt(((x_start-x)**2)+((y_start-y)**2)))

    vel_msg.linear.x = x/distance_final+3

    vel_msg.linear.y = y/distance_final+3

    if round(abs(hola_x-x)) == 0:
        vel_msg.linear.x = 0
    if round(abs(hola_y-y)) == 0:
        vel_msg.linear.y = 0

    """ if x > hola_x and y > hola_y:
        if vel_msg.linear.x < 0:
            vel_msg.linear.x *= -1
        if vel_msg.linear.y < 0:
            vel_msg.linear.y *= -1
    if x < hola_x and y > hola_y:
        if vel_msg.linear.x > 0:
            vel_msg.linear.x *= -1
        if vel_msg.linear.y < 0:
            vel_msg.linear.y *= -1
    if x < hola_x and y < hola_y:
        if vel_msg.linear.x > 0:
            vel_msg.linear.x *= -1
        if vel_msg.linear.y > 0:
            vel_msg.linear.y *= -1
    if x > hola_x and y < hola_y:
        if vel_msg.linear.x < 0:
            vel_msg.linear.x *= -1
        if vel_msg.linear.y > 0:
            vel_msg.linear.y *= -1 """

    if x > round(hola_x):
        if vel_msg.linear.x < 0:
            vel_msg.linear.x *= -1
    else:
        if vel_msg.linear.x > 0:
            vel_msg.linear.x *= -1

    if y > round(hola_y):
        if vel_msg.linear.y < 0:
            vel_msg.linear.y *= -1
    else:
        if vel_msg.linear.y > 0:
            vel_msg.linear.y *= -1

    """ v_x = vel_msg.linear.x
    v_y = vel_msg.linear.y

    dist = 0 """
    loop_rate = rospy.Rate(100)

    reached = False

    while True:
        pub.publish(vel_msg)
        loop_rate.sleep()

        dist = abs(math.sqrt(((rotatedCoordinates(hola_x, hola_y, i)[
                   0]-x_start)**2)+((rotatedCoordinates(hola_x, hola_y, i)[1]-y_start)**2)))

        if (dist > distance_final):
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            reached = True

        print("Yaw right now : " + str(yaw*180/math.pi))
        if target > 0 and abs(yaw) > abs(target-0.05) and reached:
            vel_msg.angular.z = 0
            """ elif target < 0 and abs(yaw) > target+0.05 and reached:
            vel_msg.angular.z = 0 """
        elif reached:
            vel_msg.angular.z = kp+(target-yaw)
            if clockwise:
                vel_msg.angular.z *= -1

        if vel_msg.linear.x == 0 and vel_msg.linear.y == 0 and vel_msg.angular.z == 0:
            break

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    pub.publish(vel_msg)


""" def rotate(ang_speed, angle, clockwise):
    global pub
    vel_msg = Twist()

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0

    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    if clockwise:
        vel_msg.angular.z = -abs(ang_speed)
    else:
        vel_msg.angular.z = abs(ang_speed)

    current_angle = 0
    tolerance = 0.21

    t0 = rospy.Time.now().to_sec()

    first = 0

    loop_rate = rospy.Rate(10)

    while current_angle+tolerance < angle or not first:
        rospy.loginfo(hola_theta)
        first = 1
        loop_rate.sleep()
        pub.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = ang_speed*(t1-t0)

    vel_msg.angular.z = 0
    pub.publish(vel_msg) """


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
