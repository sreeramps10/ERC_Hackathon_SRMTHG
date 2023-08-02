#!/usr/bin/env python3
# Remember to change the coordinates recieved by the planner from (X, Y) to (X + 1.79, Y + 0.66).
#you need to name this node "controller"

import rospy
from geometry_msgs.msg import Twist, PoseStamped

class ControllerNode:
    def __init__(self):
        rospy.init_node('tb3_controller')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pid_params = {'kp': 1.0, 'ki': 0.1, 'kd': 0.0}  # PID controller parameters
        self.target_tolerance = 0.2  # Tolerance for reaching the target point

    def run(self):
        rospy.Subscriber('/planned_path', PoseStamped, self.planned_path_callback)
        rospy.spin()

    def planned_path_callback(self, msg):
        path_points = [(pose.position.x, pose.position.y) for pose in msg.poses]

        for target_point in path_points:
            current_point = self.get_current_position()  # Implement this function to get the current position of the robot

            # Compute the distance and angle to the target point
            distance = self.distance_to_target(current_point, target_point)
            angle = self.angle_to_target(current_point, target_point)

            # Use PID control to compute linear and angular velocities
            linear_vel = self.pid_control(distance, 'kp')  # Proportional control for linear velocity
            angular_vel = self.pid_control(angle, 'kd')  # Derivative control for angular velocity

            # Send velocity commands to the robot
            self.send_velocity(linear_vel, angular_vel)

            # Check if the target point is reached within tolerance
            if distance < self.target_tolerance:
                break

        # Stop the robot after reaching the last point
        self.send_velocity(0.0, 0.0)

    def pid_control(self, error, param_type):
        # Proportional-Integral-Derivative (PID) controller

        # Get the corresponding PID parameter value based on param_type (kp, ki, kd)
        kp = self.pid_params['kp']
        ki = self.pid_params['ki']
        kd = self.pid_params['kd']

        # Compute the control terms
        proportional = kp * error
        self.integral += ki * error
        derivative = kd * (error - self.prev_error)

        # Combine the control terms to get the PID control output
        pid_output = proportional + self.integral + derivative

        # Save the current error for the next iteration
        self.prev_error = error

        return pid_output

    def distance_to_target(self, current_point, target_point):
        # Compute the distance between two points
        dx = target_point[0] - current_point[0]
        dy = target_point[1] - current_point[1]
        return math.sqrt(dx**2 + dy**2)

    def angle_to_target(self, current_point, target_point):
        # Compute the angle (heading) to the target point
        dx = target_point[0] - current_point[0]
        dy = target_point[1] - current_point[1]
        return math.atan2(dy, dx)

    def send_velocity(self, linear_vel, angular_vel):
        # Create a Twist message and publish it to the cmd_vel topic
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        node = ControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
