import rospy
import tf2_ros
import geometry_msgs.msg
import math
from geometry_msgs.msg import PoseStamped, Twist, Point

class ControllerNode:
    def __init__(self):
        rospy.init_node('tb3_controller')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pid_params = {'kp': 1.0, 'ki': 0.1, 'kd': 0.0}  
        self.target_tolerance = 0.2  
        self.integral = 0.0  
        self.prev_error = 0.0  

    def run(self):
        rospy.Subscriber('/planned_path', PoseStamped, self.planned_path_callback)
        rospy.spin()

    def planned_path_callback(self, msg):
        path_points = [(pose.position.x, pose.position.y) for pose in msg.poses]

        for target_point in path_points:
            current_point = self.get_current_position()
            distance = self.distance_to_target(current_point, target_point)
            angle = self.angle_to_target(current_point, target_point)
            linear_vel = self.pid_control(distance, 'kp') 
            angular_vel = self.pid_control(angle, 'kd') 
            self.send_velocity(linear_vel, angular_vel)
            if distance < self.target_tolerance:
                break
        self.send_velocity(0.0, 0.0)

    def pid_control(self, error, param_type):
        kp = self.pid_params['kp']
        ki = self.pid_params['ki']
        kd = self.pid_params['kd']

        proportional = kp * error
        self.integral += ki * error
        derivative = kd * (error - self.prev_error)
        pid_output = proportional + self.integral + derivative
        self.prev_error = error

        return pid_output

    def distance_to_target(self, current_point, target_point):
        dx = target_point[0] - current_point[0]
        dy = target_point[1] - current_point[1]
        return math.sqrt(dx**2 + dy**2)

    def angle_to_target(self, current_point, target_point):
        dx = target_point[0] - current_point[0]
        dy = target_point[1] - current_point[1]
        return math.atan2(dy, dx)

    def send_velocity(self, linear_vel, angular_vel):
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist_msg)

    def get_current_position(self):
        try:
            self.tf_buffer.can_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))
            current_position = (
                transform.transform.translation.x,
                transform.transform.translation.y
            )

            return current_position

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to get current position. Returning (0, 0) as a fallback.")
            return (0, 0)
            
if __name__ == '__main__':
    try:
        node = ControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
