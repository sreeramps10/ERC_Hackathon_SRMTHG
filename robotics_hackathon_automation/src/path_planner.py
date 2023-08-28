import obstacle_detection
import rospy
import random
import math
from geometry_msgs.msg import PoseStamped, Point
from obstacle_detection import isValidPoint

class PathPlannerNode:
    def __init__(self):
        rospy.init_node('path_planner')
        self.planned_path_pub = rospy.Publisher('/planned_path', PoseStamped, queue_size=10)
        self.obstacles = [(1.0, 1.0), (2.0, 3.0), (-2.0, -1.0)]  
    def run(self):
        start_pose = PoseStamped()
        start_pose.header.stamp = rospy.Time.now()
        start_pose.header.frame_id = 'map'
        start_pose.pose.position.x = -5.06
        start_pose.pose.position.y = -3.12
        self.plan_path(start_pose)
        rospy.spin()

    def plan_path(self, start_pose):
        # RRT parameters
        max_iterations = 5000
        goal_radius = 0.5
        step_size = 0.2

        tree = {self.pose_to_tuple(start_pose): None}

        for iteration in range(max_iterations):
            if random.random() < 0.8:
                rand_point = Point()
                rand_point.x = random.uniform(-5.0, 5.0)
                rand_point.y = random.uniform(-3.5, 3.5)
            else:
                rand_point = Point()
                rand_point.x = 0.57
                rand_point.y = 0.33

            rand_tuple = (rand_point.x, rand_point.y)
            nearest_key = min(tree.keys(), key=lambda k: math.hypot(k[0] - rand_tuple[0], k[1] - rand_tuple[1]))

            dx = rand_tuple[0] - nearest_key[0]
            dy = rand_tuple[1] - nearest_key[1]
            distance = math.hypot(dx, dy)
            if distance > step_size:
                scale = step_size / distance
                new_key = (nearest_key[0] + dx * scale, nearest_key[1] + dy * scale)
            else:
                new_key = rand_tuple

            if self.is_valid_point(nearest_key[0], nearest_key[1], new_key[0], new_key[1]):
                tree[new_key] = nearest_key

                if math.hypot(new_key[0] - 0.57, new_key[1] - 0.33) < goal_radius:
                    self.publish_planned_path(tree, new_key)
                    break

    def is_valid_point(self, parentX, parentY, childX, childY):
        return isValidPoint(parentX, parentY, childX, childY)

    def pose_to_tuple(self, pose):
        return (pose.pose.position.x, pose.pose.position.y)
        
    def tuple_to_pose(self, tuple_data):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.position.x = tuple_data[0]
        pose.pose.position.y = tuple_data[1]
        return pose

    def publish_planned_path(self, tree, goal_key):
        path = [self.tuple_to_pose(goal_key)]
        while tree[goal_key] is not None:
            goal_key = tree[goal_key]
            path.append(self.tuple_to_pose(goal_key))

        path = path[::-1]
        
        for pose in path:
            self.planned_path_pub.publish(pose)
            rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        node = PathPlannerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
