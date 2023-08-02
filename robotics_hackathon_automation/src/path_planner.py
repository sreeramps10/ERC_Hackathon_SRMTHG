#!/usr/bin/env python3
#Following is the import line for importing the obstacle detection methods i.e. isValidPoint()
#you need to name this node as "path_planner"
import obstacle_detection
import rospy
import random
import math
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
from obstacle_detection import isValidPoint

class PathPlannerNode:
    def __init__(self):
        rospy.init_node('path_planner')
        self.planned_path_pub = rospy.Publisher('/planned_path', PoseStamped, queue_size=10)
        self.obstacles = [(1.0, 1.0), (2.0, 3.0), (-2.0, -1.0)]  # Example obstacle coordinates

    def run(self):
        # Start path planning from the robot's starting position
        start_pose = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='map'), pose=Point(x=-5.06, y=-3.12))
        self.plan_path(start_pose)
        rospy.spin()

    def plan_path(self, start_pose):
        # RRT parameters
        max_iterations = 5000
        goal_radius = 0.5
        step_size = 0.2

        # RRT data structures
        tree = {start_pose: None}

        for iteration in range(max_iterations):
            # Randomly sample a point in the workspace
            if random.random() < 0.8:
                rand_point = Point(x=random.uniform(-5.0, 5.0), y=random.uniform(-3.5, 3.5))
            else:
                # With a small probability, sample the goal directly
                rand_point = Point(x=0.57, y=0.33)

            # Find the nearest point in the tree
            nearest_pose = min(tree.keys(), key=lambda pose: math.hypot(pose.pose.position.x - rand_point.x, pose.pose.position.y - rand_point.y))

            # Extend the tree towards the sampled point
            dx = rand_point.x - nearest_pose.pose.position.x
            dy = rand_point.y - nearest_pose.pose.position.y
            distance = math.hypot(dx, dy)
            if distance > step_size:
                scale = step_size / distance
                new_pose = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='map'), pose=Point(x=nearest_pose.pose.position.x + dx * scale, y=nearest_pose.pose.position.y + dy * scale))
            else:
                new_pose = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='map'), pose=rand_point)

            # Check if the new point is valid (not in collision with obstacles)
            if self.is_valid_point(nearest_pose.pose.position.x, nearest_pose.pose.position.y, new_pose.pose.position.x, new_pose.pose.position.y):
                tree[new_pose] = nearest_pose

                # Check if the goal is reached
                if math.hypot(new_pose.pose.position.x - 0.57, new_pose.pose.position.y - 0.33) < goal_radius:
                    self.publish_planned_path(tree, new_pose)
                    break

    def is_valid_point(self, parentX, parentY, childX, childY):
        # Call the isValidPoint function from obstacle_detection.py
        return isValidPoint(parentX, parentY, childX, childY)

    def publish_planned_path(self, tree, goal_pose):
        # Backtrack the tree from the goal to the start to retrieve the planned path
        path = [goal_pose]
        while tree[goal_pose] is not None:
            goal_pose = tree[goal_pose]
            path.append(goal_pose)

        # Reverse the path to get it from start to goal
        path = path[::-1]

        # Publish the planned path
        for pose in path:
            self.planned_path_pub.publish(pose)
            rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        node = PathPlannerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
