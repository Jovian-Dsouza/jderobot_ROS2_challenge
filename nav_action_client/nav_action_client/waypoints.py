import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from transforms3d.euler import euler2quat
from math import pi

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped

class TurtlebotActionClient(Node):

    def __init__(self):
        super().__init__('turtlebot_action_client')
        self._action_client = ActionClient(self, FollowWaypoints, '/FollowWaypoints')

    def send_goal(self, poses):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg,
                                                   feedback_callback=self.feedback_callback )

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('No of Missed Waypoints: {0}'.format(len(result.missed_waypoints)))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Current Waypoint : ' + str(feedback.current_waypoint))

    def createWaypoint(self, x, y , theta):
        p = PoseStamped()

        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'base_link'

        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = 0.0

        q = euler2quat(0,0, theta)
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        return p


def main(args=None):
    rclpy.init(args=args)

    action_client = TurtlebotActionClient()
    
    #Set all waypoints into a list
    waypoint_poses = []

    wp1 = action_client.createWaypoint(-0.4, 1.8 , 0)
    waypoint_poses.append(wp1)

    wp2 = action_client.createWaypoint(0.88, 0.5, pi/2)
    waypoint_poses.append(wp2)

    wp3 = action_client.createWaypoint(-0.3, -2.02, 0.0)
    waypoint_poses.append(wp3)

    # Call the action 
    action_client.send_goal(waypoint_poses)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()