"""Small helpers shared by the scripts that talk to Nav2."""
from geometry_msgs.msg import PoseStamped


def make_pose(navigator, x, y):
    """Pose at (x, y) in the map frame, facing +x, stamped with the current time."""
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose
