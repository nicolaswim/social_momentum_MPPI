import math
import rclpy.time
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
# Import specific TF exceptions
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Transform, TransformStamped

class TF2Wrapper:
    def __init__(self, node: Node) -> None:
        self._node = node
        self._tf_buffer = Buffer()
        # spin_thread=True can help process incoming TF messages more reliably
        # if the node's main thread is busy or doesn't spin frequently.
        self._tf_listener = TransformListener(self._tf_buffer, self._node, spin_thread=True)

        self._tf2_pub = self._node.create_publisher(TFMessage, "/tf", 1)

    def get_latest_pose(self, target_frame: str, source_frame: str) -> Transform | None:
        """
        Returns the transform T_target_source (which transforms a point from source_frame to target_frame).
        P_target = T_target_source * P_source
        """
        try:
            # A shorter timeout is crucial to prevent blocking the calling node's loop
            # for too long. If the planner's cycle is 0.05s, this timeout
            # should be significantly less.
            # If a TF is published every 0.1s, we're checking if it's available *now*.
            # This short timeout gives a little grace period for it to arrive.
            timeout_duration = Duration(seconds=0.03) # Reduced timeout (e.g., 0.02s to 0.04s)

            transform_stamped = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),  # Request the latest available transform
                timeout=timeout_duration,
            )
            return transform_stamped.transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # Log the specific TF error.
            # This logging can be very verbose if TFs are frequently missed, especially at startup.
            # Consider using throttle_duration_sec or making it a DEBUG level log.
            # The planner node already logs a WARNING when this returns None.
            self._node.get_logger().debug(
                f"TF lookup failed ({type(e).__name__}) for transform from '{source_frame}' to '{target_frame}'. Target time: latest. Error: {str(e)}"
            )
            return None
        except Exception as e: # Catch any other unexpected errors during TF lookup
            self._node.get_logger().error(
                f"An unexpected error (not TF-specific) occurred during TF lookup from '{source_frame}' to '{target_frame}': {type(e).__name__} - {str(e)}"
            )
            return None

    def publish_2d_pose(
        self, target_frame: str, source_frame: str, x: float, y: float, theta: float
    ) -> None:
        """
        Publishes a 2D pose as a TF transform.
        target_frame is the parent frame.
        source_frame is the child frame (this will be the frame name of the pose being published).
        """
        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.frame_id = target_frame
        transform_stamped_msg.header.stamp = self._node.get_clock().now().to_msg()
        transform_stamped_msg.child_frame_id = source_frame

        transform_stamped_msg.transform.translation.x = x
        transform_stamped_msg.transform.translation.y = y
        transform_stamped_msg.transform.translation.z = 0.0 # Explicitly set Z to 0 for 2D poses

        # Convert yaw (theta) to quaternion
        transform_stamped_msg.transform.rotation.x = 0.0
        transform_stamped_msg.transform.rotation.y = 0.0
        transform_stamped_msg.transform.rotation.z = math.sin(theta / 2.0)
        transform_stamped_msg.transform.rotation.w = math.cos(theta / 2.0)

        # TF messages are typically published as a list of transforms
        tf_message = TFMessage(transforms=[transform_stamped_msg])
        self._tf2_pub.publish(tf_message)