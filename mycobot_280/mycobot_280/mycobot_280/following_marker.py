import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker
from tf2_ros.buffer import Buffer
from tf2_ros import Duration
from tf2_ros import TransformException

class Talker(Node):
    def __init__(self):
        super().__init__("following_marker")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_marker = self.create_publisher(
            msg_type=Marker,
            topic="visualization_marker",
            qos_profile=10,
        )

        self.get_logger().info("Following marker started...")


    def timer_callback(self):

        if rclpy.ok():
            marker_ = Marker()
            marker_.header.frame_id = "/joint1"
            marker_.ns = "basic_cube"

            # now = rclpy.time.Time()
            now = self.get_clock().now()
            # self.get_logger().info("now origin: " + str(now.seconds_nanoseconds()))
            # now = now - Duration(seconds=3.0)
            # self.get_logger().info("now proces: " + str(now.seconds_nanoseconds()))

            try:
                self.get_logger().info("trying lookup_transform...")
                trans = self.tf_buffer.lookup_transform(
                    "joint1",
                    "basic_shapes",
                    now,
                    rclpy.duration.Duration(seconds=5.0)
                )

                # marker
                marker_.header.stamp = now.to_msg()
                marker_.type = marker_.CUBE
                marker_.action = marker_.ADD
                marker_.scale.x = 0.04
                marker_.scale.y = 0.04
                marker_.scale.z = 0.04

                # marker position initial
                marker_.pose.position.x = trans.transform.translation.x
                marker_.pose.position.y = trans.transform.translation.y
                marker_.pose.position.z = trans.transform.translation.z
                marker_.pose.orientation.x = trans.transform.rotation.x
                marker_.pose.orientation.y = trans.transform.rotation.y
                marker_.pose.orientation.z = trans.transform.rotation.z
                marker_.pose.orientation.w = trans.transform.rotation.w

                marker_.color.a = 1.0
                marker_.color.g = 1.0
                self.pub_marker.publish(marker_)

            except TransformException as e:
                self.get_logger().info("Error lookup_transform %s" % (e,))
                print(e)


def main():
    rclpy.init()
    talker = Talker()
    talker.timer_ = talker.create_timer(0.5, talker.timer_callback)

    rclpy.spin(talker)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
