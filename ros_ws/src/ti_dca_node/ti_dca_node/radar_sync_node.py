import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from radar_msgs.msg import RadarFrameRef

class RadarSyncNode(Node):
    def __init__(self):
        super().__init__("radar_sync_node")

        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_rect_raw")
        self.declare_parameter("radar_ref_topic", "/radar/frame_ref")
        self.declare_parameter("slop_sec", 0.20)
        self.declare_parameter("queue_size", 10)

        rgb_topic = self.get_parameter("rgb_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        radar_ref_topic = self.get_parameter("radar_ref_topic").value

        self.pub_pair = self.create_publisher(String, "/synced/pair", 50)

        self.rgb_sub = Subscriber(self, Image, rgb_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)
        self.radar_sub = Subscriber(self, RadarFrameRef, radar_ref_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.radar_sub],
            queue_size=int(self.get_parameter("queue_size").value),
            slop=float(self.get_parameter("slop_sec").value),
        )
        self.sync.registerCallback(self.cb)
        self._pair_count = 0

        self.get_logger().info(f"Syncing RGB={rgb_topic} DEPTH={depth_topic} RADAR={radar_ref_topic}")

    def _stamp_to_ns(self, stamp) -> int:
        return int(stamp.sec * 1_000_000_000 + stamp.nanosec)

    def cb(self, rgb: Image, depth: Image, radar_ref: RadarFrameRef):
        out = {
            "pair_id": self._pair_count,
            "rgb_stamp_ns": self._stamp_to_ns(rgb.header.stamp),
            "depth_stamp_ns": self._stamp_to_ns(depth.header.stamp),
            "radar_stamp_ns": self._stamp_to_ns(radar_ref.header.stamp),
            "radar_seq": radar_ref.seq,
            "radar_raw_bin": radar_ref.raw_bin,
            "radar_raw_bytes": radar_ref.raw_bytes,
            "radar_proc_npy": radar_ref.proc_npy,
            "radar_proc_info": radar_ref.proc_info_json,
        }
        msg = String()
        msg.data = json.dumps(out)
        self.pub_pair.publish(msg)
        self._pair_count += 1

def main():
    rclpy.init()
    node = RadarSyncNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()