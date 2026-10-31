import math
from types import SimpleNamespace as Node
import unittest

from scripts.r1_odometry_bridge import packet


class BridgeContract(unittest.TestCase):
    def setUp(self):
        self.message = Node(
            header=Node(frame_id="odom", stamp=Node(sec=1, nanosec=0)),
            child_frame_id="pelvis",
            pose=Node(
                pose=Node(
                    position=Node(x=0.0, y=0.0, z=0.7),
                    orientation=Node(w=1.0, x=0.0, y=0.0, z=0.0),
                )
            ),
            twist=Node(twist=Node(linear=Node(x=0.1, y=0.0, z=0.0))),
        )

    def convert(self, now=1010000000):
        return packet(self.message, now, 0.1, "odom", "pelvis")

    def test_preserves_pose_and_body_twist(self):
        data, stamp = self.convert()
        self.assertEqual(data["position_w"], [0.0, 0.0, 0.7])
        self.assertEqual(data["linear_velocity_b"], [0.1, 0.0, 0.0])
        self.assertEqual(stamp, 1000000000)

    def test_rejects_stale_and_future_source(self):
        for now in [1200000000, 900000000]:
            with self.assertRaises(ValueError):
                self.convert(now)

    def test_rejects_wrong_frames(self):
        self.message.child_frame_id = "camera"
        with self.assertRaises(ValueError):
            self.convert()

    def test_rejects_nonfinite_and_bad_rotation(self):
        self.message.twist.twist.linear.x = math.nan
        with self.assertRaises(ValueError):
            self.convert()
        self.message.twist.twist.linear.x = 0.0
        self.message.pose.pose.orientation.w = 0.0
        with self.assertRaises(ValueError):
            self.convert()


if __name__ == "__main__":
    unittest.main()
