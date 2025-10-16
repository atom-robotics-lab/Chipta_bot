import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class MecanumDummyEncoder(Node):
    def __init__(self):
        super().__init__('mecanum_dummy_encoder')
        self.get_logger().info("🚗 Mecanum Dummy Encoder Publisher Started (Extended Mode)")

        # Encoder publishers
        self.pub_fr = self.create_publisher(Int64, '/fr_encoder_ticks', 10)
        self.pub_fl = self.create_publisher(Int64, '/fl_encoder_ticks', 10)
        self.pub_rr = self.create_publisher(Int64, '/rr_encoder_ticks', 10)
        self.pub_rl = self.create_publisher(Int64, '/rl_encoder_ticks', 10)

        # Encoder state
        self.current_ticks = {"fr": 0, "fl": 0, "rr": 0, "rl": 0}

        # Motion and simulation parameters
        self.tick_rate = 20.0  # Hz
        self.tick_increment = 5
        self.motion_duration = 6.0  # seconds per motion
        self.motion_list = [
            "forward", "backward",
            "rotate_cw", "rotate_ccw",
            "strafe_right", "strafe_left",
            "diagonal_fr", "diagonal_fl"
        ]

        self.motion_index = 0
        self.step_count = 0
        self.steps_per_motion = int(self.motion_duration * self.tick_rate)

        # Timer for publishing
        self.create_timer(1.0 / self.tick_rate, self.publish_ticks)

    def publish_ticks(self):
        # Select current motion
        motion = self.motion_list[self.motion_index]

        # Simulate encoder behavior
        getattr(self, f"simulate_{motion}")()

        # Publish and log data
        self.publish_all()

        # Update step and motion cycle
        self.step_count += 1
        if self.step_count >= self.steps_per_motion:
            self.step_count = 0
            self.motion_index = (self.motion_index + 1) % len(self.motion_list)
            next_motion = self.motion_list[self.motion_index]
            self.get_logger().info(f"🔄 Switching motion to: {next_motion.upper()}")

    # ---------------- Motion Simulation Methods ----------------

    def simulate_forward(self):
        for k in self.current_ticks:
            self.current_ticks[k] += self.tick_increment

    def simulate_backward(self):
        for k in self.current_ticks:
            self.current_ticks[k] -= self.tick_increment

    def simulate_rotate_cw(self):
        self.current_ticks["fl"] += self.tick_increment
        self.current_ticks["rl"] += self.tick_increment
        self.current_ticks["fr"] -= self.tick_increment
        self.current_ticks["rr"] -= self.tick_increment

    def simulate_rotate_ccw(self):
        self.current_ticks["fl"] -= self.tick_increment
        self.current_ticks["rl"] -= self.tick_increment
        self.current_ticks["fr"] += self.tick_increment
        self.current_ticks["rr"] += self.tick_increment

    def simulate_strafe_right(self):
        # Mecanum right strafe: FR & RL forward, FL & RR backward
        self.current_ticks["fr"] += self.tick_increment
        self.current_ticks["rl"] += self.tick_increment
        self.current_ticks["fl"] -= self.tick_increment
        self.current_ticks["rr"] -= self.tick_increment

    def simulate_strafe_left(self):
        # Mecanum left strafe: FR & RL backward, FL & RR forward
        self.current_ticks["fr"] -= self.tick_increment
        self.current_ticks["rl"] -= self.tick_increment
        self.current_ticks["fl"] += self.tick_increment
        self.current_ticks["rr"] += self.tick_increment

    def simulate_diagonal_fr(self):
        # Move diagonally toward front-right
        self.current_ticks["fl"] += self.tick_increment
        self.current_ticks["rr"] += self.tick_increment
        # FR, RL stay steady

    def simulate_diagonal_fl(self):
        # Move diagonally toward front-left
        self.current_ticks["fr"] += self.tick_increment
        self.current_ticks["rl"] += self.tick_increment
        # FL, RR steady

    # ---------------- Helper ----------------

    def publish_all(self):
        msgs = {wheel: Int64() for wheel in self.current_ticks}
        for wheel, val in self.current_ticks.items():
            msgs[wheel].data = val

        self.pub_fr.publish(msgs["fr"])
        self.pub_fl.publish(msgs["fl"])
        self.pub_rr.publish(msgs["rr"])
        self.pub_rl.publish(msgs["rl"])

        self.get_logger().info(
            f"{self.motion_list[self.motion_index].upper()} | "
            f"FR:{msgs['fr'].data} FL:{msgs['fl'].data} RR:{msgs['rr'].data} RL:{msgs['rl'].data}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MecanumDummyEncoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Stopped Dummy Encoder Publisher.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
