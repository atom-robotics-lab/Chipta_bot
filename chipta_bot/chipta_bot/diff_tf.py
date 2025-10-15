import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Int32,Int64
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class DiffTF(Node):
    def __init__(self):
        super().__init__('diff_tf')
        self.get_logger().info('diff node is alive')

        self.create_subscription(Int64, "/fr_encoder_ticks", self.fr_wheel_callback, 10)
        self.create_subscription(Int64, "/fl_encoder_ticks", self.fl_wheel_callback, 10)
        self.create_subscription(Int64, "/rr_encoder_ticks", self.rr_wheel_callback, 10)
        self.create_subscription(Int64, "/rl_encoder_ticks", self.rl_wheel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.odom_broadcaster = TransformBroadcaster(self)

        self.rate_hz = self.declare_parameter("rate_hz", 10.0).value
        self.ticks_meter_fr = float(self.declare_parameter('ticks_meter_fr', 470).value)  
        self.ticks_meter_fl = float(self.declare_parameter('ticks_meter_fl', 470).value)  
        self.ticks_meter_rr = float(self.declare_parameter('ticks_meter_rr', 470).value)  
        self.ticks_meter_rl = float(self.declare_parameter('ticks_meter_rl', 470).value)

        self.encoder_min = int(self.declare_parameter('encoder_min', -9223372036854775808).value)
        self.encoder_max = int(self.declare_parameter('encoder_max',  9223372036854775807).value)
        self.encoder_low_wrap = self.declare_parameter('wheel_low_wrap', (
                self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min).value
        self.encoder_high_wrap = self.declare_parameter('wheel_high_wrap', (
                self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min).value

        self.frmult = 0.0
        self.curr_fr_enc = None
        self.prev_fr_enc = None
        self.flmult = 0.0
        self.curr_fl_enc = None
        self.prev_fl_enc = None
        self.rrmult = 0.0
        self.curr_rr_enc = None
        self.prev_rr_enc = None
        self.rlmult = 0.0
        self.curr_rl_enc = None
        self.prev_rl_enc = None

        self.create_timer(1.0 / self.rate_hz, self.update)

    def fr_wheel_callback(self,msg):
        enc = msg.data

        if self.prev_fr_enc is None:
            self.prev_fr_enc = enc
            self.curr_fr_enc = enc
            return
        
        #   self.get_logger().info(f"Left encoder ticks received: {enc}")
        if enc < self.encoder_low_wrap and self.prev_fr_enc > self.encoder_high_wrap:
            self.frmult = self.frmult + 1

        if enc > self.encoder_high_wrap and self.prev_fr_enc < self.encoder_low_wrap:
            self.frmult = self.frmult - 1

        self.curr_fr_enc = 1.0 * (enc + self.frmult * (self.encoder_max - self.encoder_min))
        self.prev_fr_enc = enc


    def fl_wheel_callback(self,msg):
        enc = msg.data

        if self.prev_fl_enc is None:
            self.prev_fl_enc = enc
            self.curr_fl_enc = enc
            return
        
        #   self.get_logger().info(f"Left encoder ticks received: {enc}")
        if enc < self.encoder_low_wrap and self.prev_fl_enc > self.encoder_high_wrap:
            self.flmult = self.flmult + 1

        if enc > self.encoder_high_wrap and self.prev_fl_enc < self.encoder_low_wrap:
            self.flmult = self.flmult - 1

        self.curr_fl_enc = 1.0 * (enc + self.flmult * (self.encoder_max - self.encoder_min))
        self.prev_fl_enc = enc

    def rr_wheel_callback(self,msg):
        enc = msg.data

        if self.prev_rr_enc is None:
            self.prev_rr_enc = enc
            self.curr_rr_enc = enc
            return
        
        #   self.get_logger().info(f"Left encoder ticks received: {enc}")
        if enc < self.encoder_low_wrap and self.prev_rr_enc > self.encoder_high_wrap:
            self.rrmult = self.rrmult + 1

        if enc > self.encoder_high_wrap and self.prev_rr_enc < self.encoder_low_wrap:
            self.rrmult = self.rrmult - 1

        self.curr_rr_enc = 1.0 * (enc + self.rrmult * (self.encoder_max - self.encoder_min))
        self.prev_rr_enc = enc

    def rl_wheel_callback(self,msg):
        enc = msg.data

        if self.prev_rl_enc is None:
            self.prev_rl_enc = enc
            self.curr_rl_enc = enc
            return
        
        #   self.get_logger().info(f"Left encoder ticks received: {enc}")
        if enc < self.encoder_low_wrap and self.prev_rl_enc > self.encoder_high_wrap:
            self.rlmult = self.rlmult + 1

        if enc > self.encoder_high_wrap and self.prev_rl_enc < self.encoder_low_wrap:
            self.rlmult = self.rlmult - 1

        self.curr_rl_enc = 1.0 * (enc + self.rlmult * (self.encoder_max - self.encoder_min))
        self.prev_rl_enc = enc

    def update(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = DiffTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
