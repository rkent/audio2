import rclpy
from rclpy.node import Node
import threading
from alsa_gain_msgs.msg import AlsaGain
from .mixer_thread import MixerThread

class AlsaGainNode(Node):

    def __init__(self):
        super().__init__('alsa_gain_node')

        self.publisher = self.create_publisher(AlsaGain, 'alsa_gain', 10)
        self.subscriber = self.create_subscription(
            AlsaGain,
            'alsa_gain_set',
            self.handle_set_gain,
            10
        )

        # Declare and read ROS2 parameters for mixer control and device
        self.declare_parameter('control', 'Master')
        self.declare_parameter('device', 'default')
        self.declare_parameter('publish_rate', 5.0)  # in seconds
        self.control = self.get_parameter('control').value
        self.device = self.get_parameter('device').value
        publish_rate = self.get_parameter('publish_rate').value

        self.get_logger().info(f"Using mixer control='{self.control}', device='{self.device}'")

        # guard condition used to trigger publish on changes
        self.publish_guard = self.create_guard_condition(self.publish_gain)
        # Initialize the ALSA Mixer object
        self.mixer_thread = MixerThread(self.control, self.device, self.publish_guard)
        self.thread = threading.Thread(target=self.mixer_thread.run)
        self.thread.start()

        # Set up a timer to publish gain information at the specified rate
        self.timer = self.create_timer(publish_rate, self.publish_gain)

    def handle_set_gain(self, msg: AlsaGain):
        if not self.mixer_thread.ready:
            self.get_logger().warning("Mixer not ready yet, cannot set gain.")
            return
        percent = msg.percent
        muted = msg.muted
        if msg.control != self.control or msg.device != self.device:
            return
        if percent is None or muted is None:
            self.get_logger().warning("Invalid gain info received, ignoring set request.")
            return
        self.get_logger().info(f"Received Set Gain Request: percent: {percent}, muted: {muted}")
        # Only set changes to prevent excessive ALSA calls and ROS messages
        percent = [int(val) for val in percent]
        s_percent = self.mixer_thread.volume
        percent_changed = False
        if len(s_percent) == len(percent) and s_percent != percent:
            percent_changed = True
        elif len(percent) == 1 and any(s_percent[i] != percent[0] for i in range(len(s_percent))):
            percent_changed = True
        if percent_changed:
            self.mixer_thread.volume = percent
        s_muted = self.mixer_thread.muted
        muted_changed = False
        if s_muted != muted:
            muted_changed = True
        if muted_changed:
            self.get_logger().info(f"Setting muted to {muted} compared to {self.mixer_thread.muted}")
            self.mixer_thread.muted = muted

    def publish_gain(self):
        if not self.mixer_thread.ready:
            self.get_logger().warning("Mixer not ready yet, skipping publish.")
            return
        msg = AlsaGain()
        msg.control = self.control
        msg.device = self.device
        percent = self.mixer_thread.volume
        muted = self.mixer_thread.muted
        if None in [percent, muted]:
            self.get_logger().warning("Invalid gain info, skipping publish.")
            return
        msg.percent = percent
        msg.muted = muted
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published Gain: {msg.percent}%, Muted: {msg.muted}"
        )

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = AlsaGainNode()
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mixer_thread.close()
        node.thread.join()
