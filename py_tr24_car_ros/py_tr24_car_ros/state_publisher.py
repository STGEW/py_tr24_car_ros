import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf_transformations import quaternion_from_euler

from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

from tr24_ros_interfaces.msg import Position2D


class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        # robot state
        self._x = 0.0
        self._y = 0.0
        self._phi = 0.0

        # Subscribe to position update
        self._sub_from_car_new_pos = self.create_subscription(
            Position2D, 'from_car_pos',
            self._from_car_new_pos_cb, 20)

    def _from_car_new_pos_cb(self, msg):
        self.get_logger().info(
            f'From car new pos; '
            f'x: {msg.x} y: {msg.y} phi: {msg.phi}')
        self._x = msg.x
        self._y = msg.y
        self._phi = msg.phi

        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'car_body'

        # update transform
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.transform.translation.x = self._x
        odom_trans.transform.translation.y = self._y
        odom_trans.transform.translation.z = 0.0

        # roll=0, pitch=0, yaw=phi
        quaternion = quaternion_from_euler(0.0, 0.0, self._phi) 
    
        # Set rotation (quaternion)
        odom_trans.transform.rotation.x = quaternion[0]
        odom_trans.transform.rotation.y = quaternion[1]
        odom_trans.transform.rotation.z = quaternion[2]
        odom_trans.transform.rotation.w = quaternion[3]

        self.broadcaster.sendTransform(odom_trans)

def main(args=None):

    rclpy.init(args=args)
    node = StatePublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
