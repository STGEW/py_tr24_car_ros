from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
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

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state
        tilt = 0.
        tinc = degree
        swivel = 0.
        angle = 0.
        height = 0.
        hinc = 0.005

        self._x = 0.0
        self._y = 0.0
        self._phi = 0.0

        # # message declarations
        # odom_trans = TransformStamped()
        # odom_trans.header.frame_id = 'odom'
        # # odom_trans.child_frame_id = 'axis'
        # odom_trans.child_frame_id = 'tank_body'
        
        # joint_state = JointState()

        # FROM CAR
        self._sub_from_car_new_pos = self.create_subscription(
            Position2D, 'from_car_pos',
            self._from_car_new_pos_cb, 20)

        # try:
        #     y_shift = 0.0
        #     while rclpy.ok():
        #         rclpy.spin_once(self)

        #         # update joint_state
        #         now = self.get_clock().now()
        #         joint_state.header.stamp = now.to_msg()
        #         # joint_state.name = ['swivel', 'tilt', 'periscope']
        #         # joint_state.position = [swivel, tilt, height]
        #         joint_state.name = ['left_track_joint', 'right_track_joint']
        #         joint_state.position = [left_track_joint, right_track_joint]
                

        #         # update transform
        #         # (moving in a circle with radius=2)
        #         odom_trans.header.stamp = now.to_msg()
        #         odom_trans.transform.translation.x = self._x
        #         odom_trans.transform.translation.y = self._y
        #         odom_trans.transform.translation.z = 0.0

        #         # y_shift += 0.01

        #         # if y_shift > 10.0:
        #         #     y_shift = 0.0
        #         # odom_trans.transform.translation.x = cos(angle)*2
        #         # odom_trans.transform.translation.y = sin(angle)*2
        #         # odom_trans.transform.translation.z = 0.7
        #         # odom_trans.transform.rotation = \
        #         #     euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw

        #         # send the joint state and transform
        #         self.joint_pub.publish(joint_state)
        #         self.broadcaster.sendTransform(odom_trans)

        #         # Create new robot state
        #         tilt += tinc
        #         if tilt < -0.5 or tilt > 0.0:
        #             tinc *= -1
        #         height += hinc
        #         if height > 0.2 or height < 0.0:
        #             hinc *= -1
        #         swivel += degree
        #         angle += degree/4

        #         # This will adjust as needed per iteration
        #         loop_rate.sleep()

        # except KeyboardInterrupt:
        #     pass

    def _from_car_new_pos_cb(self, msg):
        self.get_logger().info(
            f'From car new pos; '
            f'x: {msg.x} y: {msg.y} phi: {msg.phi}')
        self._x = msg.x
        self._y = msg.y
        self._phi = msg.phi

        left_track_joint = 0.0
        right_track_joint = 0.0

        now = self.get_clock().now()
        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        # joint_state.name = ['swivel', 'tilt', 'periscope']
        # joint_state.position = [swivel, tilt, height]
        joint_state.name = ['left_track_joint', 'right_track_joint']
        joint_state.position = [left_track_joint, right_track_joint]
        
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'tank_body'

        # update transform
        # (moving in a circle with radius=2)
        odom_trans.header.stamp = now.to_msg()
        odom_trans.transform.translation.x = self._x
        odom_trans.transform.translation.y = self._y
        odom_trans.transform.translation.z = 0.0

        # y_shift += 0.01

        # if y_shift > 10.0:
        #     y_shift = 0.0
        # odom_trans.transform.translation.x = cos(angle)*2
        # odom_trans.transform.translation.y = sin(angle)*2
        # odom_trans.transform.translation.z = 0.7
        # Convert yaw angle (phi) to quaternion
        quaternion = quaternion_from_euler(0.0, 0.0, self._phi)  # roll=0, pitch=0, yaw=phi
    
        # Set rotation (quaternion)
        odom_trans.transform.rotation.x = quaternion[0]
        odom_trans.transform.rotation.y = quaternion[1]
        odom_trans.transform.rotation.z = quaternion[2]
        odom_trans.transform.rotation.w = quaternion[3]

        # send the joint state and transform
        self.joint_pub.publish(joint_state)
        self.broadcaster.sendTransform(odom_trans)


def main(args=None):

    rclpy.init(args=args)
    node = StatePublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
