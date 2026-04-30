import rclpy
import math
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

HALF_DISTANCE_BETWEEN_WHEELS = 0.165
WHEEL_RADIUS = 0.0975


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def quat_inverse(q):
    return [-q[0], -q[1], -q[2], q[3]]


def quat_rotate_vector(q, v):
    q_v = [v[0], v[1], v[2], 0.0]
    q_inv = quat_inverse(q)
    return quat_multiply(quat_multiply(q, q_v), q_inv)[:3]


class P3DXDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice("left wheel")
        self.__right_motor = self.__robot.getDevice("right wheel")

        self.__imu = self.__robot.getDevice("imu")
        self.__gps = self.__robot.getDevice("gps")
        self.__lidar = self.__robot.getDevice("Sick LMS 291")

        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.__imu.enable(self.__timestep)
        self.__gps.enable(self.__timestep)
        self.__lidar.enable(self.__timestep)

        self.__left_motor.setPosition(float("inf"))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float("inf"))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node("p3dxnode")
        self.__node.create_subscription(Twist, "cmd_vel", self.__cmd_vel_callback, 1)
        self.__odom_publisher = self.__node.create_publisher(Odometry, "odom", 10)
        self.__tf_broadcaster = TransformBroadcaster(self.__node)

        self.__initial_gps = None
        self.__initial_imu = None

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (
            forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS
        ) / WHEEL_RADIUS
        command_motor_right = (
            forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS
        ) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)

        # Publish Odometry
        gps_values = self.__gps.getValues()
        imu_quat = self.__imu.getQuaternion()

        if (
            gps_values
            and imu_quat
            and not math.isnan(gps_values[0])
            and not math.isnan(imu_quat[0])
        ):
            if self.__initial_gps is None:
                self.__initial_gps = list(gps_values)
                self.__initial_imu = list(imu_quat)

            p_diff = [
                gps_values[0] - self.__initial_gps[0],
                gps_values[1] - self.__initial_gps[1],
                gps_values[2] - self.__initial_gps[2],
            ]

            initial_imu_inv = quat_inverse(self.__initial_imu)
            p_odom = quat_rotate_vector(initial_imu_inv, p_diff)
            q_odom = quat_multiply(initial_imu_inv, imu_quat)

            now = self.__node.get_clock().now().to_msg()

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"

            odom.pose.pose.position.x = p_odom[0]
            odom.pose.pose.position.y = p_odom[1]
            odom.pose.pose.position.z = p_odom[2]

            # Both Webots and ROS quat are (x, y, z, w)
            odom.pose.pose.orientation.x = q_odom[0]
            odom.pose.pose.orientation.y = q_odom[1]
            odom.pose.pose.orientation.z = q_odom[2]
            odom.pose.pose.orientation.w = q_odom[3]

            self.__odom_publisher.publish(odom)

            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"

            t.transform.translation.x = p_odom[0]
            t.transform.translation.y = p_odom[1]
            t.transform.translation.z = p_odom[2]

            t.transform.rotation.x = q_odom[0]
            t.transform.rotation.y = q_odom[1]
            t.transform.rotation.z = q_odom[2]
            t.transform.rotation.w = q_odom[3]

            self.__tf_broadcaster.sendTransform(t)
