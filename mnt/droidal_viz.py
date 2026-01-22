import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

# Updated URDF for Droidal
URDF_CONTENT = """<?xml version="1.0"?>
<robot name="droidal">
  <material name="blue"><color rgba="0 0 0.8 1"/></material>
  <material name="black"><color rgba="0 0 0 1"/></material>
  <material name="grey"><color rgba="0.5 0.5 0.5 1"/></material>

  <link name="base_link">
    <visual>
      <origin xyz="0.02 0 0.215" rpy="0 0 0"/>
      <geometry><box size="0.15 0.43 0.43"/></geometry>
      <material name="blue"/>
    </visual>
  </link>

  <link name="rear_body">
    <visual>
      <origin xyz="0 0 0.215" rpy="0 0 0"/>
      <geometry><box size="0.30 0.20 0.43"/></geometry>
      <material name="blue"/>
    </visual>
  </link>

  <joint name="body_joint" type="fixed">
    <parent link="base_link"/>
    <child link="rear_body"/>
    <origin xyz="-0.225 0 0" rpy="0 0 0"/>
  </joint>

  <link name="left_wheel">
    <visual>
      <geometry><cylinder length="0.05" radius="0.095"/></geometry>
      <material name="black"/>
    </visual>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.215 0.025" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="right_wheel">
    <visual>
      <geometry><cylinder length="0.05" radius="0.095"/></geometry>
      <material name="black"/>
    </visual>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.215 0.025" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="caster">
    <visual>
      <geometry><sphere radius="0.05"/></geometry>
      <material name="grey"/>
    </visual>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="rear_body"/>
    <child link="caster"/>
    <origin xyz="-0.15 0 -0.02" rpy="0 0 0"/>
  </joint>

  <link name="laser_frame">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame"/>
    <origin xyz="-0.125 0 0.22" rpy="0 0 0"/>
  </joint>
</robot>
"""

def euler_to_quaternion(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy
    ]

class DroidalBridge(Node):
    def __init__(self):
        super().__init__('droidal_viz_bridge')
        
        # Publish URDF
        qos_profile = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, history=rclpy.qos.HistoryPolicy.KEEP_LAST)
        self.robot_desc_pub = self.create_publisher(String, 'robot_description', qos_profile)
        msg = String(); msg.data = URDF_CONTENT; self.robot_desc_pub.publish(msg)
        
        self.subscription = self.create_subscription(String, 'odrive_status', self.listener_callback, 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Odometry State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_p_right = None
        self.last_p_left = None
        
        # Physical Constants
        self.wheel_radius = 0.095 # 19cm diameter
        self.wheel_base = 0.43    # 43cm apart
        
        self.get_logger().info('Droidal Bridge: Odometry enabled. Robot will move in RViz!')

    def listener_callback(self, msg):
        try:
            parts = msg.data.split(' ')
            p_right = float(parts[0].split(':')[1]) # Motor 0
            p_left = float(parts[1].split(':')[1])  # Motor 1
            
            now = self.get_clock().now().to_msg()

            # 1. Update Odometry (x, y, theta)
            if self.last_p_right is not None:
                # Delta in rotations (turns)
                # We invert dp_right because the physical orientation/feedback is reversed
                dp_right = -(p_right - self.last_p_right)
                dp_left = (p_left - self.last_p_left)

                # Distance each wheel traveled (meters)
                d_right = dp_right * 2.0 * math.pi * self.wheel_radius
                d_left = dp_left * 2.0 * math.pi * self.wheel_radius

                # Average distance and change in angle
                d_center = (d_left + d_right) / 2.0
                d_theta = (d_right - d_left) / self.wheel_base

                # Update pose
                self.x += d_center * math.cos(self.th)
                self.y += d_center * math.sin(self.th)
                self.th += d_theta

            self.last_p_right = p_right
            self.last_p_left = p_left

            # 2. Broadcast Odom -> Base Link
            t_odom = TransformStamped()
            t_odom.header.stamp = now
            t_odom.header.frame_id = 'odom'
            t_odom.child_frame_id = 'base_link'
            t_odom.transform.translation.x = self.x
            t_odom.transform.translation.y = self.y
            t_odom.transform.translation.z = 0.07 # Ground clearance
            
            q_body = euler_to_quaternion(0, 0, self.th)
            t_odom.transform.rotation.x = q_body[0]
            t_odom.transform.rotation.y = q_body[1]
            t_odom.transform.rotation.z = q_body[2]
            t_odom.transform.rotation.w = q_body[3]

            # 3. Broadcast Static and Wheel TFs
            transforms = [t_odom]
            
            # Rear Body
            t_rear = TransformStamped()
            t_rear.header.stamp = now; t_rear.header.frame_id = 'base_link'; t_rear.child_frame_id = 'rear_body'
            t_rear.transform.translation.x = -0.225; t_rear.transform.rotation.w = 1.0
            transforms.append(t_rear)

            # Caster
            t_caster = TransformStamped()
            t_caster.header.stamp = now; t_caster.header.frame_id = 'rear_body'; t_caster.child_frame_id = 'caster'
            t_caster.transform.translation.x = -0.15; t_caster.transform.translation.z = -0.02; t_caster.transform.rotation.w = 1.0
            transforms.append(t_caster)

            # Wheels
            for i, name in enumerate(['right_wheel', 'left_wheel']):
                tw = TransformStamped()
                tw.header.stamp = now
                tw.header.frame_id = 'base_link'
                tw.child_frame_id = name
                tw.transform.translation.y = -0.215 if i == 0 else 0.215
                tw.transform.translation.z = 0.025
                
                # Flip the sign of the angle for the right wheel (i == 0) to match visual
                raw_pos = p_right if i == 0 else p_left
                angle = raw_pos * 2.0 * math.pi
                if i == 0:
                    angle = -angle
                
                q_wheel = euler_to_quaternion(-1.5708, angle, 0)
                tw.transform.rotation.x, tw.transform.rotation.y = q_wheel[0], q_wheel[1]
                tw.transform.rotation.z, tw.transform.rotation.w = q_wheel[2], q_wheel[3]
                transforms.append(tw)

            # 4. Broadcast Laser Frame (New)
            t_laser = TransformStamped()
            t_laser.header.stamp = now
            t_laser.header.frame_id = 'base_link'
            t_laser.child_frame_id = 'laser_frame'
            # Using the measurements we calculated: 22cm back from front, 22cm from floor
            t_laser.transform.translation.x = -0.125
            t_laser.transform.translation.y = 0.0
            t_laser.transform.translation.z = 0.22 
            t_laser.transform.rotation.w = 1.0  # No rotation relative to base_link
            transforms.append(t_laser)
            #self.tf_broadcaster.sendTransform(transforms)
            # New way: Give the transform a tiny bit of 'future' validity 
            # to help the message filter synchronize.
            future_now = self.get_clock().now() + rclpy.duration.Duration(seconds=0.1)
            for t in transforms:
                t.header.stamp = future_now.to_msg()

            self.tf_broadcaster.sendTransform(transforms)
            
        except Exception as e:
            self.get_logger().warn(f'Parsing error: {e}')

def main(args=None):
    rclpy.init(args=args); node = DroidalBridge(); rclpy.spin(node); rclpy.shutdown()

if __name__ == '__main__':
    main()
