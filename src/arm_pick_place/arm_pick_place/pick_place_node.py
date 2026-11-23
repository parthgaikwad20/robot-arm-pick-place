"""Pick-and-place demo node.

Moves the simulated arm through a pick-and-place cycle in straight Cartesian
lines (IK solved every control step), opens/closes the gripper, and publishes
the cube as an RViz marker that rides along with the gripper while grasped.

Topics
  /joint_states             sensor_msgs/JointState   -> robot_state_publisher
  /pick_place/markers       visualization_msgs/MarkerArray (cube + pads)
  /pick_place/status        std_msgs/String          (current phase)

Parameters
  pick_xyz   [x, y, z]  cube centre at pick location   (default 0.30, 0.10, 0.02)
  place_xyz  [x, y, z]  cube centre at place location  (default 0.10, -0.30, 0.02)
  approach_height  float  height above target for approach (default 0.13)
  speed            float  Cartesian speed in m/s          (default 0.12)
  loop             bool   swap pick/place and repeat       (default True)
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from arm_pick_place.kinematics import inverse, UnreachableError

ARM_JOINTS = ['joint1_base_yaw', 'joint2_shoulder', 'joint3_elbow', 'joint4_wrist']
FINGER_JOINTS = ['finger_left_joint', 'finger_right_joint']
GRIPPER_OPEN = 0.025
GRIPPER_CLOSED = 0.0
CUBE = 0.04
RATE_HZ = 50.0


def smoothstep(t):
    return t * t * (3 - 2 * t)


class PickPlace(Node):
    def __init__(self):
        super().__init__('pick_place')
        self.declare_parameter('pick_xyz', [0.30, 0.10, 0.02])
        self.declare_parameter('place_xyz', [0.10, -0.30, 0.02])
        self.declare_parameter('approach_height', 0.13)
        self.declare_parameter('speed', 0.12)
        self.declare_parameter('loop', True)

        self.pick = list(self.get_parameter('pick_xyz').value)
        self.place = list(self.get_parameter('place_xyz').value)
        self.approach = float(self.get_parameter('approach_height').value)
        self.speed = float(self.get_parameter('speed').value)
        self.loop = bool(self.get_parameter('loop').value)

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'pick_place/markers', 10)
        self.status_pub = self.create_publisher(String, 'pick_place/status', 10)

        self.home = [0.20, 0.0, 0.30]
        self.tool = list(self.home)        # current Cartesian tool position
        self.q = inverse(*self.tool)       # current arm joint angles
        self.grip = GRIPPER_OPEN
        self.cube_attached = False
        self.cube_pos = list(self.pick)

        self.steps = self.build_plan()
        self.step_idx = 0
        self.step_t = 0.0
        self.step_start = None
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)
        self.get_logger().info(
            f'Pick-and-place ready: pick={self.pick} place={self.place}')

    # ------------------------------------------------------------------ plan
    def build_plan(self):
        """A plan is a list of (phase name, kind, argument)."""
        above = lambda p: [p[0], p[1], p[2] + self.approach]
        return [
            ('move above pick', 'move', above(self.pick)),
            ('open gripper', 'grip', GRIPPER_OPEN),
            ('descend to cube', 'move', list(self.pick)),
            ('close gripper', 'grip', GRIPPER_CLOSED),
            ('attach cube', 'attach', True),
            ('lift cube', 'move', above(self.pick)),
            ('carry to place', 'move', above(self.place)),
            ('lower cube', 'move', list(self.place)),
            ('open gripper', 'grip', GRIPPER_OPEN),
            ('release cube', 'attach', False),
            ('retreat', 'move', above(self.place)),
            ('return home', 'move', list(self.home)),
            ('pause', 'wait', 1.0),
        ]

    # ------------------------------------------------------------------ loop
    def tick(self):
        if self.step_idx >= len(self.steps):
            if not self.loop:
                self.publish()
                return
            self.pick, self.place = self.place, self.pick  # put it back
            self.steps = self.build_plan()
            self.step_idx = 0

        name, kind, arg = self.steps[self.step_idx]
        dt = 1.0 / RATE_HZ

        if self.step_start is None:  # entering a new step
            self.step_start = (list(self.tool), self.grip)
            self.step_t = 0.0
            self.status_pub.publish(String(data=name))
            self.get_logger().info(f'[{self.step_idx + 1}/{len(self.steps)}] {name}')

        done = False
        if kind == 'move':
            start = self.step_start[0]
            dist = math.dist(start, arg)
            duration = max(dist / self.speed, 0.3)
            self.step_t += dt / duration
            s = smoothstep(min(self.step_t, 1.0))
            target = [a + (b - a) * s for a, b in zip(start, arg)]
            try:
                self.q = inverse(*target)
                self.tool = target
            except UnreachableError as e:
                self.get_logger().error(str(e))
                self.timer.cancel()
                return
            done = self.step_t >= 1.0
        elif kind == 'grip':
            self.step_t += dt / 0.6
            s = smoothstep(min(self.step_t, 1.0))
            self.grip = self.step_start[1] + (arg - self.step_start[1]) * s
            done = self.step_t >= 1.0
        elif kind == 'attach':
            self.cube_attached = arg
            if not arg:
                self.cube_pos = list(self.tool)
            done = True
        elif kind == 'wait':
            self.step_t += dt / arg
            done = self.step_t >= 1.0

        self.publish()
        if done:
            self.step_idx += 1
            self.step_start = None

    # --------------------------------------------------------------- output
    def publish(self):
        now = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = now
        js.name = ARM_JOINTS + FINGER_JOINTS
        js.position = list(self.q) + [self.grip, self.grip]
        self.joint_pub.publish(js)

        markers = MarkerArray()
        cube = Marker()
        cube.header.stamp = now
        cube.ns, cube.id, cube.type = 'cube', 0, Marker.CUBE
        if self.cube_attached:
            cube.header.frame_id = 'tool_link'  # rides with the gripper via TF
        else:
            cube.header.frame_id = 'base_link'
            cube.pose.position.x, cube.pose.position.y, cube.pose.position.z = self.cube_pos
        cube.pose.orientation.w = 1.0
        cube.scale.x = cube.scale.y = cube.scale.z = CUBE
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = 0.1, 0.6, 1.0, 1.0
        markers.markers.append(cube)

        for i, (p, rgb) in enumerate([(self.pick, (0.2, 0.8, 0.3)),
                                      (self.place, (0.9, 0.2, 0.2))]):
            pad = Marker()
            pad.header.stamp = now
            pad.header.frame_id = 'base_link'
            pad.ns, pad.id, pad.type = 'pads', i, Marker.CYLINDER
            pad.pose.position.x, pad.pose.position.y = p[0], p[1]
            pad.pose.position.z = 0.001
            pad.pose.orientation.w = 1.0
            pad.scale.x = pad.scale.y = 0.08
            pad.scale.z = 0.002
            pad.color.r, pad.color.g, pad.color.b = rgb
            pad.color.a = 0.6
            markers.markers.append(pad)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = PickPlace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
