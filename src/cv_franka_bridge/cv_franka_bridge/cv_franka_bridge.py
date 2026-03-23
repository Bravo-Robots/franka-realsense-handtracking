"""
Interpret gestures from the user, and convert waypoints into robot motion.

This node interprets four gestures from a human user: thumbs up, thumbs down,
closed fist, and open palm. These gestures are used to control whether or not
the robot tracks the position of the users hand and to control the gripper.
Waypoints received are transformed into the robot base link's frame. Two
PD loops, one for position and one for orientation, are used to control the robot.

SUBSCRIBERS:
  + /waypoint (PoseStamped) - The 3D location of the hand's pose.
  + /right_gesture (String) - The gesture that the right hand is making.
PUBLISHERS:
  + /text_marker (Marker) - The text marker that is published to the RViz.
SERVICE CLIENTS:
  + /robot_waypoints (PlanPath) - The service that plans and executes the robot's
    motion.
ACTION CLIENTS:
  + /fr3_gripper/homing (Homing) - The action server that homes the gripper.
  + /fr3_gripper/grasp (Grasp) - The action server that controls the gripper.

"""
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

from visualization_msgs.msg import Marker

from std_srvs.srv import Empty
from std_msgs.msg import String

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import PoseStamped
import tf2_ros
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from franka_msgs.action import Homing, Grasp

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


import numpy as np

class CvFrankaBridge(Node):

    def __init__(self):
        super().__init__('cv_franka_bridge')

        # get parameters
        self.x_limits = [-0.6, 0.6]
        self.y_limits = [-0.7, 0.7]
        self.z_limits = [-0.7, 0.7]
        self.armed = False  # “Thumb_Up” activó el control (latched)
        
        # create callback groups
        self.waypoint_callback_group = MutuallyExclusiveCallbackGroup()
        self.gesture_callback_group = MutuallyExclusiveCallbackGroup()

        # create subscribers
        self.waypoint_subscriber = self.create_subscription(PoseStamped, 'waypoint', self.waypoint_callback, 10, callback_group=self.waypoint_callback_group)
        self.right_gesture_subscriber = self.create_subscription(String, 'right_gesture', self.right_gesture_callback, 10, callback_group=self.gesture_callback_group)
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 50)
        
        # create publishers
        self.traj_pub = self.create_publisher(JointTrajectory, "/fr3_arm_controller/joint_trajectory", 10)
        self.gripper_joint_pub = self.create_publisher(JointState, "/fr3_gripper/joint_states", 10)
        self.gripper_joint_names = ["fr3_finger_joint1", "fr3_finger_joint2"]

        # create clients
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.ik_client.wait_for_service(timeout_sec=0.1)

        self._ik_pending = False
        self._last_valid_arm_q: list[float] | None = None

        # create timer
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.tracking = False
        self.hand_origin_ps: PoseStamped | None = None
        self.ee_origin: Pose | None = None
        self.initialize = None

        # waypoint guardado como PoseStamped (para poder transformarlo bien)
        self.current_waypoint_ps: PoseStamped | None = None

        # --- Gripper gesture debounce ---
        self.gripper_stable_frames = 1

        self._g_stable_count = 0
        self._g_last = None

        # --- PID para suavizar (por eje) ---
        self.pid_kp = float(12.0)
        self.pid_ki = float(0.05)
        self.pid_kd = float(0.0001)

        # Kp = 10.0    Ki = 0.001   Kd = 0.0005

       # límites de paso por ciclo (suavizado)
        self.hand_scale = 0.003

        self.lower_distance_threshold = 4.0
        self.upper_distance_threshold = 450.0

        self.pid_i_limit = float(1.0)
        self.pid_step_limit = float(self.hand_scale)

        # estados PID por eje
        self._i = np.zeros(3, dtype=float)     # integral [x,y,z]
        self._prev_e = np.zeros(3, dtype=float)
        self._prev_t = self.get_clock().now()

        # create action clients
        self.gripper_homing_client = ActionClient(
                self, Homing, 'franka_gripper/homing')
        self.gripper_grasping_client = ActionClient(
                self, Grasp, 'franka_gripper/grasp')

        self.gripper_grasping_client.wait_for_server(timeout_sec=1.0)
        # with a fake gripper, the homing server will not be created
        if not self.gripper_homing_client.wait_for_server(
                timeout_sec=1):
            self.gripper_ready = False
            self.gripper_homed = True

        # create tf buffer and listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # create class variables
        self.gripper_ready = True
        self.gripper_status = "Open"
        self.gripper_homed = False
        self.gripper_force_control = False
        self.gripper_force = 0.001
        self.max_gripper_force = 10.0
        self.traj_time_from_start = 0.10
        self.arm_joint_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3",
            "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"
        ]

        self.current_waypoint = None
        self.previous_waypoint = None
        self.offset = None
        self.initial_ee_pose = self.get_ee_pose()
        self.desired_ee_pose = self.initial_ee_pose
        self.waypoints = []
        self.prev_gesture = None
        self.start_time = self.get_clock().now()

    def publish_gripper_width(self, width: float):
        """
        Publica apertura de garra como JointState en /fr3_gripper/joint_states.

        width = apertura total entre dedos.
        Cada dedo recibe la mitad.
        """
        half_width = float(width)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.gripper_joint_names)
        msg.position = [half_width, half_width]
        msg.velocity = [0.2, 0.2]
        msg.effort = []

        self.gripper_joint_pub.publish(msg)


    def get_transform(self, target_frame, source_frame):
        """Get the transform between two frames."""
        try:
            trans = self.buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            translation = trans.transform.translation
            rotation = trans.transform.rotation
            return translation, rotation
        except Exception as e:
            self.get_logger().warn(f"TF error {target_frame}<-{source_frame}: {e}")
            return None

    def get_ee_pose(self):
        out = self.get_transform("fr3_link0", "fr3_hand_tcp")
        if out is None:
            return None
        ee_home_pos, ee_home_rot = out
        ee_pose = Pose()
        ee_pose.position.x = ee_home_pos.x
        ee_pose.position.y = ee_home_pos.y
        ee_pose.position.z = ee_home_pos.z
        ee_pose.orientation.x = ee_home_rot.x
        ee_pose.orientation.y = ee_home_rot.y
        ee_pose.orientation.z = ee_home_rot.z
        ee_pose.orientation.w = ee_home_rot.w
        return ee_pose

    def waypoint_callback(self, msg):

        if self.current_waypoint_ps is None:
            self.current_waypoint_ps = msg
            return

        # distancia XYZ (en las unidades del waypoint; en tu caso parece mm)
        dx = msg.pose.position.x - self.current_waypoint_ps.pose.position.x
        dy = msg.pose.position.y - self.current_waypoint_ps.pose.position.y
        dz = msg.pose.position.z - self.current_waypoint_ps.pose.position.z
        distance = float(np.linalg.norm([dx, dy]))

        # filtra jitter muy pequeño y saltos enormes
        if distance < self.lower_distance_threshold or distance > self.upper_distance_threshold:
           # self.get_logger().info(f"Descarto waypoint por salto: d={round(dx, 3), round(dy, 3), round(dz, 3)}, distance: {round(distance, 3)}")
           return

        self.current_waypoint_ps = msg

    def _waypoint_in_base(self) -> PoseStamped | None:
        """Devuelve el waypoint transformado a fr3_link0 si es posible."""
        if self.current_waypoint_ps is None:
            return None

        ps = self.current_waypoint_ps

        # si ya viene en base, perfecto
        if ps.header.frame_id == "" or ps.header.frame_id == "fr3_link0":
            out = PoseStamped()
            out.header = ps.header
            out.header.frame_id = "fr3_link0"
            out.pose = ps.pose
            return out

        try:
            # tf2_geometry_msgs PoseStamped soporta buffer.transform
            return self.buffer.transform(ps, "fr3_link0", timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().info(f"No puedo transformar waypoint a fr3_link0: {e}")
            return None

    def right_gesture_callback(self, msg: String):
        g = msg.data

        # --- Debounce de gesto para garra ---
        if g == self._g_last:
            self._g_stable_count += 1
        else:
            self._g_last = g
            self._g_stable_count = 1

        # STOP duro: Thumb_Down corta y DESARMA (no vuelve a moverse con otros gestos)
        if g == "Thumb_Down":
            self.armed = False
            self.tracking = False
            
            self.initialize = None
            self.hand_origin_ps = None
            self.ee_origin = None
            
            # reset PID para que no haya tirón al rearmar
            if hasattr(self, "_i"):
                self._i[:] = 0.0
                self._prev_e[:] = 0.0
                self._prev_t = self.get_clock().now()

            self.prev_gesture = g
            return  # importante: no sigas procesando nada más en este callback

        # Armado en borde de subida de Thumb_Up
        if g == "Thumb_Up" and self.prev_gesture != "Thumb_Up":
            self.armed = True
            self.tracking = True

            ee = self.get_ee_pose()
            if ee is not None:
                self.ee_origin = ee
                self.desired_ee_pose = Pose()
                self.desired_ee_pose.position = ee.position
                self.desired_ee_pose.orientation = ee.orientation
                self.hand_origin_ps = self._waypoint_in_base()
                return

            # reset PID al rearmar
            if hasattr(self, "_i"):
                self._i[:] = 0.0
                self._prev_e[:] = 0.0
                self._prev_t = self.get_clock().now()


        # Si está armado, sigue trackeando (aunque el gesto sea Closed_Fist/Open_Palm/None)
        self.tracking = bool(self.armed)

        stable_ok = self._g_stable_count >= self.gripper_stable_frames

        if stable_ok and g == "Closed_Fist" and self.gripper_status != "Closed":
            self.gripper_status = "Closed"
            self.gripper_force_control = False
            self.gripper_force = 0.01

            self.publish_gripper_width(0.01)

            grasp_goal = Grasp.Goal()
            grasp_goal.width = 0.01
            grasp_goal.speed = 0.1
            grasp_goal.epsilon.inner = 0.05
            grasp_goal.epsilon.outer = 0.05
            grasp_goal.force = self.gripper_force

            future = self.gripper_grasping_client.send_goal_async(
                grasp_goal, feedback_callback=self.feedback_callback
            )
            future.add_done_callback(self.grasp_response_callback)
            self._g_stable_count = 0  # resetea para que no repita

        elif stable_ok and g == "Open_Palm" and self.gripper_status != "Open":
            self.gripper_status = "Open"
            self.gripper_force_control = False
            self.gripper_force = 3.0

            self.publish_gripper_width(0.06)

            grasp_goal = Grasp.Goal()
            grasp_goal.width = 0.075
            grasp_goal.speed = 0.2
            grasp_goal.epsilon.inner = 0.001
            grasp_goal.epsilon.outer = 0.001
            grasp_goal.force = self.gripper_force

            future = self.gripper_grasping_client.send_goal_async(
                grasp_goal, feedback_callback=self.feedback_callback
            )
            future.add_done_callback(self.grasp_response_callback)

            self._g_stable_count = 0

        self.prev_gesture = g

    def grasp_response_callback(self, future):
        """Callback for the grasp response."""
        try:
            goal_handle = future.result()
            self.get_logger().info('Goal accepted :)')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)

        except Exception: 
            self.get_logger().info('Goal rejected :(')
            return

    def get_result_callback(self, future):
        """Callback for the grasp result."""

        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        self.gripper_ready = True

    def feedback_callback(self, feedback):
        """Callback for the feedback from the gripper action server."""
        self.get_logger().info(f"Feedback: {feedback}")

    async def home_gripper(self):
        """Home the gripper."""

        await self.gripper_homing_client.send_goal_async(Homing.Goal(), feedback_callback=self.feedback_callback)
        self.gripper_homed = True

    def _joint_state_cb(self, msg: JointState) -> None:
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        try:
            self._last_valid_arm_q = [float(name_to_pos[j]) for j in self.arm_joint_names]
        except Exception:
            pass

    def _send_ik_request(self, target_pose: PoseStamped) -> None:
        if self._ik_pending:
            self.get_logger().info("IK request pending, skipping new request")
            return
        if not self.ik_client.service_is_ready():
            self.get_logger().info("IK service not ready")
            return
        if self._last_valid_arm_q is None:
            self.get_logger().info("No valid arm joint state available")
            return

        req = GetPositionIK.Request()

        pik = PositionIKRequest()
        pik.group_name = "fr3_arm"
        pik.ik_link_name = "fr3_hand_tcp"
        pik.pose_stamped = target_pose
        pik.avoid_collisions = True
        pik.timeout.sec = 0
        pik.timeout.nanosec = int(0.05 * 1e9)

        rs = RobotState()
        rs.joint_state.name = list(self.arm_joint_names)
        rs.joint_state.position = list(self._last_valid_arm_q)
        pik.robot_state = rs

        req.ik_request = pik

        self._ik_pending = True
        fut = self.ik_client.call_async(req)
        fut.add_done_callback(self._on_ik_done)

    def _on_ik_done(self, future) -> None:
        self._ik_pending = False
        try:
            resp = future.result()
        except Exception:
            return

        if resp.error_code.val != 1:
            return

        sol = resp.solution.joint_state
        name_to_pos = {n: p for n, p in zip(sol.name, sol.position)}
        try:
            q_solution = [float(name_to_pos[j]) for j in self.arm_joint_names]
        except Exception:
            return

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(self.arm_joint_names)
        
        pt = JointTrajectoryPoint()
        pt.positions = list(q_solution)
        t = float(self.traj_time_from_start)
        pt.time_from_start.sec = int(t)
        pt.time_from_start.nanosec = int((t - int(t)) * 1e9)

        traj.points = [pt]
        self.traj_pub.publish(traj)

    def _pid_step(self, e: np.ndarray, dt: float) -> np.ndarray:
        if dt <= 0.0:
            dt = 0.01

        de = (e - self._prev_e) / dt

        i_candidate = self._i + e * dt
        i_candidate = np.clip(i_candidate, -self.pid_i_limit, self.pid_i_limit)

        u = self.pid_kp * e + self.pid_ki * i_candidate + self.pid_kd * de
        step = u * dt
        step_sat = np.clip(step, -self.pid_step_limit, self.pid_step_limit)

        saturated = (step != step_sat)
        if np.any(saturated):
            i_new = self._i.copy()
            for k in range(3):
                if not saturated[k]:
                    i_new[k] = i_candidate[k]
            self._i = i_new
        else:
            self._i = i_candidate

        self._prev_e = e.copy()

        return step_sat
    
    def timer_callback(self):
        if not self.tracking or not self.armed:
            return

        if self.ee_origin is None or self.hand_origin_ps is None:
            return

        wp_base = self._waypoint_in_base()
        if wp_base is None:
            return

        ee_pose = self.get_ee_pose()
        if ee_pose is None:   # si implementas get_ee_pose() -> None cuando TF falla
            return

        self._g_stable_count = 0
        self._g_last = None

        # deltas mano (ajusta /1000 según unidades reales)
        dx_hand = (wp_base.pose.position.x - self.hand_origin_ps.pose.position.x) * self.hand_scale
        dy_hand = (wp_base.pose.position.y - self.hand_origin_ps.pose.position.y) * self.hand_scale
        dz_hand = (self.hand_origin_ps.pose.position.z - wp_base.pose.position.z) * self.hand_scale
        # remapeo con inversión
        desired_x = float(self.ee_origin.position.x - dz_hand)  # mano Z -> robot X
        desired_y = float(self.ee_origin.position.y - dx_hand)  # mano X -> robot Y
        desired_z = float(self.ee_origin.position.z - dy_hand)  # mano Y -> robot Z (invertido)

        # clamp límites
        desired_x = min(max(desired_x, self.x_limits[0]), self.x_limits[1])
        desired_y = min(max(desired_y, self.y_limits[0]), self.y_limits[1])
        desired_z = min(max(desired_z, self.z_limits[0]), self.z_limits[1])

        # dt
        now = self.get_clock().now()
        dt = (now - self._prev_t).nanoseconds * 1e-9
        self._prev_t = now

        e = np.array([
            desired_x - float(ee_pose.position.x),
            desired_y - float(ee_pose.position.y),
            desired_z - float(ee_pose.position.z),
        ], dtype=float)
        
        step = self._pid_step(e, dt)

        target = PoseStamped()
        target.header.stamp = now.to_msg()
        target.header.frame_id = "fr3_link0"
        target.pose.position.x = float(ee_pose.position.x + step[0])
        target.pose.position.y = float(ee_pose.position.y + step[1])
        target.pose.position.z = float(ee_pose.position.z + step[2])
        target.pose.orientation = self.desired_ee_pose.orientation

        self._send_ik_request(target)

def main(args=None):
    rclpy.init(args=args)

    cv_franka_bridge = CvFrankaBridge()

    rclpy.spin(cv_franka_bridge)


if __name__ == '__main__':
    main()
