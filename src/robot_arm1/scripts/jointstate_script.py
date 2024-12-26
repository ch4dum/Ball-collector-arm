#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Bool, Float64, String
from geometry_msgs.msg import Point
from math import pi
import numpy as np
import yaml
import os

class JointStateNode(Node):
    def __init__(self):
        super().__init__('jointstate_script_node')

        # Publishers and Subscribers
        self.velo_pub = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands", 10)
        self.joint_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.capture_pub = self.create_publisher(Bool, "/capture", 10)  # Publisher for /capture
        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.mode_sub = self.create_subscription(Int16, "/change_mode", self.change_mode_callback, 10)
        self.ball_pose_sub = self.create_subscription(Point, '/ball_pose', self.ball_pose_callback, 10)
        self.ball_color_sub = self.create_subscription(String, '/ball_color', self.ball_color_callback, 10)
        
        # Gripper-related
        self.gripper_still_count = 0
        self.prev_left_gear = 0.0
        self.prev_right_gear = 0.0
        self.gripper_still_threshold = 0.0005  # Threshold for considering the gripper is "not moving"
        self.gripper_still_max_count = 10      # Number of consecutive checks indicating the gripper has fully closed

        # General control/variables
        self.dt = 0.01
        self.q = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.cmd = 0.0
        self.name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "left_gear_joint",
            "left_finger_joint",
            "right_gear_joint",
            "right_finger_joint"
        ]
        self.velo = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pos = []
        self.mode = 0
        self.total_modes = 8
        self.step = 0
        self.count_step = 0
        self.set_step_joint2 = [0, 0.376, 0.752, 1.096, 1.2]
        self.joint_order = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "left_gear_joint",
            "left_finger_joint",
            "right_gear_joint",
            "right_finger_joint"
        ]

        # Path points for taking pictures (mode=8)
        self.path_points = [
            (0.9, 0.0), (1.6, 0.0),
            (1.6, 0.376), (0.9, 0.376),
            (0.9, 0.752), (1.6, 0.752),
            (1.6, 1.096), (0.9, 1.096)
        ]
        self.current_target_index = 0
        self.pause_timer = 0.0
        self.moving_to_target = True

        # PID parameters
        self.y_pid = {"kp": 10.0, "ki": 0.0, "kd": 0.0, "prev_error": 0.0, "integral": 0.0}
        self.x_pid = {"kp": 5.0, "ki": 0.0, "kd": 0.0, "prev_error": 0.0, "integral": 0.0}

        self.joint1_pid = {"kp": 32.0, "ki": 0.0, "kd": 0.01, "prev_error": 0.0, "integral": 0.0}
        self.joint2_pid = {"kp": 32.0, "ki": 16.0, "kd": 4.0, "prev_error": 0.0, "integral": 0.0}

        self.save_color = ""

        # Joint limits (min, max)
        self.limits = [
            (0.0, 2.3),     # Joint 1
            (0.0, 1.2),     # Joint 2
            (-1.570, 0.0),  # Joint 3
            (0.0, 0.300),   # Joint 4
            (-1, -0.4),     # left_gear_joint
            (-0.5, 0.5),    # left_finger_joint
            (1, 0.4),       # right_gear_joint
            (-0.5, 0.5)     # right_finger_joint
        ]

        self.joint1_direction = 1
        self.joint1_at_limit = False
        self.home_completed = False
        self.pause_flag = False
        self.pause_start_time = None
        self.pause_duration = 0.25

        # For ball detection
        self.ball_posx = 0
        self.ball_color = 0

        # Create main loop timer
        self.create_timer(self.dt, self.sim_loop)

        # YAML file paths for saving positions
        self.yaml_file_path = os.path.join(os.path.expanduser("~"), "robotarm_ws", "config", "joint_positions.yaml")
        self.processed_yaml_file_path = os.path.join(os.path.expanduser("~"), "robotarm_ws", "config", "processed_joint_positions.yaml")

        self.sub_state_20 = 0  # For state machine usage if needed

        # Create directory if it does not exist
        yaml_directory = os.path.dirname(self.yaml_file_path)
        if not os.path.exists(yaml_directory):
            os.makedirs(yaml_directory)
            self.get_logger().info(f"📂 Created directory: {yaml_directory}")

        self.position_count = 0
        self.clear_yaml_file()

        # For user-input color amounts
        self.user_input_colors = {}

        # For mode=20, 25
        # List of targets with keys { 'joint1':..., 'joint2':..., 'ball_color':... }
        self.target_positions = []
        self.current_target = None

        # Startup message
        startup_message = """
        ====================================================
                    🚀 JointStateNode has started! 🚀
        ====================================================
        Use the following command to change mode:
            ros2 topic pub /change_mode std_msgs/Int16 "data: N"

        Where N is:
        ---------- Teleop Mode  ----------
            1-6: Joint control
                1. joint1
                2. joint2
                3. joint3
                4. joint4
                5. left_gear_joint and right_gear_joint
                6. left_finger_joint and right_finger_joint

        ---------- Set Home Mode ----------
            7: Set home mode

        ---------- Taking Picture Mode ----------
            8: Taking picture (Auto mode)
        ====================================================
        """
        self.get_logger().info(startup_message)

    def is_close(self, pos1, pos2, tolerance):
        """
        Check if two positions (dicts) are within a certain tolerance for joint1 and joint2.
        """
        return (
            abs(pos1['joint1'] - pos2['joint1']) <= tolerance and
            abs(pos1['joint2'] - pos2['joint2']) <= tolerance
        )

    def process_saved_positions(self, tolerance=0.05):
        """
        Process the saved positions in self.yaml_file_path, group close positions, and
        save the averaged result to self.processed_yaml_file_path.
        """
        if not os.path.exists(self.yaml_file_path):
            self.get_logger().warn("❗ No saved positions file found to process.")
            return

        with open(self.yaml_file_path, 'r') as f:
            try:
                data = yaml.safe_load(f) or {}
                positions = data.get('saved_positions', [])
            except yaml.YAMLError as e:
                self.get_logger().error(f"❗ Failed to load YAML file: {e}")
                return

        grouped_positions = []
        while positions:
            current = positions.pop(0)
            group = [current]

            close_positions = []
            for pos in positions:
                if self.is_close(pos, current, tolerance):
                    group.append(pos)
                else:
                    close_positions.append(pos)
            positions = close_positions

            avg_joint1 = float(np.mean([pos['joint1'] for pos in group]))
            avg_joint2 = float(np.mean([pos['joint2'] for pos in group]))

            colors = [pos['ball_color'] for pos in group]
            most_common_color = max(set(colors), key=colors.count)

            grouped_positions.append({
                'joint1': avg_joint1,
                'joint2': avg_joint2,
                'ball_color': most_common_color
            })

        with open(self.processed_yaml_file_path, 'w') as f:
            yaml.dump({'averaged_positions': grouped_positions}, f, default_flow_style=False)
        self.get_logger().info(f"✅ Processed positions saved to {self.processed_yaml_file_path}")

    def clear_yaml_file(self):
        """
        Clear the YAML file before saving new data.
        """
        yaml_directory = os.path.dirname(self.yaml_file_path)
        if not os.path.exists(yaml_directory):
            os.makedirs(yaml_directory)
            self.get_logger().info(f"📂 Created directory: {yaml_directory}")

        if os.path.exists(self.yaml_file_path):
            os.remove(self.yaml_file_path)
            self.get_logger().info("🗑️ Previous joint positions file cleared.")

    def save_joint_position_to_yaml(self, joint1, joint2):
        """
        Save the given joint1, joint2 positions and ball_color to the YAML file.
        """
        yaml_directory = os.path.dirname(self.yaml_file_path)
        if not os.path.exists(yaml_directory):
            os.makedirs(yaml_directory)
            self.get_logger().info(f"📂 Created directory: {yaml_directory}")

        data = {}
        if os.path.exists(self.yaml_file_path):
            with open(self.yaml_file_path, 'r') as yaml_file:
                try:
                    data = yaml.safe_load(yaml_file) or {}
                except yaml.YAMLError:
                    data = {}

        if 'saved_positions' not in data:
            data['saved_positions'] = []

        self.position_count += 1

        if isinstance(self.ball_color, str):
            self.save_color = self.ball_color
        else:
            self.save_color = self.ball_color.data if hasattr(self.ball_color, 'data') else str(self.ball_color)

        # If the color is '0', skip saving it
        if self.save_color == '0':
            return

        data['saved_positions'].append({
            'id': self.position_count,
            'joint1': joint1,
            'joint2': joint2,
            'ball_color': self.save_color
        })

        try:
            with open(self.yaml_file_path, 'w') as yaml_file:
                yaml.dump(data, yaml_file, default_flow_style=False)
            self.get_logger().info(
                f"✅ Saved position {self.position_count} with ball color '{self.save_color}' to {self.yaml_file_path}"
            )
        except Exception as e:
            self.get_logger().error(f"❗ Failed to save positions: {e}")

    def ball_pose_callback(self, msg: Point):
        """
        Callback for the ball's position.
        """
        self.ball_posx = msg.x

    def ball_color_callback(self, msg: String):
        """
        Callback for the ball's color.
        """
        self.ball_color = msg.data

    def joint_state_callback(self, msg: JointState):
        """
        Callback for joint states, updates self.pos accordingly.
        """
        joint_position_map = {name: pos for name, pos in zip(msg.name, msg.position)}
        self.pos = [joint_position_map.get(joint, 0.0) for joint in self.joint_order]

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback for cmd_vel (linear.x).
        """
        self.cmd = msg.linear.x

    def change_mode_callback(self, msg: Int16):
        """
        Callback to change the control mode (1-6, 7, 8, etc.).
        """
        if 1 <= msg.data <= self.total_modes:
            self.mode = msg.data
            self.get_logger().info(f"""
    ====================================================
                🔄 Mode changed to: {self.mode}
    ====================================================
    {"Teleop Mode: Control joints." if 1 <= self.mode <= 6 else
    "Set Home Mode: Moving to home position." if self.mode == 7 else
    "Taking Picture Mode: Auto-capture mode." if self.mode == 8 else ""}
    """)
        else:
            self.get_logger().warn(f"❗ Invalid mode: {msg.data}. Mode must be between 1 and {self.total_modes}.")

    def pid_control(self, target, current, pid_params):
        """
        A simple PID control function for 1D control.
        """
        error = target - current
        pid_params["integral"] += error * self.dt
        derivative = (error - pid_params["prev_error"]) / self.dt
        output = (
            pid_params["kp"] * error +
            pid_params["ki"] * pid_params["integral"] +
            pid_params["kd"] * derivative
        )
        pid_params["prev_error"] = error
        return output

    def sim_loop(self):
        """
        Main loop controlling the robot's movement based on the current mode.
        """
        # If joint states aren't populated yet, skip.
        if len(self.pos) < len(self.joint_order):
            self.get_logger().warn("❗ Joint positions not fully populated. Skipping loop iteration.")
            return

        # Initialize velocities to zero each iteration
        self.velo = [0.0] * len(self.velo)
        tolerance = 0.005

        # Mode 0: Do nothing
        if self.mode == 0:
            self.velo = [0.0] * len(self.velo)

        # Mode 7: Set Home
        if self.mode == 7:
            # Check if left_gear and right_gear are at their home limit
            if abs(self.pos[4] - self.limits[4][0]) <= tolerance and abs(self.pos[6] - self.limits[6][0]) <= tolerance:
                # Move joint4 down
                self.velo[3] = -0.5
                if abs(self.pos[3] - self.limits[3][0]) <= tolerance:
                    # Move joint3 up
                    self.velo[2] = 0.5
                    if abs(self.pos[2] - self.limits[2][1]) <= tolerance:
                        # Move joint2 down
                        self.velo[1] = -0.5
                        if abs(self.pos[1] - self.limits[1][0]) <= tolerance:
                            # Move joint1 down
                            self.velo[0] = -0.5
                            if abs(self.pos[0] - self.limits[0][0]) <= tolerance:
                                self.velo = [0.0] * len(self.velo)
                                self.get_logger().info("""
                        ====================================================
                                        ✅ Home position reached!
                        ====================================================
                        """)
                                self.process_saved_positions()
                                self.velo[2] = 0.5
                                self.mode = 0
                                self.velo = [0.0] * len(self.velo)
                                return
            else:
                # Close the gripper
                self.velo[4] = -0.5
                self.velo[6] = 0.5

        # Mode 9: Similar to going Home but afterwards goes to mode=15
        if self.mode == 9:
            if abs(self.pos[4] - self.limits[4][0]) <= tolerance and abs(self.pos[6] - self.limits[6][0]) <= tolerance:
                self.velo[3] = -0.5
                if abs(self.pos[3] - self.limits[3][0]) <= tolerance:
                    self.velo[2] = 0.5
                    if abs(self.pos[2] - self.limits[2][1]) <= tolerance:
                        self.velo[1] = -0.5
                        if abs(self.pos[1] - self.limits[1][0]) <= tolerance:
                            self.velo[0] = -0.5
                            if abs(self.pos[0] - self.limits[0][0]) <= tolerance:
                                self.velo[2] = 0.5
                                if abs(self.pos[2] - self.limits[2][1]) <= tolerance:
                                    self.velo = [0.0] * len(self.velo)
                                    self.get_logger().info("""
                            ====================================================
                                            ✅ Home position reached!
                            ====================================================
                            """)
                                    self.process_saved_positions()
                                    self.velo[2] = 0.5
                                    self.mode = 15
                                    self.velo = [0.0] * len(self.velo)
                                    return
                                else:
                                    self.velo[2] = 0.5
            else:
                self.velo[2] = 0.5
                self.velo[4] = -0.5
                self.velo[6] = 0.5

        # Mode 8: Taking Pictures (Auto mode)
        if self.mode == 8:
            if self.moving_to_target:
                tol = 0.01
                # If the ball is near x=0.0, save position if the color is not blank
                if abs(self.ball_posx + 0.0) <= tol:
                    if self.save_color != ' ':
                        self.save_joint_position_to_yaml(self.pos[0], self.pos[1])
                        self.get_logger().info("🎯 Target position saved successfully.")

                target_x, target_y = self.path_points[self.current_target_index]

                # PID control
                pid_x = self.pid_control(target_x, self.pos[0], self.x_pid)
                self.velo[0] = np.clip(pid_x, -0.2, 0.2)

                pid_y = self.pid_control(target_y, self.pos[1], self.y_pid)
                self.velo[1] = np.clip(pid_y, -0.2, 0.2)

                # Check if we reached the target
                if abs(target_x - self.pos[0]) < 0.001 and abs(target_y - self.pos[1]) < 0.001:
                    self.velo[0] = 0.0
                    self.velo[1] = 0.0
                    self.pause_timer = 0.0
                    self.moving_to_target = False
            else:
                # Pause before moving to the next target
                self.pause_timer += self.dt
                if self.pause_timer >= 0.25:
                    self.current_target_index += 1
                    if self.current_target_index >= len(self.path_points):
                        self.current_target_index = 0
                        self.mode = 9
                        self.get_logger().info("""
        ====================================================
            📸 Path complete. Switching to Home Mode.
        ====================================================
        """)
                    else:
                        self.moving_to_target = True

        # Mode 5: Teleop for left/right gear
        elif self.mode == 5:
            self.velo[4] = self.cmd
            self.velo[6] = -self.cmd

        # Mode 6: Teleop for left/right finger
        elif self.mode == 6:
            self.velo[5] = self.cmd
            self.velo[7] = -self.cmd

        else:
            # Modes 1-4: Teleop for joints
            for i in range(1, 5):
                if self.mode == i:
                    self.velo[i-1] = self.cmd

        # After going Home -> mode=15 -> get user input -> move to mode=20
        if self.mode == 15:
            self.velo = [0.0] * len(self.velo)
            if os.path.exists(self.processed_yaml_file_path):
                self.velo = [0.0] * len(self.velo)
                with open(self.processed_yaml_file_path, 'r') as f:
                    data = yaml.safe_load(f) or {}
                    averaged_positions = data.get('averaged_positions', [])

                color_count = {}
                for pos_data in averaged_positions:
                    c = pos_data['ball_color']
                    if c not in color_count:
                        color_count[c] = 0
                    color_count[c] += 1

                self.get_logger().info("📝 Summary of ball colors:")
                for c, cnt in color_count.items():
                    self.get_logger().info(f"{c}: {cnt}")

                # Ask user how many balls for each color
                colors_to_ask = ["Pink", "Blue", "Red", "Green", "Yellow"]
                self.get_logger().info("Enter the number of balls for each color (skip if none):")
                for c in colors_to_ask:
                    val = input(f"{c}: ").strip()
                    if val != '':
                        try:
                            val = int(val)
                        except:
                            val = 0
                        self.user_input_colors[c] = val

                self.get_logger().info(f"User input colors: {self.user_input_colors}")

                # Build target_positions based on user input
                self.target_positions = []
                for pos_data in averaged_positions:
                    c = pos_data['ball_color']
                    if c in self.user_input_colors and self.user_input_colors[c] > 0:
                        self.target_positions.append(pos_data)
                        self.user_input_colors[c] -= 1

                self.mode = 20

        # Mode 20: Move to target positions
        if self.mode == 20:
            self.velo = [0.0] * len(self.velo)

            # 1) If there's no current target, but we still have targets left, get the next one
            if self.current_target is None and len(self.target_positions) > 0:
                self.current_target = self.target_positions[0]
                self.get_logger().info(f"🎯 Moving to target: {self.current_target}")

            # If we don't have any target left
            if self.current_target is None and len(self.target_positions) == 0:
                self.get_logger().info("✅ No more targets to move to.")
                self.mode = 7  # Go Home
            else:
                joint1_target = self.current_target['joint1']
                joint2_target = self.current_target['joint2']

                # (A) Move joint1, joint2 via PID
                pid_x = self.pid_control(joint1_target, self.pos[0], self.x_pid)
                self.velo[0] = np.clip(pid_x, -0.2, 0.2)

                pid_y = self.pid_control(joint2_target, self.pos[1], self.y_pid)
                self.velo[1] = np.clip(pid_y, -0.2, 0.2)

                reach_tol = 0.003
                if (abs(joint1_target - self.pos[0]) < reach_tol and 
                    abs(joint2_target - self.pos[1]) < reach_tol):

                    # Stop joint1, joint2
                    self.velo[0] = 0.0
                    self.velo[1] = 0.0
                    self.get_logger().info(f"🏁 Reached target: {self.current_target}")

                    # (B) Raise joint3 (move to self.limits[2][1])
                    self.velo[2] = 0.5
                    if abs(self.pos[2] - self.limits[2][1]) <= tolerance:
                        # Once joint3 is up, move joint4 to 0.1
                        if abs(self.pos[3] - 0.1) > 0.005:
                            self.velo[3] = 0.1
                            # Stop all gripper joints (avoid shaking)
                            self.velo[4] = 0.0
                            self.velo[5] = 0.0
                            self.velo[6] = 0.0
                            self.get_logger().info("Joint4 Going Out")
                        else:
                            # (C) If joint4 is near 0.1 -> Gripper grabs
                            if abs(self.pos[3] - 0.1) <= 0.005:
                                self.get_logger().info("Gripper Grabbing...")

                                # Command gripper to close
                                self.velo[4] = 0.5
                                self.velo[6] = -0.5
                                self.velo[3] = 0.0  # Stop joint4 motion

                                # Check if the gripper is "not moving"
                                diff_left = abs(self.pos[4] - self.prev_left_gear)
                                diff_right = abs(self.pos[6] - self.prev_right_gear)

                                if diff_left < self.gripper_still_threshold and diff_right < self.gripper_still_threshold:
                                    self.gripper_still_count += 1
                                else:
                                    self.gripper_still_count = 0

                                # Update previous gear positions
                                self.prev_left_gear = self.pos[4]
                                self.prev_right_gear = self.pos[6]

                                # If gripper hasn't moved for enough counts, we consider it fully closed
                                if self.gripper_still_count > self.gripper_still_max_count:
                                    self.velo[4] = 0.5
                                    self.velo[6] = -0.5
                                    self.get_logger().info("✅ Gripper fully closed (no movement). Stopping gripper.")

                                    # (D) Move to mode=26 for the next step
                                    self.mode = 26

        # Mode 26: After the ball is gripped
        if self.mode == 26:
            joint1_target = self.current_target['joint1']
            joint2_target = self.current_target['joint2']

            # (A) Keep moving joint1, joint2 via PID
            pid_x = self.pid_control(joint1_target, self.pos[0], self.x_pid)
            self.velo[0] = np.clip(pid_x, -0.2, 0.2)

            pid_y = self.pid_control(joint2_target, self.pos[1], self.y_pid)
            self.velo[1] = np.clip(pid_y, -0.2, 0.2)

            """
            Example:
                (1) Keep the gripper closed (no further movement).
                (2) Move joint4 back to 0.0.
                (3) Once joint4 is back, go to mode=27.
            """

            # 1) Keep the gripper closed
            self.velo[4] = 0.5
            self.velo[6] = -0.5

            # 2) Move joint4 back to 0.0
            joint4_tol = 0.005
            if abs(self.pos[3]) > joint4_tol:
                # If pos[3] > 0, move negative; else move positive
                if self.pos[3] > 0.0:
                    self.velo[3] = -0.1
                else:
                    self.velo[3] = 0.1
                self.get_logger().info("Joint4 Backing...")
            else:
                self.velo[3] = 0.0
                self.get_logger().info("✅ Joint4 returned to 0.0 -> Moving to mode=27")
                self.mode = 27

        # Mode 27: Lower joint3 to -1.57
        if self.mode == 27:
            joint1_target = self.current_target['joint1']
            joint2_target = self.current_target['joint2']

            # (A) Move joint1, joint2 via PID
            pid_x = self.pid_control(joint1_target, self.pos[0], self.x_pid)
            self.velo[0] = np.clip(pid_x, -0.2, 0.2)

            pid_y = self.pid_control(joint2_target, self.pos[1], self.y_pid)
            self.velo[1] = np.clip(pid_y, -0.2, 0.2)

            # Keep gripper closed
            self.velo[4] = 0.5
            self.velo[6] = -0.5

            # Move joint3 down to -1.57
            if abs(self.pos[2] - (-1.57)) > 0.1:
                self.velo[2] = -0.1
            else:
                self.velo[2] = 0.0
                self.get_logger().info("✅ Joint3 reached -1.57 => Moving to mode=28")
                self.mode = 28

        # Mode 28: Perform final adjustment on joint4, then remove the current target
        if self.mode == 28:
            joint1_target = self.current_target['joint1']
            joint2_target = self.current_target['joint2']

            # (A) Move joint1, joint2 via PID
            pid_x = self.pid_control(joint1_target, self.pos[0], self.x_pid)
            self.velo[0] = np.clip(pid_x, -0.2, 0.2)

            pid_y = self.pid_control(joint2_target, self.pos[1], self.y_pid)
            self.velo[1] = np.clip(pid_y, -0.2, 0.2)

            # Keep gripper closed
            self.velo[4] = 0.5
            self.velo[6] = -0.5

            # Adjust joint4 to 0.2
            if abs(self.pos[3] - 0.2) > 0.1:
                self.velo[3] = 0.1
            else:
                self.velo[3] = 0.0
                self.velo[4] = -0.5
                self.velo[6] = 0.5
                self.get_logger().info("✅ Finish this target")

                # Remove the target from the list since it's done
                if self.current_target in self.target_positions:
                    self.target_positions.remove(self.current_target)
                self.current_target = None

                # If no more targets, go home
                if len(self.target_positions) == 0:
                    self.get_logger().info("🎉 All targets are done. Moving to Home!")
                    self.mode = 7
                else:
                    # Still have targets left
                    self.mode = 20

        # Publish velocity commands
        msg = Float64MultiArray()
        msg.layout.dim = []
        msg.layout.data_offset = 0
        msg.data = self.velo
        self.velo_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()