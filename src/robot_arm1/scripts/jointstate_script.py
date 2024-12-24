#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Bool ,Float64 ,String
from geometry_msgs.msg import Point
from math import pi
import numpy as np
import yaml
import os

class JointStateNode(Node):
    def __init__(self):
        super().__init__('jointstate_script_node')

        self.velo_pub = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands",10)
        self.joint_sub = self.create_subscription(JointState, "/joint_states",self.joint_state_callback, 10)
        self.capture_pub = self.create_publisher(Bool, "/capture", 10)  # Publisher for /capture
        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.mode_sub = self.create_subscription(Int16, "/change_mode", self.change_mode_callback, 10)
        self.ball_pose_sub = self.create_subscription(Point, '/ball_pose',self.ball_pose_callback, 10)
        self.ball_color_sub = self.create_subscription(String, '/ball_color',self.ball_color_callback, 10)
        
        self.dt = 0.01  
        self.q = [0.0, 0.0, 0.0, 0.0, 0.0]  
        self.cmd = 0.0  
        self.name = ["joint1", "joint2", "joint3", "joint4", "left_gear_joint", "left_finger_joint", "right_gear_joint", "right_finger_joint"]  
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
        
        self.path_points = [
            (0.9, 0.0), (1.6, 0.0),
            (1.6, 0.376), (0.9, 0.376),
            (0.9, 0.752), (1.6, 0.752),
            (1.6, 1.096), (0.9, 1.096)
        ]
        self.current_target_index = 0
        self.pause_timer = 0.0
        self.moving_to_target = True
        self.y_pid = {"kp": 10.0, "ki": 0.0, "kd": 0.0, "prev_error": 0.0, "integral": 0.0}
        self.x_pid = {"kp": 5.0, "ki": 0.0, "kd": 0.0, "prev_error": 0.0, "integral": 0.0}

        self.joint1_pid = {"kp": 32.0, "ki": 0.0, "kd": 0.01, "prev_error": 0.0, "integral": 0.0}
        self.joint2_pid = {"kp": 32.0, "ki": 16.0, "kd": 4.0, "prev_error": 0.0, "integral": 0.0}
        self.save_color = ""
        self.limits = [
            (0.0, 2.3),         # Joint 1
            (0.0, 1.2),         # Joint 2
            (-1.570, 0.0),      # Joint 3
            (0.0, 0.300),       # Joint 4
            (-1, -0.4),         # left_gear_joint
            (-0.5,0.5),         # left_finger_joint
            (1,0.4),            # right_gear_joint
            (-0.5,0.5)          # right_finger_joint
        ]
        self.joint1_direction = 1
        self.joint1_at_limit = False
        self.home_completed = False
        self.pause_flag = False
        self.pause_start_time = None
        self.pause_duration = 0.25
        self.ball_posx = 0
        self.ball_color = 0
        self.create_timer(self.dt, self.sim_loop)

        self.yaml_file_path = os.path.join(os.path.expanduser("~"), "robotarm_ws", "config", "joint_positions.yaml")
        self.processed_yaml_file_path = os.path.join(os.path.expanduser("~"), "robotarm_ws", "config", "processed_joint_positions.yaml")

        # ตรวจสอบและสร้างโฟลเดอร์ถ้ายังไม่มี
        yaml_directory = os.path.dirname(self.yaml_file_path)
        if not os.path.exists(yaml_directory):
            os.makedirs(yaml_directory)
            self.get_logger().info(f"📂 Created directory: {yaml_directory}")
        self.position_count = 0
        self.clear_yaml_file()

        # Array สำหรับเก็บค่าจากการ input สีรอบสุดท้าย
        self.user_input_colors = {}

        # สำหรับ mode=20, 25
        self.target_positions = []  # จะเก็บตำแหน่ง { 'joint1':..., 'joint2':..., 'ball_color':... }
        self.current_target = None  # ตำแหน่งปัจจุบันที่กำลังเคลื่อนไปหา

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
        return (abs(pos1['joint1'] - pos2['joint1']) <= tolerance and 
                abs(pos1['joint2'] - pos2['joint2']) <= tolerance)

    def process_saved_positions(self, tolerance=0.05):
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
        yaml_directory = os.path.dirname(self.yaml_file_path)
        if not os.path.exists(yaml_directory):
            os.makedirs(yaml_directory)  # สร้างโฟลเดอร์ถ้าไม่มีอยู่
            self.get_logger().info(f"📂 Created directory: {yaml_directory}")

        if os.path.exists(self.yaml_file_path):
            os.remove(self.yaml_file_path)
            self.get_logger().info("🗑️ Previous joint positions file cleared.")

    def save_joint_position_to_yaml(self, joint1, joint2):
        yaml_directory = os.path.dirname(self.yaml_file_path)
        if not os.path.exists(yaml_directory):
            os.makedirs(yaml_directory)  # สร้างโฟลเดอร์ถ้าไม่มีอยู่
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
            self.get_logger().info(f"✅ Saved position {self.position_count} with ball color '{self.save_color}' to {self.yaml_file_path}")
        except Exception as e:
            self.get_logger().error(f"❗ Failed to save positions: {e}")


    def ball_pose_callback(self, msg: Point):
        self.ball_posx = msg.x

    def ball_color_callback(self, msg: String):
        self.ball_color = msg.data

    def joint_state_callback(self, msg):
        joint_position_map = {name: pos for name, pos in zip(msg.name, msg.position)}
        self.pos = [joint_position_map.get(joint, 0.0) for joint in self.joint_order]

    def cmd_vel_callback(self, msg: Twist):
        self.cmd = msg.linear.x

    def change_mode_callback(self, msg: Int16):
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

        self.velo = [0.0] * len(self.velo)
        
        tolerance = 0.005

        # mode=0 do nothing
        if self.mode == 0:
            self.velo = [0.0] * len(self.velo)
        
        # SetHome (mode=7)
        if self.mode == 7:
            if abs(self.pos[4] - self.limits[4][0]) <= tolerance and abs(self.pos[6] - self.limits[6][0]) <= tolerance:
                self.velo[3] = -0.5
                if abs(self.pos[3] - self.limits[3][0]) <= tolerance:
                    self.velo[2] = 0.5
                    if abs(self.pos[2] - self.limits[2][1]) <= tolerance:
                        self.velo[1] = -0.5
                        if abs(self.pos[1] - self.limits[1][0]) <= tolerance:
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
                self.velo[4] = -0.5
                self.velo[6] = 0.5
        
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

        
    # Taking Picture (mode=8)
        if self.mode == 8:
            if self.moving_to_target:
                tol = 0.008
                if abs(self.ball_posx + 0.0) <= tol:
                    if self.save_color != '0':
                        self.save_joint_position_to_yaml(self.pos[0], self.pos[1])
                        self.get_logger().info("🎯 Target position saved successfully.")

                target_x, target_y = self.path_points[self.current_target_index]

                # PID Control
                pid_x = self.pid_control(target_x, self.pos[0], self.x_pid)
                self.velo[0] = np.clip(pid_x, -0.2, 0.2)

                pid_y = self.pid_control(target_y, self.pos[1], self.y_pid)
                self.velo[1] = np.clip(pid_y, -0.2, 0.2)

                # Check if reached target
                if abs(target_x - self.pos[0]) < 0.001 and abs(target_y - self.pos[1]) < 0.001:
                    self.velo[0] = 0.0
                    self.velo[1] = 0.0
                    self.pause_timer = 0.0
                    self.moving_to_target = False
            else:
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

        elif self.mode == 5:
            self.velo[4] = self.cmd
            self.velo[6] = -self.cmd

        elif self.mode == 6:
            self.velo[5] = self.cmd
            self.velo[7] = -self.cmd

        else:
            # Teleop mode
            for i in range(1, 5):
                if self.mode == i:
                    self.velo[i-1] = self.cmd

        # หลังจาก home เสร็จ -> mode=15 -> รับค่าจากผู้ใช้ -> เข้า mode=20
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

                self.target_positions = []
                for pos_data in averaged_positions:
                    c = pos_data['ball_color']
                    if c in self.user_input_colors and self.user_input_colors[c] > 0:
                        self.target_positions.append(pos_data)
                        self.user_input_colors[c] -= 1

                self.mode = 20

        if self.mode == 20:
            self.velo = [0.0] * len(self.velo)
            if self.current_target is None and len(self.target_positions) > 0:
                self.current_target = self.target_positions[0]
                self.get_logger().info(f"🎯 Moving to target: {self.current_target}")

            if self.current_target is None and len(self.target_positions) == 0:
                self.get_logger().info("✅ No more targets to move to.")
                self.mode = 7  
            else:
                joint1_target = self.current_target['joint1']
                joint2_target = self.current_target['joint2']

                pid_x = self.pid_control(joint1_target, self.pos[0], self.x_pid)
                self.velo[0] = np.clip(pid_x, -0.2, 0.2)

                pid_y = self.pid_control(joint2_target, self.pos[1], self.y_pid)
                self.velo[1] = np.clip(pid_y, -0.2, 0.2)

                reach_tol = 0.003
                if (abs(joint1_target - self.pos[0]) < reach_tol and 
                    abs(joint2_target - self.pos[1]) < reach_tol):

                    self.velo[0] = 0.0
                    self.velo[1] = 0.0
                    self.get_logger().info(f"🏁 Reached target: {self.current_target}")
                    self.velo[2] = 0.5
                    if abs(self.pos[2] - self.limits[2][1]) <= tolerance:
                        if abs(self.pos[3] - 0.12) > 0.05:
                            self.velo[3] = 0.1
                            self.get_logger().info(f"Joint4 Going Out")
                        else:
                            self.get_logger().info(f"Gripper Grapping")
                            self.velo[4] = 0.5
                            self.velo[6] = -0.5
                            self.velo[3] = 0.0
                            self.mode = 26

        if self.mode == 26:
            self.velo[4] = 0.5
            self.velo[6] = -0.5
            if abs(self.pos[4] + 0.4) <= 0.001:
                self.get_logger().info(f"Joint4 Backing")
                self.velo[3] = -0.1
                self.velo[4] = 0.5
                self.velo[6] = -0.5
            if abs(self.pos[3] - 0.0) <= 0.05:
                self.velo[3] = 0.0
                self.velo[4] = 0.5
                self.velo[6] = -0.5
                self.mode = 27

        if self.mode == 27:
            if abs(self.pos[2] - (-1.570)) > 0.1:
                self.velo[2] = -0.5
                self.velo[4] = 0.5
                self.velo[6] = -0.5
            else:
                self.velo[2] = 0.0
                self.velo[4] = -0.5
                self.velo[6] = 0.5
                if abs(self.pos[2] - 0.0) > 0.1:
                    self.velo[2] = 0.5
                else:
                    self.velo[2] = 0.0
                    self.velo[4] = 0.0
                    self.velo[6] = 0.0
                    self.get_logger().info("🔄 Ready for next ball.")
                    self.mode = 20

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