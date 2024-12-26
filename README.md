# Smart Robotic Arm for Ball Collection Simulation

This project presents the design and simulation of an intelligent robotic arm capable of collecting colored balls and placing them into designated baskets based on specified quantities and colors. Leveraging ROS 2 and Gazebo, the system models the robotic arm's movement, control, and perception capabilities. A camera is utilized for detecting the color and position of the balls, translating these detections into real-world coordinates relative to the robot's initial frame, allowing precise manipulation by the gripper. The system supports multiple operation modes, including a scanning mode for ball identification and an autonomous mode for seamless task execution.

## Features

- **Ball Collection:** The robotic arm identifies and picks up colored balls from random positions and places them in a basket.
- **Multiple Operation Modes:** Includes TeleOp, Scan, SetHome, and Auto modes for versatile functionality.
- **PID Control:** Precise joint control using PID algorithms to ensure smooth operation.
- **Simulation in Gazebo:** Fully simulated environment with interactive robot control and visualization.

## System Overview

### System Diagram
![image](https://github.com/user-attachments/assets/41fc89d2-5713-41bb-9bdc-638da1ead339)
1. **Start by Randomizing Balls:** Randomly assigns colors and positions to the balls.
2. **Receive User Commands:** Accepts user inputs for ball color and number.
3. **Scan Mode:** Scans all balls to identify their colors and locations.
4. **Record Ball Locations:** Stores the positions of all identified balls.
5. **Navigate to Target Position:** Moves to the saved position of the selected ball.
6. **Track Ball Position:** Uses the robot’s camera to calculate the distance to the ball.
7. **Pick Ball:** Aligns the gripper and uses the robotic arm to pick up the ball.
8. **Place Ball:** Places the ball into the basket.
9. **Repeat:** Continues until all balls are collected or the operation is terminated.

### Modes of Operation

1. **TeleOp Mode:** Allows manual control of the robotic arm.
2. **Scan Mode:** Identifies and records the positions and colors of all balls.
3. **SetHome Mode:** Returns the robotic arm to its home position.
4. **Auto Mode:** Automatically performs the ball collection and placement tasks based on the user’s commands.

### Simulation Environment

- The simulation is conducted in **Gazebo**, a powerful robot simulation platform.
- The robot model includes:
  - A 4-DOF robotic arm with a gripper.
  - A shelf system for storing balls.
- Visualization and control of joint states using ROS 2 tools.

## How to Run

1. Install ROS 2 (Humble or compatible version).
2. Clone this repository and navigate to the workspace.
3. Build the ROS 2 workspace:
   ```bash
   colcon build
   ```
4. Source the workspace:
   ```bash
   source install/setup.bash
   ```
5. Launch the simulation:
   ```bash
   ros2 launch robot_arm1 newgazebo.launch.py 
   ```
    <package_name> robot_arm1
    <launch_file> newgazebo

6. Control the robot using:
   - **TeleOp:** `ros2 topic pub /cmd_vel geometry_msgs/Twist ...`
   - **Mode Change:** `ros2 topic pub /change_mode std_msgs/Int16 "data: <mode_number>"`

### Changing Modes and Using the Script

Upon running the script,
```bash
ros2 run robot_arm1 jointstate_script.py
```
the following modes and controls are available:

```text
===========================
🎉 JointStateNode has started! 🎉
===========================
Use the following command to change mode:
ros2 topic pub /change_mode std_msgs/Int16 "data: N"

Where N is:

    ---------- TeleOp Mode ----------
    1-6: Joint control
       joint1
       joint2
       joint3
       joint4
       left_gear_joint and right_gear_joint
       left_finger_joint and right_finger_joint

    ---------- Set Home Mode ----------
    7: Set home mode

    ---------- Taking Picture Mode ----------
    8: Auto mode
===========================
```
when The image shows the names of the joints at each position that needs to be controlled.
![image](https://github.com/user-attachments/assets/380f4391-1d2c-478e-b9a5-497b598910ea) ![image](https://github.com/user-attachments/assets/6f32ffd0-ff3f-4ea1-8568-4983807b5ade)


### Example Run Outputs

#### Running TeleOp Mode

Command:
```bash
ros2 topic pub /change_mode std_msgs/Int16 "data: 1"
```
manual control joint by :
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Expected Behavior:
- Joint1 starts moving under manual control.
- Joint state values are updated and published.

#### Returning to Home Position

Command:
```bash
ros2 topic pub /change_mode std_msgs/Int16 "data: 7"
```
Expected Behavior:
- The robot smoothly moves back to its initial position.

#### Auto Mode for Ball Collection

Command:
```bash
ros2 topic pub /change_mode std_msgs/Int16 "data: 8"
```
Expected Behavior:
- The robot scans the environment.
- Ball positions and colors are identified and logged.
- The robot autonomously picks up balls and places them in the designated basket.
- Camera alignment and precise movements are observed.

## Future Work

- **Enhancements:**
  - Add real-time depth sensing for ball detection.
  - Optimize PID parameters for smoother operations.
- **Testing:**
  - Perform stress tests with more balls and complex arrangements.
  - Integrate a failure recovery system for missed balls.

## References

- [ROS 2 Documentation](https://docs.ros.org/en/)
- [Gazebo Documentation](http://gazebosim.org/)
- **Project Presentation:** See `Project Presentation` for additional details and system diagrams.
- [Project Presentation](http://gazebosim.org/)
