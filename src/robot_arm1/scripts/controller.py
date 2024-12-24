#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from lab1_interfaces.srv import SetMode
import sys
import subprocess

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.teleop_process =None
        self.client = self.create_client(SetMode, 'mode')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('Service available, sending request...')

    def send_request(self, mode, x=None, y=None, z=None, ref=None):
        request = SetMode.Request()
        request.mode.data = mode

        if mode == 0:
            if x is not None and y is not None and z is not None:
                request.x.data = x
                request.y.data = y
                request.z.data = z

        if mode == 1:
            if ref is not None:
                request.x.data = ref 
        if mode == 2:
            pass
                
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.display_response(response, mode, x, y, z, ref)


    def display_response(self, response, mode, x=None, y=None, z=None, ref=None,q1sol=None,q2sol=None,q3sol=None,ipk =None):
        if response.result.data == 1:
            if mode == 0:
                self.get_logger().info(f"Success! Mode IPK with values: x={x}, y={y}, z={z}")
                if response.ipk.data == 1:
                    self.get_logger().info(f"It's in workspace! Rad to go: q1={response.q0sol.data}, q2={response.q1sol.data}, a3={response.q2sol.data}")
                else:
                    self.get_logger().info(f"It's out workspace!!!")
            elif mode == 1:
                ref_type = 'ref_hand' if ref == 0 else 'ref_base'
                self.get_logger().info(f"Success! Mode Teleop with {ref_type}")
                self.run_teleop_twist_keyboard()
            elif mode == 2:
                self.get_logger().info("Success! Mode Auto")
            else:
                self.get_logger().info("Unknown mode, but operation succeeded.")
        else:
            self.get_logger().warning(f"Operation failed with response result: {response.result.data}")
    def run_teleop_twist_keyboard(self):
        if self.teleop_process is not None and self.teleop_process.poll() is None:
            self.get_logger().info("Terminating previous teleop.py process.")
            self.teleop_process.terminate()
        self.get_logger().info("Starting teleop.py in a new terminal...")

        self.teleop_process = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'ros2 run example_description teleop.py; exec bash'])

    def stop_teleop_twist_keyboard(self):
        if self.teleop_process is not None and self.teleop_process.poll() is None:
            self.get_logger().info("Stopping teleop_twist_keyboard...")
            self.teleop_process.terminate()
            self.teleop_process = None
def main(args=None):
    rclpy.init(args=args)
    
    # Check if the mode argument is passed
    if len(sys.argv) < 2:
        print("mode 0 : Inverse Pose Kinematics (IPK)\n mode 1 : Teleoperation\n mode 2 : Auto")
        print("Usage: ros2 run example_description controller.py <mode> <x> <y> <z> [for mode 0]")
        print("Usage: ros2 run example_description controller.py <mode> <0 or 1>[0: ref_hand or 1:ref_base for mode 1]")
        print("Usage: ros2 run example_description controller.py <mode> [for mode 2]")
        return
    
    mode = int(sys.argv[1])  
  
    x, y, z, ref = None, None, None, None
    if mode == 0:
        if len(sys.argv) < 5: 
            print("Error: For Mode 0, you need to provide x, y, z values!")
            print("Usage: ros2 run example_description controller.py 0 <x> <y> <z>")
            return
        x = float(sys.argv[2])  
        y = float(sys.argv[3]) 
        z = float(sys.argv[4])  
    elif mode == 1:
        if len(sys.argv) < 3: 
            print("Error: For Mode 1, you need to provide 0: ref_hand or 1: ref_base values!")
            print("Usage: ros2 run example_description controller.py 1 <ref>")
            return
        ref = float(sys.argv[2]) 
    elif mode == 2:
        if len(sys.argv) < 2: 
            print("Error: For Mode 2, Run Auto Mode!")
            print("Usage: ros2 run example_description controller.py 2")
            return
  
    node = ControllerNode()
    node.send_request(mode, x, y, z, ref)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()