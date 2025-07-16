#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class RobotCalibrationTool(Node):
    """
    Tool to help calibrate the transformation between robots.
    This helps find the correct position and orientation offsets.
    """

    def __init__(self):
        super().__init__('robot_calibration_tool')

        # Default robot configurations
        self.robot_configs = {
            0: {
                'position': [-5.0, -7.0, 0.0],
                'orientation': [0.0, 0.0, 0.0],  # roll, pitch, yaw in radians
                'description': 'Robot 0 (Blue)'
            },
            1: {
                'position': [-1.0, 0.0, 0.0],
                'orientation': [0.0, 0.0, 0.0],
                'description': 'Robot 1 (Green)'
            },
            2: {
                'position': [5.0, 5.0, 0.0],
                'orientation': [0.0, 0.0, 0.0],
                'description': 'Robot 2 (Red)'
            }
        }

        # Load calibration if exists
        self.calibration_file = 'robot_calibration.yaml'
        self.load_calibration()

        self.get_logger().info('Robot Calibration Tool Started')
        self.get_logger().info('Commands:')
        self.get_logger().info('  show - Show current calibration')
        self.get_logger().info('  set <robot_id> pos <x> <y> <z> - Set position')
        self.get_logger().info('  set <robot_id> rot <roll> <pitch> <yaw> - Set rotation (degrees)')
        self.get_logger().info('  save - Save calibration to file')
        self.get_logger().info('  load - Load calibration from file')
        self.get_logger().info('  help - Show this help')
        self.get_logger().info('  exit - Exit calibration tool')

        # Start interactive calibration
        self.interactive_calibration()

    def load_calibration(self):
        """Load calibration from file if exists"""
        if os.path.exists(self.calibration_file):
            try:
                with open(self.calibration_file, 'r') as f:
                    data = yaml.safe_load(f)
                    if data and 'robots' in data:
                        for robot_id, config in data['robots'].items():
                            if robot_id in self.robot_configs:
                                self.robot_configs[robot_id].update(config)
                self.get_logger().info(f'Loaded calibration from {self.calibration_file}')
            except Exception as e:
                self.get_logger().error(f'Failed to load calibration: {e}')

    def save_calibration(self):
        """Save current calibration to file"""
        try:
            data = {'robots': {}}
            for robot_id, config in self.robot_configs.items():
                data['robots'][robot_id] = {
                    'position': config['position'],
                    'orientation': config['orientation']
                }

            with open(self.calibration_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)

            self.get_logger().info(f'Saved calibration to {self.calibration_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save calibration: {e}')

    def show_calibration(self):
        """Display current calibration"""
        print("\n===== Current Robot Calibration =====")
        for robot_id, config in self.robot_configs.items():
            print(f"\n{config['description']}:")
            print(f"  Position: {config['position']}")
            rot_deg = np.degrees(config['orientation'])
            print(f"  Rotation (degrees): [roll={rot_deg[0]:.1f}, pitch={rot_deg[1]:.1f}, yaw={rot_deg[2]:.1f}]")

            # Show transformation matrix
            transform = self.create_transformation_matrix(
                config['position'], config['orientation']
            )
            print(f"  Transformation Matrix:")
            for row in transform:
                print(f"    [{row[0]:7.3f}, {row[1]:7.3f}, {row[2]:7.3f}, {row[3]:7.3f}]")
        print("=" * 37 + "\n")

    def create_transformation_matrix(self, position, orientation):
        """Create 4x4 transformation matrix"""
        rotation = R.from_euler('xyz', orientation)
        rotation_matrix = rotation.as_matrix()

        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = position

        return transform

    def set_position(self, robot_id, x, y, z):
        """Set robot position"""
        if robot_id in self.robot_configs:
            self.robot_configs[robot_id]['position'] = [x, y, z]
            self.get_logger().info(f"Set Robot {robot_id} position to [{x}, {y}, {z}]")
        else:
            self.get_logger().error(f"Invalid robot ID: {robot_id}")

    def set_rotation(self, robot_id, roll_deg, pitch_deg, yaw_deg):
        """Set robot rotation (input in degrees, stored in radians)"""
        if robot_id in self.robot_configs:
            roll_rad = np.radians(roll_deg)
            pitch_rad = np.radians(pitch_deg)
            yaw_rad = np.radians(yaw_deg)
            self.robot_configs[robot_id]['orientation'] = [roll_rad, pitch_rad, yaw_rad]
            self.get_logger().info(
                f"Set Robot {robot_id} rotation to [roll={roll_deg}°, pitch={pitch_deg}°, yaw={yaw_deg}°]"
            )
        else:
            self.get_logger().error(f"Invalid robot ID: {robot_id}")

    def interactive_calibration(self):
        """Interactive calibration interface"""
        while True:
            try:
                command = input("\nCalibration> ").strip().lower()

                if command == 'exit':
                    break
                elif command == 'help':
                    self.show_help()
                elif command == 'show':
                    self.show_calibration()
                elif command == 'save':
                    self.save_calibration()
                elif command == 'load':
                    self.load_calibration()
                    self.show_calibration()
                elif command.startswith('set'):
                    self.process_set_command(command)
                else:
                    print(f"Unknown command: {command}")
                    print("Type 'help' for available commands")

            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

    def process_set_command(self, command):
        """Process set commands"""
        parts = command.split()
        if len(parts) < 6:
            print("Invalid set command. Usage:")
            print("  set <robot_id> pos <x> <y> <z>")
            print("  set <robot_id> rot <roll> <pitch> <yaw>")
            return

        try:
            robot_id = int(parts[1])
            cmd_type = parts[2]

            if cmd_type == 'pos' and len(parts) == 6:
                x, y, z = float(parts[3]), float(parts[4]), float(parts[5])
                self.set_position(robot_id, x, y, z)
            elif cmd_type == 'rot' and len(parts) == 6:
                roll, pitch, yaw = float(parts[3]), float(parts[4]), float(parts[5])
                self.set_rotation(robot_id, roll, pitch, yaw)
            else:
                print("Invalid set command format")
        except ValueError:
            print("Invalid number format")

    def show_help(self):
        """Show help information"""
        print("\n===== Robot Calibration Tool Help =====")
        print("Commands:")
        print("  show                              - Show current calibration")
        print("  set <robot_id> pos <x> <y> <z>    - Set robot position")
        print("  set <robot_id> rot <r> <p> <y>    - Set robot rotation (degrees)")
        print("  save                              - Save calibration to file")
        print("  load                              - Load calibration from file")
        print("  help                              - Show this help")
        print("  exit                              - Exit calibration tool")
        print("\nExample:")
        print("  set 0 pos -5.0 -7.0 1.0")
        print("  set 0 rot 0 0 45")
        print("  save")
        print("=" * 39)


def main(args=None):
    rclpy.init(args=args)

    node = RobotCalibrationTool()

    # The node handles everything in its constructor
    # Just need to clean up
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()