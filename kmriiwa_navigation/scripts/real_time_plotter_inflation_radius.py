#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float32
from rcl_interfaces.srv import GetParameters
import numpy as np
from datetime import datetime
import os

class PlotRelay(Node):
    def __init__(self):
        super().__init__('plot_relay')
        
        # Publishers
        self.distance_pub = self.create_publisher(Float32, '/goal_distance', 10)
        self.inflation_pub = self.create_publisher(Float32, '/inflation_radius', 10)
        
        # Subscribe to Nav2 feedback
        self.feedback_sub = self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self.feedback_callback,
            10
        )
        
        # Parameter client for reading inflation radius
        self.param_client = self.create_client(
            GetParameters,
            '/global_costmap/global_costmap/get_parameters'
        )
        
        # Store current values
        self.current_inflation_radius = None
        
        # Data storage arrays
        self.distance_array = []
        self.inflation_array = []
        self.time_array = []
        self.start_time = self.get_clock().now()
        
        # Read inflation radius periodically
        self.create_timer(0.5, self.publish_inflation)
        
        # Save data periodically (every 5 seconds)
        self.create_timer(5.0, self.save_data)
        
        # Output directory
        self.output_dir = os.path.expanduser('~/navigation_data')
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info(f'PlotRelay node initialized. Data will be saved to: {self.output_dir}')
    
    def feedback_callback(self, msg):
        """Handle Nav2 feedback with distance remaining"""
        distance_remaining = msg.feedback.distance_remaining
        
        # Filter out zero or invalid distances
        if distance_remaining <= 0.0:
            self.get_logger().debug(
                f'Ignoring invalid distance: {distance_remaining:.2f}m',
                throttle_duration_sec=2.0
            )
            return
        
        # Publish distance
        distance_msg = Float32()
        distance_msg.data = distance_remaining
        self.distance_pub.publish(distance_msg)
        
        # Store data in arrays
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.time_array.append(current_time)
        self.distance_array.append(distance_remaining)
        self.inflation_array.append(self.current_inflation_radius if self.current_inflation_radius else 0.0)
        
        self.get_logger().info(
            f'Distance: {distance_remaining:.2f}m, Inflation: {self.current_inflation_radius:.3f}m, Points: {len(self.time_array)}',
            throttle_duration_sec=1.0
        )
    
    def publish_inflation(self):
        """Read inflation radius from parameter and publish"""
        if not self.param_client.service_is_ready():
            self.get_logger().warn(
                'Parameter service not ready',
                throttle_duration_sec=5.0
            )
            return
        
        try:
            request = GetParameters.Request()
            request.names = ['inflation_layer.inflation_radius']
            
            future = self.param_client.call_async(request)
            future.add_done_callback(self.param_callback)
            
        except Exception as e:
            self.get_logger().error(f'Failed to request inflation radius: {str(e)}')
    
    def param_callback(self, future):
        """Handle parameter response"""
        try:
            response = future.result()
            
            if response.values and len(response.values) > 0:
                inflation_radius = response.values[0].double_value
                self.current_inflation_radius = inflation_radius
                
                # Publish the value
                inflation_msg = Float32()
                inflation_msg.data = inflation_radius
                self.inflation_pub.publish(inflation_msg)
                
                self.get_logger().debug(f'Inflation radius: {inflation_radius:.3f}m')
            else:
                self.get_logger().warn('Empty parameter response')
                
        except Exception as e:
            self.get_logger().error(f'Error reading inflation radius: {str(e)}')
    
    def save_data(self):
        """Save collected data to NPZ file"""
        if len(self.time_array) == 0:
            self.get_logger().debug('No data to save yet')
            return
        
        try:
            # Generate filename with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(self.output_dir, f'navigation_data_{timestamp}.npz')
            
            # Convert lists to numpy arrays and save
            np.savez(
                filename,
                time=np.array(self.time_array),
                distance=np.array(self.distance_array),
                inflation_radius=np.array(self.inflation_array)
            )
            
            self.get_logger().info(
                f'Saved {len(self.time_array)} data points to {filename}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Failed to save data: {str(e)}')
    
    def save_final_data(self):
        """Save final data on shutdown"""
        if len(self.time_array) > 0:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(self.output_dir, f'navigation_data_final_{timestamp}.npz')
            
            try:
                np.savez(
                    filename,
                    time=np.array(self.time_array),
                    distance=np.array(self.distance_array),
                    inflation_radius=np.array(self.inflation_array)
                )
                
                self.get_logger().info(
                    f'Final save: {len(self.time_array)} data points to {filename}'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to save final data: {str(e)}')

def main():
    rclpy.init()
    node = PlotRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down PlotRelay node')
    finally:
        # Save data one last time before shutting down
        node.save_final_data()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()