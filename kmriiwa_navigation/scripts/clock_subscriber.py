from tf2_ros import Buffer, TransformListener
import rclpy
from rosgraph_msgs.msg import Clock

def get_tf_with_ros1_clock():
    node = Node('tf_clock_sync')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    clock_time = None
    
    def clock_callback(msg):
        nonlocal clock_time
        clock_time = msg.clock

    clock_sub = node.create_subscription(Clock, '/clock', clock_callback, 10)
    
    while rclpy.ok() and clock_time is None:
        rclpy.spin_once(node)
        
    return tf_buffer.lookup_transform('map', 'kmriiwa_base_footprint', clock_time)
