import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import math


# Publica transformadas de mi robot

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # velocidades de las ruedas
        self.wr_sub = self.create_subscription(Float32, 'wr', self.wr_callback, 10)
        self.wl_sub = self.create_subscription(Float32, 'wl', self.wl_callback, 10)
        
        # Publisher JointState
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Variables 
        self.left_wheel_angle = 0.0  # Ángulo acumulado de la rueda izquierda [rad]
        self.right_wheel_angle = 0.0  # Ángulo acumulado de la rueda derecha [rad]
        self.wr = 0.0  # Velocidad angular rueda derecha [rad-seg]
        self.wl = 0.0  # Velocidad angular rueda izquierda [rad-seg]
        self.prev_time = self.get_clock().now()
        
       
        self.joint_state = JointState()
        self.joint_state.name = ['wheel_l_joint', 'wheel_r_joint']  
    
    def wr_callback(self, msg):
        self.wr = msg.data
    
    def wl_callback(self, msg):
        self.wl = msg.data
    
    def publish_joint_states(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9  # Delta de tiempo
        self.prev_time = current_time
        
        # velocidades (en radianes)
        self.left_wheel_angle += self.wl * dt
        self.right_wheel_angle += self.wr * dt
        
        # Normalizar angles
        self.left_wheel_angle = math.atan2(math.sin(self.left_wheel_angle), math.cos(self.left_wheel_angle))
        self.right_wheel_angle = math.atan2(math.sin(self.right_wheel_angle), math.cos(self.right_wheel_angle))
        
        
        self.joint_state.header.stamp = current_time.to_msg()
        self.joint_state.position = [self.left_wheel_angle, self.right_wheel_angle]
        self.joint_state.velocity = [self.wl, self.wr]
        self.joint_pub.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()