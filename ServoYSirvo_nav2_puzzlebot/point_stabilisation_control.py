import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import transforms3d
import math

class PointStabilisationNode(Node):
    def __init__(self):
        super().__init__('point_stabilisation_control')
        
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        self.create_subscription(Pose, "set_point", self.goal_cb,10)
        # Publica
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # goal
        self.goal_x = 0.0  # [m]
        self.goal_y = 0.0  # [m]
        self.goal_theta = 0.0  # [rad]
        # ganancias
        self.k_rho = 0.3
        self.k_alpha = 0.8
        self.k_beta = -0.15
        # Límite ya llegó
        self.distance_threshold = 0.05  # [m]
        self.angle_threshold = 0.05  # [rad]
        
        # posición 
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Timer 
        self.timer = self.create_timer(0.04, self.control_loop)

    def goal_cb(self,msg):
         self.goal_x = msg.position.x 
         self.goal_y = msg.position.y 
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Convertir quaternion a Euler 
        quat = [
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ]
        _, _, self.theta = transforms3d.euler.quat2euler(quat)
        
    def control_loop(self):
        # Calcular errores en coordenadas polares
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        rho = math.sqrt(dx**2 + dy**2)
        
        alpha = math.atan2(dy, dx) - self.theta
        beta = self.goal_theta - self.theta - alpha
        
        # Normalize
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        beta = math.atan2(math.sin(beta), math.cos(beta))
        
        # llegué a la meta?
        if rho < self.distance_threshold and abs(beta) < self.angle_threshold:
            self.get_logger().info("Goal reached!")
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return
        
        v = self.k_rho * rho
        w = self.k_alpha * alpha + self.k_beta * beta
        
        # Limitar velocidades
        max_linear = 0.3
        max_angular = 1.0
        v = max(min(v, max_linear), -max_linear)
        w = max(min(w, max_angular), -max_angular)
        
        # Publicar comando de control
        cmd_vel = Twist()
        cmd_vel.linear.x = v
        cmd_vel.angular.z = w
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = PointStabilisationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()