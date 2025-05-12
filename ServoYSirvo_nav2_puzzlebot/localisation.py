import rclpy 
from rclpy.node import Node 
from nav_msgs.msg import Odometry 
from std_msgs.msg import Float32 
from rclpy import qos 
import numpy as np 
import transforms3d 
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class Localisation(Node): 
    def __init__(self): 
        super().__init__('localisation') 
        
        # Suscriptores
        self.wr_sub = self.create_subscription(Float32, 'wr', self.wr_callback, qos.qos_profile_sensor_data) 
        self.wl_sub = self.create_subscription(Float32, 'wl', self.wl_callback, qos.qos_profile_sensor_data) 
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publicador
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)      

        # Parámetros físicos
        self.r = 0.05  # radio ruedas (m)
        self.L = 0.19  # separación entre ruedas (m)

        # Estado inicial
        self.w = 0.0  # velocidad angular
        self.v = 0.0  # velocidad lineal
        self.x = 0.0  # posición x
        self.y = 0.0  # posición y
        self.theta = 0.0  # orientación (yaw)
        self.wr = 0.0  # velocidad rueda derecha
        self.wl = 0.0  # velocidad rueda izquierda

        self.prev_time_ns = self.get_clock().now().nanoseconds  

        # Inicializar matriz de covarianza
        self.Sigma = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.1],
            [0.0, 0.1, 1.0]
        ])
        self.Q = np.array([
            [0.000567, -0.00065, -0.00163],
            [-0.00065, 0.002631, 0.003897],
            [-0.00163, 0.003897, 0.011256]
        ])

        # Timer
        timer_period = 0.02 
        self.timer = self.create_timer(timer_period, self.timer_callback) 

    def wr_callback(self, msg): 
        self.wr = msg.data 

    def wl_callback(self, msg): 
        self.wl = msg.data 

    def timer_callback(self): 
        v, w = self.get_robot_vel(self.wr, self.wl) 
        self.update_pose(v, w)  
        odom_msg = self.fill_odom_message(self.x, self.y, self.theta) 
        self.odom_pub.publish(odom_msg) 

    def get_robot_vel(self, wr, wl): 
        v = self.r * (wr + wl) / 2.0 
        w = self.r * (wr - wl) / self.L 
        return v, w 

    def update_pose(self, v, w): 
        dt = (self.get_clock().now().nanoseconds - self.prev_time_ns) * 1e-9  
        self.prev_time_ns = self.get_clock().now().nanoseconds  

        # Movimiento
        dx = v * np.cos(self.theta) * dt
        dy = v * np.sin(self.theta) * dt
        dtheta = w * dt

        self.x += dx
        self.y += dy
        self.theta += dtheta
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))  # Normalize angle

        # Actualiza la matriz de covarianza (EKF Predict Step)
        H = np.array([
            [1.0, 0.0, -v * dt * np.sin(self.theta)],
            [0.0, 1.0,  v * dt * np.cos(self.theta)],
            [0.0, 0.0, 1.0]
        ])
        self.Sigma = H @ self.Sigma @ H.T + self.Q

    def fill_odom_message(self, x, y, yaw): 
        odom_msg = Odometry()  
        odom_msg.header.stamp = self.get_clock().now().to_msg()  
        odom_msg.header.frame_id = 'odom' 
        odom_msg.child_frame_id = 'base_link'  

        odom_msg.pose.pose.position.x = x  
        odom_msg.pose.pose.position.y = y  
        odom_msg.pose.pose.position.z = 0.0   

        quat = transforms3d.euler.euler2quat(0, 0, yaw)  
        odom_msg.pose.pose.orientation.w = quat[0] 
        odom_msg.pose.pose.orientation.x = quat[1] 
        odom_msg.pose.pose.orientation.y = quat[2] 
        odom_msg.pose.pose.orientation.z = quat[3] 
        
        # Asigna la covarianza al mensaje de odometría (solo para pose)
        cov = np.zeros((6, 6))
        cov[0, 0] = self.Sigma[0, 0]
        cov[0, 1] = self.Sigma[0, 1]
        cov[1, 0] = self.Sigma[1, 0]
        cov[1, 1] = self.Sigma[1, 1]
        cov[0, 5] = self.Sigma[0, 2]
        cov[5, 0] = self.Sigma[2, 0]
        cov[1, 5] = self.Sigma[1, 2]
        cov[5, 1] = self.Sigma[2, 1]
        cov[5, 5] = self.Sigma[2, 2]
        odom_msg.pose.covariance = cov.flatten().tolist()


        return odom_msg 

def main(args=None): 
    rclpy.init(args=args) 
    node = Localisation() 
    try: 
        rclpy.spin(node) 
    except KeyboardInterrupt: 
        pass 
    finally: 
        if rclpy.ok(): 
            rclpy.shutdown() 
        node.destroy_node() 

if __name__ == '__main__': 
    main()
