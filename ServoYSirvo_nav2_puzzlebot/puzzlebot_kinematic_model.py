import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32 
from rclpy import qos 
import numpy as np 
import transforms3d 
 
#se obtiene con llantas conociendo la velocidad lineal y angular del puzzle bot la odometría
#velocidad de nuestras llantas

class KinematicModelNode(Node): 

    def __init__(self): 

        super().__init__('kinematic_model_node') 

        # Suscripto

        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10) 
        # publisher 

        self.wr_pub = self.create_publisher(Float32,'wr',qos.qos_profile_sensor_data) 
        self.wl_pub = self.create_publisher(Float32,'wl',qos.qos_profile_sensor_data) #  

        self.r=0.05 #radio llanta
        self.L = 0.19 #separación llantas        

        ############ Variables ############### 

        self.w = 0.0 # robot's angular speed [rad/s] 
        self.v = 0.0 #robot's linear speed [m/s] 
        self.wr_msg = Float32() #Ros message to publish the right wheel speed 
        self.wl_msg = Float32() #Ros message to publish the left wheel speed 

 

         

        timer_period = 0.02 # Desired time to update the robot's pose [s] 

        # Create a timer to publish the wheel speeds 

        self.timer = self.create_timer(timer_period, self.timer_callback) 


     

    def timer_callback(self): 

        # Update the robot's pose based on the current velocities 
        wr,wl=self.get_wheel_speeds(self.v,self.w)
        self.wr_msg.data=wr
        self.wl_msg.data=wl  #we get the messages

        #we publish
        self.wr_pub.publish(self.wr_msg)
        self.wl_pub.publish(self.wl_msg)

       
     

    def cmd_vel_callback(self, msg): 

        self.v = msg.linear.x 
        self.w = msg.angular.z 

 

     

    def get_wheel_speeds(self,v,w): 

        # Calculate velocidad llantas 

        wr = (2*v+w*self.L)/(2*self.r)
        wl = (2*v-w*self.L)/(2*self.r)

        print("WR: ",str(wr))
        print("WL: ",str(wl))


        return wr, wl 

 

 

def main(args=None): 

    rclpy.init(args=args) 
    node = KinematicModelNode() 
    try: 

        rclpy.spin(node) 

    except KeyboardInterrupt: 

        pass 

    finally: 

        if rclpy.ok():  # Ensure shutdown is only called once 
            rclpy.shutdown() 
        node.destroy_node() 
        
if __name__ == '__main__': 

    main() 