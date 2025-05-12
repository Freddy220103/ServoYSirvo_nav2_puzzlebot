import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class OdomToBaseTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_to_base_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        # Timer que llama a broadcast_tf cada 0.1 segundos
        self.timer = self.create_timer(0.1, self.broadcast_tf)

        # Valores de ejemplo (estos podrían venir de tu odometría)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Conversión de yaw a cuaternión
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        # Enviar la transformación
        self.tf_broadcaster.sendTransform(t)

        # Actualización de valores para simular movimiento (esto es solo un ejemplo)
        self.x += 0.01
        self.y += 0.01
        self.yaw += 0.005

def main(args=None):
    rclpy.init(args=args)
    node = OdomToBaseTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()