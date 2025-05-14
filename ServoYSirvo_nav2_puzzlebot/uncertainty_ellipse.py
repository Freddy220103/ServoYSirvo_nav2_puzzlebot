#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf_transformations

class UncertaintyEllipse(Node):
    def __init__(self):
        super().__init__('uncertainty_ellipse_node')

        self.publisher = self.create_publisher(Marker, 'uncertainty_ellipse', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Initial pose [x, y, theta]
        self.mu = np.array([0.0, 0.0, 0.0])

        # Initial covariance matrix Σ₀
        self.Sigma = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.1],
            [0.0, 0.1, 1.0]
        ])

        # Control inputs
        self.v = 1.0    # Linear velocity (m/s)
        self.w = 1.0    # Angular velocity (rad/s)
        self.dt = 1.0   # Time step (s)

        # Process noise Q
        self.Q = np.array([
            [0.01, 0.01, 0.01],
            [0.01, 0.01, 0.01],
            [0.01, 0.01, 0.03]
        ])

    def timer_callback(self):
        θ = self.mu[2]
        Δt = self.dt

        # Step 1: Predict new pose
        dx = self.v * np.cos(θ) * Δt
        dy = self.v * np.sin(θ) * Δt
        dθ = self.w * Δt
        self.mu += np.array([dx, dy, dθ])

        # Step 2: Calculate Jacobian H
        H = np.array([
            [1.0, 0.0, -self.v * Δt * np.sin(θ)],
            [0.0, 1.0,  self.v * Δt * np.cos(θ)],
            [0.0, 0.0, 1.0]
        ])

        # Step 3: Covariance update
        self.Sigma = H @ self.Sigma @ H.T + self.Q

        # Step 4: Visualize ellipse
        self.publish_ellipse_marker()

    def publish_ellipse_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ellipse"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(self.mu[0])
        marker.pose.position.y = float(self.mu[1])
        marker.pose.position.z = 0.0

        # Extract 2x2 covariance for (x, y)
        cov2d = self.Sigma[0:2, 0:2]

        # Eigen decomposition for ellipse axes
        eigvals, eigvecs = np.linalg.eigh(cov2d)
        order = eigvals.argsort()[::-1]
        eigvals = eigvals[order]
        eigvecs = eigvecs[:, order]

        angle = np.arctan2(eigvecs[1, 0], eigvecs[0, 0])
        q = tf_transformations.quaternion_from_euler(0, 0, angle)

        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        # Scale the axes (2σ for 95% confidence)
        marker.scale.x = 2.0 * np.sqrt(eigvals[0])
        marker.scale.y = 2.0 * np.sqrt(eigvals[1])
        marker.scale.z = 0.01  # flat ellipse

        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = UncertaintyEllipse()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
