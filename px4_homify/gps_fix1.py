#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorGps
import time
import numpy as np
from geolocaltransform.geolocaltransform import GeoLocalTransform

from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseArray
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class GPSFix(Node):
    def __init__(self):
        super().__init__('gps_fix')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            SensorGps,
            'fmu/out/vehicle_gps_position',
            self.gps_callback,
            qos_profile
        )

        self.publish_launch_gps = self.create_publisher(Point, 'launch_gps', qos_profile)

        # Get the parameters
        self.declare_parameter('gps_fix_time', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('origin_lat', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('origin_lon', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('origin_alt', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('launch_gps', rclpy.Parameter.Type.DOUBLE_ARRAY)

        self.total_time = self.get_parameter('gps_fix_time').get_parameter_value().double_value
        self.origin_lat = self.get_parameter('origin_lat').get_parameter_value().double_value
        self.origin_lon = self.get_parameter('origin_lon').get_parameter_value().double_value
        self.origin_alt = self.get_parameter('origin_alt').get_parameter_value().double_value

        self.get_logger().info(f'GPS Fix Time: {self.total_time}')
        self.get_logger().info(f'Origin Latitude: {self.origin_lat}')
        self.get_logger().info(f'Origin Longitude: {self.origin_lon}')
        self.get_logger().info(f'Origin Altitude: {self.origin_alt}')

        self.status = 'running'
        self.mean_utm = np.zeros(3)
        self.median_utm = np.zeros(3)
        self.median_gps = np.zeros(3)
        self.mean_gps = np.zeros(3)

        self.gps_data = np.zeros((0, 2))
        self.utm_data = np.zeros((0, 3))

        self.launch_gps = np.zeros(3)

        self.start_time = time.time()

        while self.status == 'running':
            rclpy.spin_once(self)

        self.get_gps_utm_stats()
        self.get_logger().info(f'Median Launch Location: Latitude: {self.median_launch_gps[0]:.7f}, Longitude: {self.median_launch_gps[1]:.7f}')
        self.get_logger().info(f'Mean Launch Location: Latitude: {self.mean_launch_gps[0]:.7f}, Longitude: {self.mean_launch_gps[1]:.7f}')
        # self.launch_gps = self.mean_launch_gps
        self.launch_gps = self.median_launch_gps
        filtered_altitudes = np.array(self.gps_altitues)
        filtered_altitudes = filtered_altitudes[self.reject_outliers(filtered_altitudes) < self.m]
        # self.get_logger().info(f'Filtered Altitude data size: {filtered_altitudes.shape}')
        self.launch_gps[2] = np.mean(filtered_altitudes)
        gps_params = rclpy.parameter.Parameter('launch_gps', rclpy.Parameter.Type.DOUBLE_ARRAY, [self.launch_gps[0], self.launch_gps[1], self.launch_gps[2]])
        self.set_parameters([gps_params])
        with open('data/launch_gps', 'w') as f:
            f.write(f'{self.median_launch_gps[0]:.9f} {self.median_launch_gps[1]:.9f} {self.launch_gps[2]:.2f}')
            f.write('\n')
            f.write(f'{self.mean_launch_gps[0]:.9f} {self.mean_launch_gps[1]:.9f} {self.launch_gps[2]:.2f}')

        geo_transformer = GeoLocalTransform(self.origin_lat, self.origin_lon, self.origin_alt)
        xyz = geo_transformer.Forward(self.launch_gps[0], self.launch_gps[1], self.launch_gps[2])
        self.get_logger().info(f'Relative Launch Location: X: {xyz[0]:.2f}, Y: {xyz[1]:.2f}, Z: {xyz[2]:.2f}')
        self.get_logger().info(f'Distance from Origin xy: {np.linalg.norm(xyz[:2]):.2f}')
        self.status = 'done'

        # Write self.utm_data to a file
        with open('data/utm_data', 'w') as f:
            mean_utm = np.mean(self.utm_data, axis=0)
            for data in self.utm_data:
                data -= mean_utm
                f.write(f'{data[0]:.3f} {data[1]:.3f} {data[2]:.3f}\n')

        with open('data/launch_utm', 'w') as f:
            f.write(f'{xyz[0]:.3f} {xyz[1]:.3f} {xyz[2]:.3f}\n')

    def reject_outliers(self, data):
        q1 = np.percentile(data, 25)
        q3 = np.percentile(data, 75)
        iqr = q3 - q1
        lower_bound = q1 - 1.2 * iqr
        upper_bound = q3 + 1.2 * iqr
        return data[(data >= lower_bound) & (data <= upper_bound)]


    def get_gps_utm_stats(self):
        self.get_logger().info(f'Initial UTM data size: {self.utm_data.shape}')
        filtered_data = [self.reject_outliers(self.utm_data[:, i]) for i in range(3)]
        self.get_logger().info(f'Filtered UTM data size: {len(filtered_data[0])}')
        self.median_utm = np.array([np.median(data) for data in filtered_data])
        median_idx = np.array([np.min(data) for i, data in enumerate(self.utm_data.T)])
        # median_idx = np.array([np.where(data == self.median_utm[i]) for i, data in enumerate(self.utm_data.T)])
        self.get_logger().info(f'Median UTM: {self.median_utm}')
        self.get_logger().info(f'Median Index: {median_idx}')
        # self.meadian_gps = np.array([self.gps_data[idx[0][0]] for idx in median_idx])
        self.mean_utm = np.array([np.mean(data) for data in filtered_data])

    def gps_callback(self, msg):
        if self.status == 'done':
            self.publish_launch_gps.publish(Point(x=self.launch_gps[0], y=self.launch_gps[1], z=self.launch_gps[2]))

        elif msg.fix_type >= 3:
            true_gps = np.array([msg.lat, msg.lon], dtype=np.float64)*1e-7
            self.gps_data = np.append(self.gps_data, true_gps)
            transformer = GeoLocalTransform()
            xyz = transformer.GetUTMForward(true_gps[0], true_gps[1])
            xyz[2] = msg.alt
            self.utm_data = np.vstack((self.utm_data, xyz))

            time_elapsed = time.time() - self.start_time
            if time_elapsed >= self.total_time:
                self.status = 'data'
            else:
                self.get_logger().info(f'Time left: {self.total_time - time_elapsed:.2f}')
        else:
            self.get_logger().info('No GPS Fix')

def main(args=None):
    rclpy.init(args=args)
    gps_fix_node = GPSFix()

    while rclpy.ok():
        rclpy.spin(gps_fix_node)
    gps_fix_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
