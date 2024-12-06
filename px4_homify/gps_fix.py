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
        self.m = 3
        self.gps_data = []
        self.utm_data = []
        self.gps_altitues = []
        self.launch_gps = np.zeros(3)
        self.start_time = time.time()

        while self.status == 'running':
            rclpy.spin_once(self)

        self.median_launch_gps, self.mean_launch_gps = self.get_median_gps_coordinates()
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
        with open('launch_gps', 'w') as f:
            f.write(f'{self.median_launch_gps[0]:.9f} {self.median_launch_gps[1]:.9f} {self.launch_gps[2]:.2f}')
            f.write('\n')
            f.write(f'{self.mean_launch_gps[0]:.9f} {self.mean_launch_gps[1]:.9f} {self.launch_gps[2]:.2f}')

        geo_transformer = GeoLocalTransform(self.origin_lat, self.origin_lon, self.origin_alt)
        xyz = geo_transformer.Forward(self.launch_gps[0], self.launch_gps[1], self.launch_gps[2])
        self.get_logger().info(f'Relative Launch Location: X: {xyz[0]:.2f}, Y: {xyz[1]:.2f}, Z: {xyz[2]:.2f}')
        self.get_logger().info(f'Distance from Origin xy: {np.linalg.norm(xyz[:2]):.2f}')
        self.status = 'done'

    def reject_outliers(self, data):
        d = np.abs(data - np.median(data))
        mdev = np.median(d)
        self.get_logger().info(f'Median Deviation: {mdev}')
        self.get_logger().info(f'Max Deviation: {np.max(d)}')
        s = d/mdev if mdev else np.zeros(len(d))
        return s

    def get_median_gps_coordinates(self):
        utm_data = np.array(self.utm_data)
        self.get_logger().info(f'Initial UTM data size: {utm_data.shape}')
        utm_data_x = np.array(utm_data[self.reject_outliers(utm_data[:,0]) < self.m, 0])
        utm_data_y = np.array(utm_data[self.reject_outliers(utm_data[:,1]) < self.m, 1])
        self.get_logger().info(f'Filtered UTM x data size: {utm_data_x.shape}')
        self.get_logger().info(f'Filtered UTM y data size: {utm_data_y.shape}')

        median = np.array([np.median(utm_data_x), np.median(utm_data_y), 0.])
        idx = np.argmin(np.linalg.norm(utm_data - median, axis=1))
        self.get_logger().info(f'Idx: {idx}')
        # median_gps = self.gps_data[idx]
        median_gps = np.zeros(3)
        median_gps[:2] = self.gps_data[idx]

        mean_utm_x = np.mean(utm_data_x)
        mean_utm_y = np.mean(utm_data_y)
        self.get_logger().info(f'Mean UTM X: {mean_utm_x}, Mean UTM Y: {mean_utm_y}')
        geo_transformer = GeoLocalTransform()
        mean_gps = geo_transformer.UTMReverse(mean_utm_x, mean_utm_y, median_gps[0], median_gps[1])
        return median_gps, mean_gps


    def gps_callback(self, msg):
        if self.status == 'done':
            self.publish_launch_gps.publish(Point(x=self.launch_gps[0], y=self.launch_gps[1], z=self.launch_gps[2]))

        elif msg.fix_type >= 3:
            self.gps_altitues.append(msg.alt)
            true_gps = np.array([msg.lat, msg.lon], dtype=np.float64)*1e-7
            self.gps_data.append(true_gps)
            transformer = GeoLocalTransform()
            xyz = transformer.UTMForward(true_gps[0], true_gps[1])
            self.utm_data.append(xyz)

            if time.time() - self.start_time > self.total_time:
                self.status = 'data'
            else:
                self.get_logger().info(f'Time left: {self.total_time - (time.time() - self.start_time):.2f} seconds')
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
