#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from px4_msgs.msg import SensorGps
from px4_msgs.msg import VehicleLocalPosition
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
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        px4_gps_topic = 'fmu/out/vehicle_gps_position'
        px4_local_pos_topic = 'fmu/out/vehicle_local_position'
        self.get_logger().info(f'Waiting for messages from topics: {px4_gps_topic}, {px4_local_pos_topic}')
        ret, msg = wait_for_message(SensorGps, self, px4_gps_topic, qos_profile=qos_profile)
        if not ret:
            self.get_logger().error(f'Failed to get message from topic {px4_gps_topic}')
            self.destroy_node()
            return
        ret, msg = wait_for_message(VehicleLocalPosition, self, px4_local_pos_topic, qos_profile=qos_profile)
        if not ret:
            self.get_logger().error(f'Failed to get message from topic {px4_local_pos_topic}')
            self.destroy_node()
            return
        # self.get_logger().info(f'Received messages from topics: {px4_gps_topic}, {px4_local_pos_topic}. Creating subscriptions')

        self.subscription = self.create_subscription(
            SensorGps,
            px4_gps_topic,
            self.gps_callback,
            qos_profile
        )
        
        # Collecting the heading and local z at launch as well.
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            px4_local_pos_topic,
            self.local_pos_callback,
            qos_profile=qos_profile
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

        # self.get_logger().info(f'GPS Fix Time: {self.total_time}')
        # self.get_logger().info(f'Origin Latitude: {self.origin_lat}')
        # self.get_logger().info(f'Origin Longitude: {self.origin_lon}')
        # self.get_logger().info(f'Origin Altitude: {self.origin_alt}')

        self.status = 'running'
        self.m = 3
        self.gps_data = []
        self.utm_data = []
        self.gps_altitues = []
        self.headings = []
        self.local_altitudes = []
        self.launch_gps = np.zeros(3)
        self.start_time = time.time()

        # self.get_logger().info('GPS Fix Node Initialized')

        # Use a timer instead of blocking spin loop
        self.collection_timer = self.create_timer(0.1, self.check_collection_status)  # 100ms

    def check_collection_status(self):
        """Timer callback to check if collection time is complete"""
        time_elapsed = time.time() - self.start_time
        
        if time_elapsed > self.total_time and self.status == 'running':
            self.status = 'data'
            self.collection_timer.cancel()
            self.complete_gps_processing()
        else:
            self.get_logger().info(f'Time left: {self.total_time - time_elapsed:.2f} seconds', throttle_duration_sec=1.0)

    def complete_gps_processing(self):
        """Complete GPS data processing once collection is done"""
        if len(self.gps_data) == 0:
            self.get_logger().error('No GPS data collected!')
            return

        self.median_launch_gps, self.mean_launch_gps = self.get_median_gps_coordinates()
        self.median_heading = self.get_median_heading()
        self.median_altitude = self.get_median_altitude()
        self.get_logger().info(f'Median Heading: {self.median_heading:.2f}')
        self.get_logger().info(f'Median Altitude: {self.median_altitude:.2f}')
        self.get_logger().info(f'Median Launch Location: Latitude: {self.median_launch_gps[0]:.7f}, Longitude: {self.median_launch_gps[1]:.7f}')
        self.get_logger().info(f'Mean Launch Location: Latitude: {self.mean_launch_gps[0]:.7f}, Longitude: {self.mean_launch_gps[1]:.7f}')
        # self.launch_gps = self.mean_launch_gps
        self.launch_gps = self.median_launch_gps
        #filtered_altitudes = np.array(self.gps_altitues)
        #filtered_altitudes = filtered_altitudes[self.reject_outliers(filtered_altitudes) < self.m]
        # self.get_logger().info(f'Filtered Altitude data size: {filtered_altitudes.shape}')
        #self.launch_gps[2] = np.mean(filtered_altitudes)
        self.launch_gps[2] = self.median_heading
        gps_params = rclpy.parameter.Parameter('launch_gps', rclpy.Parameter.Type.DOUBLE_ARRAY, [self.launch_gps[0], self.launch_gps[1], self.launch_gps[2], self.median_altitude])
        self.set_parameters([gps_params])
        # with open('launch_gps', 'w') as f:
        #     f.write(f'{self.median_launch_gps[0]:.9f} {self.median_launch_gps[1]:.9f} {self.launch_gps[2]:.2f}')
        #     f.write('\n')
        #     f.write(f'{self.mean_launch_gps[0]:.9f} {self.mean_launch_gps[1]:.9f} {self.launch_gps[2]:.2f}')

        # geo_transformer = GeoLocalTransform(self.origin_lat, self.origin_lon, self.origin_alt)
        # xyz = geo_transformer.Forward(self.launch_gps[0], self.launch_gps[1], self.launch_gps[2])
        # self.get_logger().info(f'Relative Launch Location: X: {xyz[0]:.2f}, Y: {xyz[1]:.2f}, Z: {xyz[2]:.2f}')
        # self.get_logger().info(f'Distance from Origin xy: {np.linalg.norm(xyz[:2]):.2f}')
        self.status = 'done'
        self.destroy_subscription(self.subscription)
        self.destroy_subscription(self.local_pos_sub)

    def reject_outliers(self, data):
        d = np.abs(data - np.median(data))
        mdev = np.median(d)
        self.get_logger().info(f'Median Deviation: {mdev}')
        self.get_logger().info(f'Max Deviation: {np.max(d)}')
        s = d/mdev if mdev else np.zeros(len(d))
        return s

    def get_median_gps_coordinates(self):
        utm_data = np.array(self.utm_data)
        utm_data_x = np.array(utm_data[self.reject_outliers(utm_data[:,0]) < self.m, 0])
        utm_data_y = np.array(utm_data[self.reject_outliers(utm_data[:,1]) < self.m, 1])

        median = np.array([np.median(utm_data_x), np.median(utm_data_y), 0.])
        idx = np.argmin(np.linalg.norm(utm_data - median, axis=1))
        # median_gps = self.gps_data[idx]
        median_gps = np.zeros(3)
        median_gps[:2] = self.gps_data[idx]

        mean_utm_x = np.mean(utm_data_x)
        mean_utm_y = np.mean(utm_data_y)
        geo_transformer = GeoLocalTransform()
        mean_gps = geo_transformer.UTMReverse(mean_utm_x, mean_utm_y, median_gps[0], median_gps[1])
        return median_gps, mean_gps


    def gps_callback(self, msg):
        self.received_gps = True
        if self.status in ['done', 'data']:
            # Stop collecting once timer expires or processing is complete
            # self.publish_launch_gps.publish(Point(x=self.launch_gps[0], y=self.launch_gps[1], z=self.launch_gps[2]))
            return

        elif msg.fix_type >= 3 and self.status == 'running':
            self.gps_altitues.append(msg.alt)
            true_gps = np.array([msg.lat, msg.lon], dtype=np.float64)*1e-7
            self.gps_data.append(true_gps)
            transformer = GeoLocalTransform()
            xyz = transformer.UTMForward(true_gps[0], true_gps[1])
            self.utm_data.append(xyz)

        else:
            self.get_logger().warning('No GPS Fix', once=True)

    def get_median_heading(self):
        # return np.median(self.headings)
        headings_data = np.array(self.headings)
        headings_data = headings_data[self.reject_outliers(headings_data) < self.m]
        return np.median(np.array(headings_data))

    def get_median_altitude(self):
        altitudes_data = np.array(self.local_altitudes)
        altitudes_data = altitudes_data[self.reject_outliers(altitudes_data) < self.m]
        return np.median(np.array(altitudes_data))

    def local_pos_callback(self, msg):
        self.received_local_pos = True
        if self.status == 'running':
            self.headings.append(msg.heading)
            self.local_altitudes.append(msg.z)


def main(args=None):
    rclpy.init(args=args)
    gps_fix_node = GPSFix()

    rclpy.spin(gps_fix_node)
    gps_fix_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
