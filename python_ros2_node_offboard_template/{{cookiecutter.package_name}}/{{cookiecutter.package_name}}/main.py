import random
import time
import numpy as np
from functools import partial

import rclpy
from rclpy.node import Node

from {{cookiecutter.custom_ros2_msgs_name}}.msg import NotifyVehicles, TargetAngle


class Monitor(Node):

    def __init__(self):
        super().__init__('sync_monitor')

        self.angle_timeout = rclpy.Duration(seconds=0.3)

        self.notify_delay_sub = self.create_subscription(TargetAngle, '/monitor/notify_angle', self.notify_angle_cb, 10)
        self.notify_vehicles_timer = self.create_timer(1, self.notfy_vehicles_timer_cb)

        self.current_vehicle_map = {}
        self.vehicle_theta_map = {}

        self.get_logger().info("Initialised")
    
    def notify_vehicles_timer_cb(self):
        curr_time = self.get_clock().now().to_msg()

        vehicles = self.__get_current_vehicle_namespaces()
        self.current_vehicle_map = dict(enumerate(sorted(vehicles)))

        msg = NotifyVehicles()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.total_vehicles = len(self.current_vehicle_map)

        s_msg = TargetAngle()
        for vname, id in self.current_vehicle_map.items():
            msg.vehicle_id = id

            topic = f'/{vname}/notify_vehicles'
            pub = self.create_publisher(NotifyVehicles, topic, 10)
            pub.publish(msg)
            self.get_logger().info(f'Sent Vehicle Notfication to {topic}')

            if vname in self.vehicle_theta_map:
                arr_time, theta = self.vehicle_theta_map[vname]
                if curr_time - arr_time > self.angle_timeout:
                    continue
                

    def notify_angle_cb(self, msg):
        self.get_logger().info(f'Delay received from {msg.vehicle_id}, with delay {msg.theta}')
        self.vehicle_theta_map[msg.vehicle_id] = (msg.time, msg.theta)


    def __get_current_vehicle_namespaces(self):
        topic_list = self.get_topic_names_and_types()
        namespaces = set()
        # self.get_logger().info('Found the following topics:')
        for topic_name, _ in topic_list:
            # self.get_logger().info(topic_name)
            if 'mavros' in topic_name:
                name = topic_name.split('/')[1]
                if name == 'mavros':
                    name = ''
                namespaces.add(name)
        self.get_logger().info(f'Found {len(namespaces)} namespaces: {",".join(namespaces)}')
        return namespaces

def main(args=None):
    rclpy.init(args=args)
    mon = Monitor()
    rclpy.spin(mon)
    mon.destroy_node()
    rclpy.shutdown()