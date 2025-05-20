#!/usr/bin/env python3
import time
import csv

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt64
from rcl_interfaces.msg import ParameterDescriptor

class LatencySubscriber(Node):
    def __init__(self):
        super().__init__('latency_subscriber')

        self.declare_parameter('topic', 'latency_usec')
        reliability_desc = ParameterDescriptor(
            description="QoS reliability policy: 'best_effort' or 'reliable'",
            additional_constraints="Must be 'best_effort' or 'reliable'"
        )
        self.declare_parameter('reliability', 'best_effort', reliability_desc)
        self.declare_parameter('depth', 10)
        self.declare_parameter('output_csv', 'latency_results.csv')

        reliability_str = self.get_parameter('reliability').get_parameter_value().string_value
        depth = self.get_parameter('depth').get_parameter_value().integer_value
        topic = self.get_parameter('topic').get_parameter_value().string_value

        if reliability_str.lower() == 'reliable':
            reliability = ReliabilityPolicy.RELIABLE
        else:
            reliability = ReliabilityPolicy.BEST_EFFORT

        qos = QoSProfile(
            depth=depth,
            reliability=reliability
        )

        self.count = 0
        self.records = []  # (trial, sent_us, recv_us, latency_us)
        self.last_recv_time = time.time()
        self.prev_count = 0

        self.sub = self.create_subscription(
            UInt64,
            topic,
            self.listener_callback,
            qos_profile=qos)

        self.status_timer = self.create_timer(1.0, self.status_callback)

        self.get_logger().info(
            f'Latency subscriber ready on topic {topic} with reliability {reliability_str} and depth {depth}'
        )

    def listener_callback(self, msg: UInt64):
        now = time.time()
        recv_us = int(now * 1_000_000)
        sent_us = msg.data
        latency_us = recv_us - sent_us

        self.records.append((self.count, sent_us, recv_us, latency_us))
        self.count += 1
        self.last_recv_time = now

    def status_callback(self):
        now = time.time()
        interval_msgs = self.count - self.prev_count
        self.prev_count = self.count

        delta = now - self.last_recv_time
        if delta > 0.1:
            self.get_logger().warn(
                f'No new messages for {delta:.3f}s! Total received: {self.count}')
        else:
            self.get_logger().info(
                f'Received {interval_msgs} msgs in last 1s, total={self.count}')

    def save_csv(self):
        if not self.records:
            self.get_logger().warn('No records to save.')
            return
        filename = self.get_parameter('output_csv').get_parameter_value().string_value
        if not filename.endswith('.csv'):
            filename += '.csv'
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['trial', 'sent_us', 'recv_us', 'latency_us'])
            writer.writerows(self.records)
        self.get_logger().info(f'Saved {len(self.records)} rows to {filename}')

def main(args=None):
    rclpy.init(args=args)
    node = LatencySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown signal received, saving CSV...')
        node.save_csv()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
