from math import e
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rosidl_runtime_py.utilities import get_interface

class Introspector(Node):
    def __init__(self):
        super().__init__('introspector')
        self.get_logger().info('Introspector node has been created.')
        self.list_topics()
      
    def list_topics(self):
        topics = self.get_topic_names_and_types()
        self.get_logger().info('Available topics:')
        for topic in topics:
            try:
                self.get_logger().info(f'Topic: {topic[0]}, Type: {topic[1]}')
                self.get_logger().info(f'Interface: {get_interface(topic[1][0]).get_fields_and_field_types()}')
            except Exception as e:
                self.get_logger().error(f'Error retrieving interface for topic {topic[0]}: {e}')
        self.get_logger().info('Introspection complete.')

def main():
    rclpy.init()
    node = Introspector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()