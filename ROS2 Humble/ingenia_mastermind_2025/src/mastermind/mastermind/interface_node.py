# Archivo: interface_node.py
# Versión: V1.0
# Autores: Miguel Ángel García de Vicente


import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class InterfaceNode(Node):

    def __init__(self):
        super().__init__('interface')


        # Esto es solo un código de prueba para comprobar que funciona ->

        self.publisher_ = self.create_publisher(String, 'topicazo', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        # self.get_logger().info("TEST PYTHON")

    # Esto es parte del código de prueba ->
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
