import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String

class CoordinateListener(Node):
    def __init__(self):
        super().__init__('coordinate_listener')
        
        # Sottoscrizione al topic delle coordinate
        self.coord_sub = self.create_subscription(
            Point,
            '/coordinates',
            self.coord_callback,
            10
        )
        
        # Publisher per inviare un messaggio quando le coordinate vengono ricevute

        #Manda il tutto a control node, da fixare
        self.msg_pub = self.create_publisher(String, '/output', 10)

    def coord_callback(self, msg):
        # Ricevute le coordinate, prepara e invia il messaggio
        received_coordinates = f"Received coordinates: x={msg.x}, y={msg.y}, z={msg.z}"
        self.get_logger().info(received_coordinates)

        # Creare un messaggio String e pubblicarlo
        output_msg = String()
        output_msg.data = received_coordinates
        self.msg_pub.publish(output_msg)

def main():
    rclpy.init()
    coordinate_listener_node = CoordinateListener()
    rclpy.spin(coordinate_listener_node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
