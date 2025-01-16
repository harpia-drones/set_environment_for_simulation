import rclpy 
from rclpy.node import Node 

class MyClassNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("node_name") # MODIFY NAME


def main(args=None):
    rclpy.init(args=args)
    node = MyClassNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()