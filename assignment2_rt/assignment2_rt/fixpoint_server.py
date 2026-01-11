import rclpy
from rclpy.node import Node
from custom_message.srv import FixPoint
 

class FixpointServer(Node):

    def __init__(self):
        super().__init__('fixpoint_server') # name of the node
        self.srv = self.create_service(FixPoint, 'get_point', self.callback) # name of the service

    def ask_float(self, prompt):
        while True:
            value = input(prompt)
            try:
                return float(value)
            except ValueError:
                print("Valore non valido, inserisci un numero (float).")

    def callback(self, request, response):
        print("\n It is possible to set a new fix point to get the distance from.")
        x = self.ask_float("Insert x value: ")
        y = self.ask_float("Insert y value: ")
        response.x = x
        response.y = y
        print(f"Fixed point set to: x={response.x}, y={response.y}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FixpointServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
