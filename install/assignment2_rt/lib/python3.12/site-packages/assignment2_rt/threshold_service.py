import rclpy
from rclpy.node import Node
from custom_message.srv import Threshold
 

class ThresholdServer(Node):

    def __init__(self):
        super().__init__('threshold_server') # name of the node
        self.srv = self.create_service(Threshold, 'get_threshold', self.callback) # name of the service

    def ask_float(self, prompt):
        while True:
            value = input(prompt)
            try:
                return float(value)
            except ValueError:
                print("Valore non valido, inserisci un numero (float).")

    def callback(self, request, response):
        print("\n It is possible to set a new threshold for obstacle avoidance.")
        threshold = self.ask_float("Insert threshold: ")
        response.threshold = threshold
        print(f"Threshold set to: {threshold}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ThresholdServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
