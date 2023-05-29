import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from proyecto_interfaces.srv import StartNavigationTest
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3


class Navigation_test(Node):

    def __init__(self):
        super().__init__('navigation_test')
        # self.subscription_camera_1 = self.create_subscription(Image,'map/camera_1',self.callback_camera_1, 10)
        # self.subscription_camera_2 = self.create_subscription(Image,'map/camera_2',self.callback_camera_2, 10)
        self.srv = self.create_service(StartNavigationTest, 'group_12/start_navigation_test_srv', self.callback_service)
        self.publisher_pos_final = self.create_publisher(Float32MultiArray, 'posicion_final', 10)
        print('Servicio para la prueba de navegación listo')
        

    def callback_service(self, request, response):
        print('Se ha llamado al servicio para la prueba de navegación')
        x_goal = request.x
        y_goal = request.y
        print("El punto de destino es: x: " + str(x_goal) + ", y: "+ str(y_goal))
        response.answer = "El servicio para la prueba de navegación ha sido aprobado"
        self.msg = Float32MultiArray()
        self.msg.data = [float(x_goal), float(y_goal)]
        self.publisher_pos_final.publish(self.msg)
        return response

        

def main(args=None):
    rclpy.init(args=args)
    navigation_test = Navigation_test()
    rclpy.spin(navigation_test)
    navigation_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()