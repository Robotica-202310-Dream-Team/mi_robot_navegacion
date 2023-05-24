import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from proyecto_interfaces.srv import StartNavigationTest
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry


class Navigation_test(Node):

    def __init__(self):
        super().__init__('navigation_test')
        self.subscription_camera_1 = self.create_subscription(Image,'map/camera_1',self.callback_camera_1, 10)
        self.subscription_camera_2 = self.create_subscription(Image,'map/camera_2',self.callback_camera_2, 10)
        self.srv = self.create_service(StartNavigationTest, 'group_12/start_navigation_test_srv', self.read_txt_callback)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'robot_cmdVel', 10)
        self.msg = Float32MultiArray()
        self.msg.data = [0.0, 0.0, 0.0, 0.0]
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                        depth=1)
        self.subPose = self.create_subscription(Odometry, 'camera/pose/sample' ,self.subscriber_callback_pose, qos_profile=qos_policy)

        print('Servicio para la prueba de navegación listo')
        

    def read_txt_callback(self, request, response):
        print('Se ha llamado al servicio para la prueba de navegación')
        x_goal = request.x
        y_goal = request.y
        print("El punto de destino es: x: " + str(x_goal) + ", y: "+ str(y_goal))
        response.answer = "El servicio para la prueba de navegación ha sido aprobado"
        return response


    def subscriber_callback_pose(self, msg):
        self.pos_x = round (msg.pose.pose.position.x*100,2 )
        self.pos_y = round (msg.pose.pose.position.y *100,2 )
        self.pos_z = round (msg.pose.pose.position.z *100,2 )
        print (f"x = {self.pos_x} y = {self.pos_y} z = {self.pos_y}")
        

def main(args=None):
    rclpy.init(args=args)
    navigation_test = Navigation_test()
    rclpy.spin(navigation_test)
    navigation_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()