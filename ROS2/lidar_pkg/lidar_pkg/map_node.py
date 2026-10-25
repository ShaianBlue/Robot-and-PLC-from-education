import numpy as np
import math
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros

class MapBuild(Node):

    def __init__(self):
        super().__init__('frame_pubulisher')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # self.tfsubscription = self.create_subscription(
        #     TransformStamped,
        #     '/tf',
        #     self.tf_callback,
        #     10)
        self.timer = self.create_timer(1.0, self.tf_callback)
        self.flag = False
        self.y = 0
        self.x = -0.032
        self.yaw = 0




        self.map_resolution = 0.05
        self.map_data = [-1] * 400 * 200
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = 'map'
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_msg.info.width = 400
        self.map_msg.info.height = 200
        self.map_msg.info.resolution = 0.05
        self.map_msg.info.origin.position.x = -10.00
        self.map_msg.info.origin.position.y = -5.00
        self.map_msg.info.origin.orientation.w = 1.0
        self.map_msg.data  = [-1] * self.map_msg.info.width * self.map_msg.info.height


        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Change 'lidar_topic' to your LIDAR topic name
            self.callback,
            10)

        self.publisher_ = self.create_publisher(OccupancyGrid, 'mymap', 10)
        self.timer = self.create_timer(0.5, self.publish_map)

    def tf_callback(self):
        if not self.flag:
            try:
                t = self.tf_buffer.lookup_transform( 'odom', 'base_footprint', rclpy.time.Time())
                self.y =  t.transform.translation.y
                self.x =  t.transform.translation.x -0.032
                rz = t.transform.rotation.z
                rw = t.transform.rotation.w
                rx = 0
                ry = 0
                self.yaw =    math.atan2(2 * (rw * rz + rx * ry), 1 - 2 * (rz * rz + ry * ry))
                self.get_logger().info(
                        f'{self.x} {self.y} {self.yaw}')
                self.flag = True
            except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform base_footprint to odom: {ex}')
                    return

    def callback(self,msg):
        if self.flag:
            for i, range_value in enumerate(msg.ranges):
            
                self.get_logger().info(
                        f'{i}{range_value}')
                angle = msg.angle_min + i * msg.angle_increment
                j = 0.0
                while j < range_value and j < msg.range_max:
                    tx = j * np.cos(angle) 
                    ty = j * np.sin(angle) 
                    x =  ((math.cos(self.yaw) * tx) - (math.sin(self.yaw) * ty) + self.x) / self.map_resolution + self.map_msg.info.width / 2
                    y =  ((math.sin(self.yaw) * tx) + (math.cos(self.yaw) * ty) + self.y) / self.map_resolution + self.map_msg.info.height / 2
                    x = int(x)
                    y = int(y)
                    j += 0.01
                    
                    if x + y*400 > 400 * 200 -1 or x + y*400 < 0:
                        pass
                    else :
                        # self.map_data[y*400 + x] = 100
                        self.map_msg.data[y*400 + x] = 0
                
                if range_value > msg.range_max or range_value < msg.range_min:
                    pass 
                else:
                    tx = range_value * np.cos(angle) 
                    ty = range_value * np.sin(angle) 
                    x =  ((math.cos(self.yaw) * tx) - (math.sin(self.yaw) * ty) + self.x) / self.map_resolution + self.map_msg.info.width / 2
                    y =  ((math.sin(self.yaw) * tx) + (math.cos(self.yaw) * ty) + self.y) / self.map_resolution + self.map_msg.info.height / 2
                    x = int(x)
                    y = int(y)

                    if x + y*400 > 400 * 200 -1 or x + y*400 < 0:
                        pass
                    else :
                        # self.map_data[y*400 + x] = 100
                        self.map_msg.data[y*400 + x] = 100
                self.get_logger().info(
                        f'{x}{y}')

            self.map_msg.header.stamp  = msg.header.stamp
            self.flag = False

           




    def publish_map(self):
        self.get_logger().info(f'Publishing map{len(self.map_msg.data)}')
        
        self.publisher_.publish(self.map_msg)
        self.get_logger().info('Publishing map')

        
def main():
    rclpy.init()
    node = MapBuild()
    rclpy.spin(node)


    rclpy.shutdown()