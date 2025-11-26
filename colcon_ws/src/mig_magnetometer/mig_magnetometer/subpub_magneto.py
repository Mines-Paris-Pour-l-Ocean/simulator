# importation des bibliothèques

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Header
from pygeomag import GeoMag
import numpy as np
from geometry_msgs.msg import Quaternion
from . import tools2






class ControleNode(Node):



    def __init__(self):
        super().__init__('controle_node')
        
        # position initiale du robot
        self.Lat_0 = 41.7777
        self.Lon_0 = 3.0333

        # changement du système de coordonnées pour t0 : longitude, latitude, altitude => position (x,y,z)
        self.X_0 ,self.Y_0 = tools2.wgs84_to_xyz(self.Lat_0, self.Lon_0)

        # création variable de stockage
        self.subscription = self.create_subscription(Odometry, '/bluerov/navigator/odometry', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(MagneticField, '/bluerov/sensors/Compass', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.magfield = GeoMag()
        self.q = Quaternion()




    def listener_callback(self, msg):

        # récupération des données position
        x_pos = msg.pose.pose.position.x
        y_pos = msg.pose.pose.position.y
        z_pos = msg.pose.pose.position.z

        # récupération des données orientation
        self.q = msg.pose.pose.orientation

        # changement du système de coordonnées : position (x,y,z) => longitude, latitude, altitude (profondeur)
        lon, lat, z = tools2.xyz_to_wgs84(self.X_0 + x_pos, self.Y_0 + y_pos, z_pos, self.Lat_0, self.Lon_0)

        # calcul du champ magnétique
        self.magfield = tools2.mag_field(lon, lat, z)

        # affiche dans le terminal
        #self.get_logger().info(f'Odometry: Longitude={lon}, Latitude={lat}, Profondeur={z}, Q={self.q}')


    def timer_callback(self):

        self.get_logger().info('--Debut du callback--', once = False)

        # création du message qui renvoie le champ magnétique
        new_msg = MagneticField()

        # les valeurs sont usuellement en Tesla
        qx, qy, qz, qw = self.q.x , self.q.y, self.q.z, self.q.w
        new_msg.magnetic_field.x, new_msg.magnetic_field.y, new_msg.magnetic_field.z = tools2.magfield_NED(qx, qy, qz, qw, self.magfield)
        new_msg.magnetic_field_covariance = np.zeros(9, dtype=np.float64)

        self.publisher_.publish(new_msg)
        
        self.get_logger().info(f'MagneticField : B_x = {new_msg.magnetic_field.x}, B_y = {new_msg.magnetic_field.y}, B_z = {new_msg.magnetic_field.z}')
        self.i += 1




def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ControleNode())
    ControleNode().destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()

