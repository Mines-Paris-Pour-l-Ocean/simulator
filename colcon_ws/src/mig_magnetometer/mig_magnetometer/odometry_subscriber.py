# importation des bibliothèques

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Header

from . import tools2






class MinimalSubscriber(Node):



    def __init__(self):
        super().__init__('minimal_subscriber')
        
        # position initiale du robot
        self.Lat_0 = 41.7777
        self.Lon_0 = 3.0333

        # changement du système de coordonnées pour t0 : longitude, latitude, altitude => position (x,y,z)
        self.X_0 ,self.Y_0 = tools2.wgs84_to_xyz(self.Lat_0, self.Lon_0)

        # création variable de stockage
        self.subscription = self.create_subscription(Odometry, '/bluerov/navigator/odometry', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning




    def listener_callback(self, msg):

        # récupération des données position
        x_pos = msg.pose.pose.position.x
        y_pos = msg.pose.pose.position.y
        z_pos = msg.pose.pose.position.z

        # récupération des données orientation
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # changement du système de coordonnées : position (x,y,z) => longitude, latitude, altitude (profondeur)
        lon, lat, z = tools2.xyz_to_wgs84(self.X_0 + x_pos, self.Y_0 + y_pos, z_pos, self.Lat_0, self.Lon_0)

        # calcul du champ magnétique
        magfield = tools2.mag_field(lon, lat, z)

        # création du message qui renvoie le champ magnétique
        new_msg = MagneticField()

        # header du new message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "repère du robot NED"
        new_msg.header = header

        # valeurs du champ magnétique dans le repère NED du robot
        new_msg.magnetic_field.x, new_msg.magnetic_field.y, new_msg.magnetic_field.z = tools2.magfield_NED(qx, qy, qz, qw, magfield)

        # mise en ligne du new message

        # self.pub.publish(new_msg)
        self.get_logger().info(f'Odom: Longitude={lon}, Latitude={lat}, Profondeur={z}, Qx={qx}, QY={qy}, QZ={qz}, QW={qw}, champ_xrobot={new_msg.magnetic_field.x}, champ_yrobot={new_msg.magnetic_field.y}, champ_zrobot={new_msg.magnetic_field.z}') 





def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()





if __name__ == '__main__':
    main()

