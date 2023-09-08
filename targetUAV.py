"""
hedef uçağın verilerini alıp, servera yollar, sim amaçlı
"""


import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from geometry_msgs.msg import PoseStamped

class targetUAV():
    def __init__(self, id) -> None:

        self.id = id

        self.lat = float()
        self.lon = float()
        self.alt = float()

        self.roll = float()
        self.pitch = float()
        self.yaw = float()

        rospy.Subscriber(f'/uav{self.id}/mavros/global_position/global',
                         NavSatFix, self.gps_callback)

        rospy.Subscriber(f'/uav{self.id}/mavros/local_position/pose',
                 PoseStamped, self.imu_callback)


    def gps_callback(self, gps_msg):

        self.lat = gps_msg.latitude  # deg
        self.lon = gps_msg.longitude  # deg
        self.alt = gps_msg.altitude  # m WGS 84 ellipsoid

    def imu_callback(self, msg):

        quatx = msg.pose.orientation.x
        quaty = msg.pose.orientation.y
        quatz = msg.pose.orientation.z
        quatw = msg.pose.orientation.w
        self.roll, self.pitch, self.yaw = euler_from_quaternion([quatx, quaty, quatz, quatw])

        self.roll = self.roll * 57.295
        self.pitch = self.pitch * 57.295
        self.yaw =self.yaw * 57.295

    # headingi 0 360 çeken eklenecek buraya


def main():

    rospy.init_node('konum_cekme_node', anonymous=True)
    rate = rospy.Rate(1)

    uav1 = targetUAV(1)

    while not rospy.is_shutdown():
        print("lat:", uav1.lat)
        print("lon:", uav1.lon)
        print("alt", uav1.alt)
        print(" ")
        print("roll:", uav1.roll)
        print("pitch:", uav1.pitch)
        print("yaw", uav1.yaw)
        print(" ")

        rate.sleep()



if __name__ == '__main__':
    main()
