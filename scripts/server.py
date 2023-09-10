from http.server import BaseHTTPRequestHandler
import urllib.parse
from datetime import datetime
import json

from http.server import HTTPServer, BaseHTTPRequestHandler

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from geometry_msgs.msg import PoseStamped


from targetUAV import targetUAV
import multiprocessing
import rospy
import time

"""
200: İstek başarılı
204: Gönderilen paketin Formatı Yanlış
400: İstek hatalı veya geçersiz. Böyle bir durumda hata kodu sayfa içeriği olarak gönderilir.
Hata kodlarının açıklamaları Hata kodları başlığında bulunmaktadır.
401: Kimliksiz erişim denemesi. Oturum açmanız gerekmektedir.
403: Yetkisiz erişim denemesi. Yönetici yetkilerine sahip olmayan bir hesap ile
yöneticilere özel bağlantılara giriş yapılmaya çalışmaktadır.
404: Geçersiz URL.
500: Sunucu içi hata.

"""
SIMULATION = True
target_uav_list = []

if SIMULATION:
    target_uav_list = [targetUAV(0), targetUAV(1), targetUAV(2)]

class MyHTTPRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/api/telemetri_gonder':
            # Veritabanından veri çekin (örneğin, bir sözlük)
            konum_bilgileri_list = []

            # mavrostan konum çekicez.

            for uav in target_uav_list:
                dict = {
                    "takim_numarasi": uav.id,
                    "iha_enlem": uav.lat,
                    "iha_boylam": uav.lon,
                    "iha_irtifa": uav.alt,
                    "iha_dikilme": uav.pitch,
                    "iha_yonelme": uav.yaw,
                    "iha_yatis": uav.roll,
                    "zaman_farki": 0  # düzenlenecek bura
                }

                konum_bilgileri_list.append(dict)
            
            # JSON formatında yanıt hazırlayın
            with open("telemetri_cevabi.json", "w") as f:
                json.dump(konum_bilgileri_list, f, indent=2)

            # RAKİP UÇAĞIN VERİSİNİ GÖNDER
            # Dosyaya veri yazmanın başarılı olduğuna dair bir yanıt gönder
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()

            # dicti direk yollamak yerine bu şekilde yollamak lazım
            data = json.dumps(konum_bilgileri_list)
            self.wfile.write(bytes(data, "utf-8"))
            print("çağrı yapıldı")

    def do_POST(self):

        if self.path == '/api/telemetri_gonder':
            # Veritabanından veri çekin (örneğin, bir sözlük)
            konum_bilgileri_list = []

            # mavrostan konum çekicez.

            for uav in target_uav_list:
                dict = {
                    "takim_numarasi": uav.id,
                    "iha_enlem": uav.lat,
                    "iha_boylam": uav.lon,
                    "iha_irtifa": uav.alt,
                    "iha_dikilme": uav.pitch,
                    "iha_yonelme": uav.yaw,
                    "iha_yatis": uav.roll,
                    "zaman_farki": 0  # düzenlenecek bura
                }

                konum_bilgileri_list.append(dict)
            
            # JSON formatında yanıt hazırlayın
            with open("telemetri_cevabi.json", "w") as f:
                json.dump(konum_bilgileri_list, f, indent=2)

            # RAKİP UÇAĞIN VERİSİNİ GÖNDER
            # Dosyaya veri yazmanın başarılı olduğuna dair bir yanıt gönder
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()

            # dicti direk yollamak yerine bu şekilde yollamak lazım
            data = json.dumps(konum_bilgileri_list)
            self.wfile.write(bytes(data, "utf-8"))
            print("çağrı yapıldı")


def run():
    rospy.init_node('konum_cekme_node', anonymous=True)
    rate = rospy.Rate(1)

    server = HTTPServer(('localhost', 8080), MyHTTPRequestHandler)
    print('HTTP server is running...')
    server.serve_forever()
    


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
    run()

