#!/usr/bin/env python3

import requests
import json

import rospy
from beginner_tutorials.srv import Telemetry, TelemetryResponse

def handle_string_request(request):

    rospy.loginfo("Received string request: {}".format(request.request_data))    
    #TODO: Process the request and prepare the string data response


    # Sunucu adresi
    sunucu_adresi = 'http://localhost:8080'

    # Sunucudan veri çek
    response = requests.get(f'{sunucu_adresi}/api/telemetri_gonder')

    if response.status_code == 200:
        veri = json.loads(response.text)
        print(f"Sunucudan gelen veri: {veri}")
        print(type(veri))
    else:
        print("Veri çekme hatası.")
    veri_string = json.dumps(veri)
    response = veri_string
    return TelemetryResponse(response)

if __name__ == "__main__":
    rospy.init_node("string_server")
    
    # Create the rosservice server
    rospy.Service("/string_service", Telemetry, handle_string_request)
    
    rospy.spin()
    