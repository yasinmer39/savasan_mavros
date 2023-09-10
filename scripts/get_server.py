#!/usr/bin/env python3

import requests
import json



# Sunucu adresi
sunucu_adresi = 'http://localhost:8080'

# Sunucudan veri çek
response = requests.get(f'{sunucu_adresi}/api/telemetri_gonder')

if response.status_code == 200:
    veri = json.loads(response.text)
    print(f"Sunucudan gelen veri: {veri}")
else:
    print("Veri çekme hatası.")