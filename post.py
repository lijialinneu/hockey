# !/usr/bin/env python
# -*- coding: utf-8 -*-

import requests
import time
import cv, cv2





url = "http://api.heclouds.com/bindata"

headers = {
    "Content-Type": "image/jpg", 
    "api-key": "Nq426hjWbJ=PfEHKrv=xQOLKqEc=", 
}

querystring = {"device_id": "19900576", "datastream_id": "qugunqiu-liumoban"}


while True:
    try:
        time.sleep(1)
        image = cv2.imread('1.jpg')
    
        image = cv2.resize(image, (640,480))
        cv2.imwrite('post.jpg', image)
        with open('post.jpg', 'rb') as f:
            r = requests.post(url, params=querystring, headers=headers, data=f)
            print(r.status_code)
    except:
        pass
        



