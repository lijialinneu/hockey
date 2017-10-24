# !/usr/bin/env python
# -*- coding: utf-8 -*-

import requests

class PostToOneNet:

    url = "http://api.heclouds.com/bindata"

    headers = {
        "Content-Type": "image/jpg", 
        "api-key": "Nq426hjWbJ=PfEHKrv=xQOLKqEc=", 
    }

    querystring = {"device_id": "19900576", "datastream_id": "qugunqiu-liumoban"}

    def post_function(self, path):
        with open(path, 'rb') as f:
            requests.post(self.url, params=self.querystring, \
                      headers=self.headers, data=f)    
        

    
