#!/usr/bin/env python3
import requests
import time


# MIR api
mir_ip = "http://192.168.12.20"
api_version = "v2.0.0"
header = {"Authorization":"Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="}
url = mir_ip + "/api/" + api_version + "/"


def set_msg(sound_id):
    end_point = "mission_queue"
    data = {
                "mission_id": "b2ba7f0a-24e5-11eb-a709-0001299df266",
                "parameters": [
                    {
                        "id": "sound_id",
                        "value": sound_id
                    }
                ]
            }
    response = requests.post(url+end_point, headers=header, json=data)
    return response.json()


def please():
    set_msg("c7f136b0-177c-11ed-b2f5-0001299df266")
    time.sleep(2.0)

def thanks():
    set_msg("cc9ab556-177c-11ed-b2f5-0001299df266")
    time.sleep(1.5)

def avoid():
    set_msg("cf8c36f7-177c-11ed-b2f5-0001299df266")
    time.sleep(1.5)

def failed():
    set_msg("d336eb26-177c-11ed-b2f5-0001299df266")
    time.sleep(1.0)
    set_msg("d512c10e-177c-11ed-b2f5-0001299df266")
    time.sleep(2.5)

def home():
    set_msg("d6e92dec-177c-11ed-b2f5-0001299df266")
    time.sleep(1.5)





if __name__ == '__main__':
    please()
    thanks()
    avoid()
    failed()
    home()



"""
{
    "guid": "cf8c36f7-177c-11ed-b2f5-0001299df266",
    "length": "0:00:01",
    "name": "waiter_3",
    "url": "/v2.0.0/sounds/cf8c36f7-177c-11ed-b2f5-0001299df266",
    "volume": 40
  },
  {
    "guid": "d336eb26-177c-11ed-b2f5-0001299df266",
    "length": "0:00:01",
    "name": "waiter_4",
    "url": "/v2.0.0/sounds/d336eb26-177c-11ed-b2f5-0001299df266",
    "volume": 40
  },
  {
    "guid": "d512c10e-177c-11ed-b2f5-0001299df266",
    "length": "0:00:02",
    "name": "waiter_5",
    "url": "/v2.0.0/sounds/d512c10e-177c-11ed-b2f5-0001299df266",
    "volume": 40
  },
  {
    "guid": "d6e92dec-177c-11ed-b2f5-0001299df266",
    "length": "0:00:02",
    "name": "waiter_6",
    "url": "/v2.0.0/sounds/d6e92dec-177c-11ed-b2f5-0001299df266",
    "volume": 40
  }
"""