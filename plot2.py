#!/usr/bin/env python

from __future__ import division
import os
import json
import matplotlib

# For the error: Exception ignored in: <bound method Image.del of <tkinter.PhotoImage object at 0x7f1b5f86a710>> Traceback (most recent call last):
matplotlib.use('Agg')
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import random
plt.rcParams.update({'figure.max_open_warning': 0})
import numpy as np

path_to_file = '/model/cars_frenet.json'
path_to_save_dir = '/model/plots/'
evaluation = {
    "frenet_data": {},
    "other": {}
}
car_ids = []
with open(path_to_file) as json_file:
    data = json.load(json_file)
    for obj in data:
        subData = obj
        for subObj in subData:
            car_id = subObj["car_id"];
            if car_id not in car_ids:
                car_ids.append(car_id)
            if car_id in evaluation["frenet_data"].keys():
                evaluation["frenet_data"][car_id].append(subObj["frenet_data"])
            else:
                evaluation["frenet_data"][car_id] = []
                evaluation["frenet_data"][car_id].append(subObj["frenet_data"]) 
            
            if car_id in evaluation["other"].keys():
                evaluation["other"][car_id].append(subObj["other"])
            else:
                evaluation["other"][car_id] = []
                evaluation["other"][car_id].append(subObj["other"]) 
            

            


#-------------------------car in frenet frame-----------------------
print(car_ids)

name = path_to_save_dir+"frenet_frame"
plt.figure()
car_ids = [98]
for car in car_ids:
    data = {"s":[], "d":[]}
    for fData in evaluation["frenet_data"][car]:
        data["s"].append(fData["s"])
        data["d"].append(fData["d"])
    x = []
    for index in range(len(data["s"])):
        x.append(0)
    plt.xlim([8, -8])
    y = data["s"]
    r = lambda: random.randint(0,255)
    hex_number = '#%02X%02X%02X' % (r(),r(),r())
    plt.plot(x, y,'--',label=car, color=hex_number)

    x = []
    for d in data["d"]:
        x.append(d)
    y = data["s"]
    plt.plot(x,y,'.',label=car, color=hex_number)

plt.ylabel("s = longitudinal displacement")
plt.xlabel("d = lateral displacement")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()

#--------------------------------Car in d & t-----------------------

name = path_to_save_dir+"frenet_frame_d_t"
plt.figure()
for car in car_ids:
    data = {"t":[], "d":[]}
    for fData in evaluation["frenet_data"][car]:
        data["t"].append(fData["sec"])
        data["d"].append(fData["d"])
    x = []
    for t in data["t"]:
        x.append(t)
    y = data["d"]
    plt.plot(x,y,label=car)
plt.ylabel("d = lateral displacement")
plt.xlabel("sec")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()

#--------------------------------Car in s & t-----------------------

name = path_to_save_dir+"frenet_frame_s_t"
plt.figure()
for car in car_ids:
    data = {"t":[], "s":[]}
    for fData in evaluation["frenet_data"][car]:
        data["t"].append(fData["sec"])
        data["s"].append(fData["s"])
    x = []
    for t in data["t"]:
        x.append(t)
    y = data["s"]
    plt.plot(x,y,label=car, color=hex_number)

plt.ylabel("s = longitudinal displacement")
plt.xlabel("sec")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()

#--------------------------------Car in speed & t-----------------------

name = path_to_save_dir+"frenet_frame_speed_t"
plt.figure()
for car in car_ids:
    data = {"t":[], "s":[]}
    for fData in evaluation["frenet_data"][car]:
        data["t"].append(fData["sec"])
    for fData in evaluation["other"][car]:
        data["s"].append(fData["long_speed"])
    x = []
    for t in data["t"]:
        x.append(t)
    y = data["s"]
    plt.plot(x,y,label=car, color=hex_number)

plt.ylabel("speed")
plt.xlabel("sec")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()

#-----------------------------------EGO---------------------------------------
path_to_file = '/model/ego_frenet.json'
path_to_save_dir = '/model/plots/'
evaluation_ego = {
    "frenet_data": [],
    "odom_data": [],
    "other": []
}
with open(path_to_file) as json_file:
    data = json.load(json_file)
    for obj in data:
        evaluation_ego["frenet_data"].append(obj["frenet_data"])
        evaluation_ego["odom_data"].append(obj["odom_pos"]) 
        evaluation_ego["other"].append(obj["other"]) 

#--------------------------------Ego vehicle in speed and t-----------------------

name = path_to_save_dir+"ego_frenet_frame_speed_t"
plt.figure()
data = {"s":[], "t":[]}
for fData in evaluation_ego["frenet_data"]:
    data["t"].append(fData["sec"])

for fData in evaluation_ego["other"]:
    data["s"].append(fData["long_speed"])
x = []
for t in data["t"]:
    x.append(t)
y = data["s"]
plt.plot(x,y,label="ego", color=hex_number)

plt.ylabel("s = longitudinal displacement")
plt.xlabel("sec")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()





#--------------------------------Ego vehicle in s and t-----------------------

name = path_to_save_dir+"ego_frenet_frame_s_t"
plt.figure()
data = {"s":[], "t":[]}
for fData in evaluation_ego["frenet_data"]:
    data["s"].append(fData["s"])
    data["t"].append(fData["sec"])
x = []
for t in data["t"]:
    x.append(t)
y = data["s"]
plt.plot(x,y,label="ego", color=hex_number)

plt.ylabel("s = longitudinal displacement")
plt.xlabel("sec")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()





#--------------------------------Ego vehicle in frenet frame-----------------------

path_to_file = '/model/ego_frenet.json'
path_to_save_dir = '/model/plots/'
evaluation_ego = {
    "frenet_data": [],
    "odom_data": []
}
with open(path_to_file) as json_file:
    data = json.load(json_file)
    for obj in data:
        evaluation_ego["frenet_data"].append(obj["frenet_data"])
        evaluation_ego["odom_data"].append(obj["odom_pos"]) 

name = path_to_save_dir+"ego_frenet_frame"
plt.figure()
data = {"s":[], "d":[]}
for fData in evaluation_ego["frenet_data"]:
    data["s"].append(fData["s"])
    data["d"].append(fData["d"])
x = []
for index in range(len(data["s"])):
    x.append(0)
plt.xlim([8, -8])
y = data["s"]
r = lambda: random.randint(0,255)
hex_number = '#%02X%02X%02X' % (r(),r(),r())
plt.plot(x, y,'--', color=hex_number)

x = []
for d in data["d"]:
    x.append(d)
y = data["s"]
plt.plot(x,y,'.',label="ego", color=hex_number)

plt.ylabel("s = longitudinal displacement")
plt.xlabel("d = lateral displacement")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()


#-------Ego vehicle with other vehicle frenet frame with road center as reference------------

path_to_file = '/model/ego_frenet.json'
path_to_save_dir = '/model/plots/'
evaluation_ego = {
    "frenet_data": [],
    "odom_data": []
}
with open(path_to_file) as json_file:
    data = json.load(json_file)
    for obj in data:
        evaluation_ego["frenet_data"].append(obj["frenet_data"])
        evaluation_ego["odom_data"].append(obj["odom_pos"]) 
name = path_to_save_dir+"frenet_frame_all"
plt.figure()
data = {"s":[], "d":[]}
for fData in evaluation_ego["frenet_data"]:
    data["s"].append(fData["s"])
    #road center as the reference line
    data["d"].append(fData["d"])
    #ego path as the reference line
    #data["d"].append(0)

x = data["d"]
y = data["s"]
r = lambda: random.randint(0,255)
hex_number = '#%02X%02X%02X' % (r(),r(),r())
plt.plot(x,y,'.',label="ego", color=hex_number)
plt.xlim([8, -8])
print(car_ids)
#car_ids = [114]
for car in car_ids:
    car_data = {"s":[], "d":[]}
    for fData in evaluation["frenet_data"][car]:
        car_data["d"].append(fData["d"])
        car_data["s"].append(fData["s"])
    x = car_data["d"]
    y = car_data["s"]
    r = lambda: random.randint(0,255)
    hex_number = '#%02X%02X%02X' % (r(),r(),r())
    plt.plot(x,y,'.',label=car, color=hex_number)

plt.ylabel("s = longitudinal displacement")
plt.xlabel("d = lateral displacement")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()
'''
#-------Ego vehicle with other vehicle frenet frame with ego center as reference------------

path_to_file = '/model/ego_frenet.json'
path_to_save_dir = '/model/plots/'
evaluation_ego = {
    "frenet_data": [],
    "odom_data": []
}
with open(path_to_file) as json_file:
    data = json.load(json_file)
    for obj in data:
        evaluation_ego["frenet_data"].append(obj["frenet_data"])
        evaluation_ego["odom_data"].append(obj["odom_pos"]) 
name = path_to_save_dir+"frenet_frame_all"
plt.figure()
data = {"s":[], "d":[]}
for fData in evaluation_ego["frenet_data"]:
    #if fData["s"]+30 > 270:
    #    break 
    data["s"].append(fData["s"]+30)
    #road center as the reference line
    #data["d"].append(fData["d"])
    #ego path as the reference line
    data["d"].append(0)

x = data["d"]
y = data["s"]
r = lambda: random.randint(0,255)
hex_number = '#%02X%02X%02X' % (r(),r(),r())
plt.plot(x,y,'.',label="ego", color=hex_number)
plt.xlim([8, -8])
print(car_ids)
#car_ids = [114]
for car in car_ids:
    car_data = {"s":[], "d":[]}
    for fData in evaluation["frenet_data"][car]:
        #if fData["s"]+30 > 270:
        #    break
        car_data["d"].append(fData["d_ego_ref"])
        car_data["s"].append(fData["s"]+30)
    x = car_data["d"]
    y = car_data["s"]
    r = lambda: random.randint(0,255)
    hex_number = '#%02X%02X%02X' % (r(),r(),r())
    plt.plot(x,y,'.',label=car, color=hex_number)

plt.ylabel("s = longitudinal displacement")
plt.xlabel("d = lateral displacement")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()

'''



#--------------------------------plot ego and car in d and t -----------------------

path_to_file = '/model/ego_frenet.json'
path_to_save_dir = '/model/plots/'
evaluation_ego = {
    "frenet_data": [],
    "odom_data": []
}
with open(path_to_file) as json_file:
    data = json.load(json_file)
    for obj in data:
        evaluation_ego["frenet_data"].append(obj["frenet_data"])
        evaluation_ego["odom_data"].append(obj["odom_pos"]) 
name = path_to_save_dir+"frenet_frame_all_d_t"
plt.figure()
data = {"t":[], "d":[]}
for fData in evaluation_ego["frenet_data"]:
    data["t"].append(fData["sec"])
    data["d"].append(fData["d"])

x = data["t"]
y = data["d"]
r = lambda: random.randint(0,255)
hex_number = '#%02X%02X%02X' % (r(),r(),r())
plt.plot(x,y,label="ego", color=hex_number)
print(car_ids)
#car_ids = [114]
for car in car_ids:
    car_data = {"t":[], "d":[]}
    for fData in evaluation["frenet_data"][car]:
        car_data["d"].append(fData["d"])
        car_data["t"].append(fData["sec"])
    x = car_data["t"]
    y = car_data["d"]
    r = lambda: random.randint(0,255)
    hex_number = '#%02X%02X%02X' % (r(),r(),r())
    plt.plot(x,y,label=car, color=hex_number)

plt.ylabel("d = lateral displacement")
plt.xlabel("sec")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()



'''
#--------------------------------Ego centerline in odom frame-----------------------

path_to_file = '/model/centerline.json'
path_to_save_dir = '/model/plots/'
evaluation = {
    "ego": []
}
with open(path_to_file) as json_file:
    data = json.load(json_file)
    for obj in data["ego_center"]:
        evaluation["ego"].append(obj)

name = path_to_save_dir+"centerline"
plt.figure()
data = {"x":[], "y":[]}
for fData in evaluation["ego"]:
    data["x"].append(fData["x"])
    data["y"].append(fData["y"])
x = data["x"]
y = data["y"]
r = lambda: random.randint(0,255)
hex_number = '#%02X%02X%02X' % (r(),r(),r())
plt.plot(x, y,'--', color=hex_number)
plt.ylabel("y")
plt.xlabel("x")
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()
'''

