from __future__ import division
import os
import json
import math
import matplotlib
from scipy.sparse import data

# For the error: Exception ignored in: <bound method Image.del of <tkinter.PhotoImage object at 0x7f1b5f86a710>> Traceback (most recent call last):
matplotlib.use('Agg')
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import random
plt.rcParams.update({'figure.max_open_warning': 0})
from datetime import datetime 
import numpy as np
from collections import OrderedDict


# Next step: Identify all below for IBEO and Waymo dataset.
# 1. Compute the cut-in and overtaking without lane information - Done(This logic will not work if there is no vehicle behind. But in that case, it may not be critical to change the lane)
#   A. Find example to see the logic is working for overtaking scenario - Done(We have a few examples)
# 2. Extract the lane information 
#   A. Find the region of interest, then only consider the points in that region to construct the lane - Done(There is a concern of region of interest during the lane change)
#       a. Yaw can determine the lane equation of two points of the vehicle odom. Consider the perpendicular line from each point of the constructed line, 
#           then consider only all other points that are within a threshold meter. 
#   B. Lanes can be detected from the exported intensity image from the top_down_package - Done   
#   C. Publish the lane points - Done
#   D. Save pointcloud per second to txt file - Done
#   E. Get the published lane points for 8 second and construct the intensity image at the process.py - Done
#   F. top_down_projcetion package is getting some issue with large data
#   G. ODOM frame to Baselink for lane detection
#   H. Baselink to ODOM frame for getting the right position in the world
#   I. Do the lane detection at the process.py
# 3. Convert this data to lanelet format  
# 4. Identify content for lanelet format for IBEO data
#   A. Road data: Number of lane
#   B. Road structure
#   C. Anyother static information possible such as building, trees, rad signs etc
# 4. Compute the lanelet format from the extracted content.

class Data:
    def __init__(self, **entries):
        self.__dict__.update(entries)

class Process:
    
    def __init__(self):
        self.path_to_dir = None
        if self.path_to_dir is not None:
            dirFiles = os.listdir(self.path_to_dir)
            file_list = []
            for _file in dirFiles: #filter out all non jpgs
                if _file.endswith('.txt'):
                    file_list.append(_file)
            file_list.sort(key=lambda f: int(filter(str.isdigit, f)))
        file_count = 0
        
        #new_ibeo_data/gate_south-end_north-end_20210531_051742_0_bag.txt 
        # 1. object_id:43 = lane following and cutout scenario
        # 2. object_id:2 = cut in scenario at the end 
        
        #new_ibeo_data/gate_south-end_north-end_20210601_080738_0_bag.txt 
        # 1. object_id:183 = cut-in scenario
        
        
        #trainting_0000_bags/segment-10212406498497081993_5300_000_5320_000_with_camera_labels.bag
        # 1. Cut-in scenario from right
        
        #training_0000_bags/segment-10231929575853664160_1160_000_1180_000_with_camera_labels.bag
        # 1. Cut-in scenario from right
        
        #training_0001_bags/segment-10498013744573185290_1240_000_1260_000_with_camera_labels.bag
        # 1. Ego's lane change to the left with back vehicle
        
        #training_0001_bags/segment-10584247114982259878_490_000_510_000_with_camera_labels.bag
        # 1. Ego's lane change to the left without back vehicle
        
        #training_0002_bags/segment-11119453952284076633_1369_940_1389_940_with_camera_labels.bag
        # 1. Ego's lane change to the left with back vehicle
        
        #training_0003_bags/segment-1146261869236413282_1680_000_1700_000_with_camera_labels.bag
        # 1. Cut-in from right
        
        #training_0003_bags/segment-11486225968269855324_92_000_112_000_with_camera_labels.bag
        # 1. Ego's lane change to the left without back vehicle
        
        
        self.path_to_dir = '/constraint_model/data/new_ibeo_data/'
        #file_list = ['gate_south-end_north-end_20210601_080738_0_bag.txt']
        #file_list = ['gate_south-end_north-end_20210531_051742_0_bag.txt']
        file_list = ['gate_south-end_north-end_20210528_064001_0_bag.txt']
        
        
        
        for _file in file_list:
            print("Current file: {}".format(_file))
            self.current_file = None
            self.evaluation = {
                'ego_odom':None,
                'other_objects': None
            }
            
            with open(self.path_to_dir+_file) as json_file:
                data = json.load(json_file)
                self.evaluation['ego_odom'] = data['ego_odom']
                self.current_file = os.path.splitext(_file)[0]
            
            self.reset()    
            self.get_ego_data()
            
            self.cut_in_scenario = False
            self.overtaking_scenario = True
            
            if self.cut_in_scenario:
            
                with open(self.path_to_dir+_file) as json_file:
                    data = json.load(json_file)
                    self.evaluation['other_objects'] = data['other_objects']['cut-in']
                
                self.evaluate()
                cars = self.group_by_car.keys()
                cars.append('ego')
                self.plot_multiple_car(cars = cars, png_name = 'combined_trajectory_front')
                
                # Extraction logic of cut-in scenario
                self.extraction_logic()
                
            self.reset()
            self.get_ego_data()
                
            if self.overtaking_scenario:
                
                with open(self.path_to_dir+_file) as json_file:
                    data = json.load(json_file)
                    self.evaluation['other_objects'] = data['other_objects']['overtaking']
                
                self.evaluate()
                print(self.car_processed_data.keys())
                
                cars = self.group_by_car.keys()
                cars.append('ego')
                self.plot_multiple_car(cars = cars, png_name = 'combined_trajectory_behind')
                
                # Extraction logic of cut-in scenario
                self.extraction_logic()
                
    def get_ego_data(self):
        self.car_processed_data['ego'] = {
            'x': [],
            'y': [],
            'theta': []
        }
        
        # Ego odometry
        previous = None
        for item in self.evaluation['ego_odom']:
            data  = Data(**item)
            if data.sec in self.ego_data:
                    self.ego_data[data.sec].append(data)
            else:
                self.ego_data[data.sec] = []
                self.ego_data[data.sec].append(data)
       

        # Storing the plotting data        
        for sec in self.ego_data:
            item = self.ego_data[sec]
            if len(item) > 1: #>1
                data = item[0]
                self.car_processed_data['ego']['x'].append(data.position_x)
                self.car_processed_data['ego']['y'].append(data.position_y)
            else:
                continue

    
    def reset(self):
        
        # Edo data per second
        self.ego_data = OrderedDict()

        # This contains the all the objects data under each second
        self.other_objects_data = OrderedDict()

        # This simply contains object_id per second
        self.other_objects = OrderedDict()

        # It group by object_id and then store all data of the object id in each second
        self.other_objects_sec = OrderedDict()
        self.group_ego_by_sec = OrderedDict()

        # This contains all the data under object_id per sec
        self.group_by_car = OrderedDict()
        
        # This contains all the data under object_id per nsec
        self.group_by_car_nsec = OrderedDict()

        # It is mainly for plotting
        self.car_processed_data = OrderedDict()

        self.deleted_objects = []
        
    
    def evaluate(self):
        
        # Other objects odometry 
        for item in self.evaluation['other_objects']:
            data  = Data(**item)
            if data.sec in self.other_objects_data:
                self.other_objects_data[data.sec].append(data)
                self.other_objects[data.sec].append(data.object_id)
            else:
                self.other_objects_data[data.sec] = []
                self.other_objects_data[data.sec].append(data)
                self.other_objects[data.sec] = []
                self.other_objects[data.sec].append(data.object_id)

            if data.object_id in self.other_objects_sec:
                if data.sec in self.other_objects_sec[data.object_id]:
                    self.other_objects_sec[data.object_id][data.sec].append(data)
                else:
                    self.other_objects_sec[data.object_id][data.sec] = []
                    self.other_objects_sec[data.object_id][data.sec].append(data)
            else:
                self.other_objects_sec[data.object_id] = {}
                self.other_objects_sec[data.object_id][data.sec] = []
                self.other_objects_sec[data.object_id][data.sec].append(data)
        
        #print(self.other_objects.keys())
        #print(self.ego_data.keys())
        for o_id in self.other_objects_sec.keys():
            if len(self.other_objects_sec[o_id].keys()) < 5:
                self.deleted_objects.append(o_id)
        
        # Grouping the data by car
        previous = {}
        for item in self.evaluation['other_objects']:
            data  = Data(**item)
            if data.object_id in self.group_by_car:
                # Logic to capture the data at each second
                diff = 1  
                if previous[data.object_id] is not None:
                    diff = data.sec-previous[data.object_id]  
                
                if diff >= 1:
                    self.group_by_car[data.object_id].append(data)
                    
            else:
                if data.object_id not in self.deleted_objects:
                    self.group_by_car[data.object_id] = []
                    self.group_by_car[data.object_id].append(data)
            
            # Group by car and collect all data.
            if data.object_id in self.group_by_car_nsec:
                self.group_by_car_nsec[data.object_id].append(data)
            else:
                self.group_by_car_nsec[data.object_id] = []
                self.group_by_car_nsec[data.object_id].append(data)
                
            previous[data.object_id] = data.sec
        
        for car in self.group_by_car.keys():
            self.car_processed_data[car] = {
                'x': [],
                'y': [],
                'theta': []
            }
            item = self.group_by_car[car]
            
            # It is important to remember that yaw rate is the rate of change of yaw. That it is a rate of change of z-axis on the Euler. 
            for data in item:     
                self.car_processed_data[car]['x'].append(data.position_x)
                self.car_processed_data[car]['y'].append(data.position_y)
                

    
    def extraction_logic(self):
        
        # trainting_0000_bags/results/segment-10153695247769592104_787_000_807_000_with_camera_labels.bag is an example of cut out scenario and lane following. 
        # Because the euclidean distance logic extract all cut0in, cut-out and lane -following scenario. Cut-out becuase it was lane following for a whicle beore the cut out.
        # I could solve the below issue with lateral acceleration, but still has some false positives. So we need the lane information. 
        
        #*************************CUT-IN scenario*************************
        if self.cut_in_scenario:
            # LOGIC: 
            # We choose 8 secod as the projected ego data arequired. For normal lane change, a paper estimated that duration is 8 second. 
            # 1. Loop through ego_data per second
            # 2. Project the ego path for 8 second.
            # 3. We now the the projected path of ego for 8 second.
            # 4. Loop through each second untill the 8 second
            # 5. At each second find the euclidean distance of the vehicle nearby with ego path.
            # 6. If the euclidean distance is closer to zero of any vehicle at any second and checking accleration is greater than the threshold(0.50) and occured more than once. Then that has the lane change
            count = 0 
            lane_changed_car = {}
            for sec in self.ego_data:
                #print("Projecting {} second".format(sec))
                if True:
                    # This is for getting the projected data for ego motion from sec to sec+8 seconds
                    projected_secs = [key for key in range(sec, sec+8)]
                    all_ego_data_till_proj = []
                    for temp_sec in projected_secs:
                        if temp_sec in self.ego_data:
                            for temp_data in self.ego_data[temp_sec]:
                                all_ego_data_till_proj.append(temp_data)   
                                
                    first_sec = sec
                    for proj_sec in projected_secs:
                        
                        #Checking if we have any data for other objects in that sec.
                        if proj_sec in self.other_objects:
                            all_cars_at_sec = list(set(self.other_objects[proj_sec]))
                            for car in all_cars_at_sec:
                                if car not in self.deleted_objects and car not in lane_changed_car.keys():
                                    other_data_list = self.group_by_car[car] 
                                    
                                    # Checking the other cars at the front to do the cut-in event
                                    if other_data_list[0].rel_pos_x >= 0 and other_data_list[0].rel_pos_x <= 50:
                                        
                                        for o_data in other_data_list:
                                            acceleration_change = 0
                                            '''if car == 2:
                                                print("sec: {} id: {} x: {}, y: {},linear_y: {} ".format(o_data.sec, o_data.object_id,o_data.rel_pos_x, o_data.rel_pos_y, o_data.linear_y))                               
                                            '''
                                            # Loop through ego_s trajectory till 8 second
                                            for e_data in all_ego_data_till_proj:
                                                x1, y1 = e_data.position_x, e_data.position_y
                                                x2, y2 = o_data.position_x, o_data.position_y
                                                
                                                # Finding the euclidean distance is using this formula square root((x2-x1)^2+ (y2-y1)^2)
                                                d = math.sqrt(
                                                    (
                                                        math.pow((x2-x1), 2)
                                                        +
                                                        math.pow((y2-y1), 2)
                                                    )
                                                )
                                                
                                                # Checking accleration is greater than the threshold and occured more than once
                                                if abs(o_data.linear_y) >= 0.50:
                                                    acceleration_change += 1
                                                
                                                if d < 0.50 and acceleration_change > 1:
                                                    print("object_id: {}".format(o_data.object_id))
                                                    print("position_x: {}".format(o_data.position_x))
                                                    print("position_y: {}".format(o_data.position_y))
                                                    print("linear_y: {}".format(o_data.linear_y))
                                                    print(e_data.sec)
                                                    print("________________")
                                                    store = {
                                                        'object_id': car, 
                                                        'euclidean_dist': d,
                                                        'first_sec': first_sec,
                                                        'lc_detected': e_data.sec,
                                                        'position_x': o_data.position_x, 
                                                        'position_y': o_data.position_y,
                                                        'linear_y': o_data.linear_y
                                                        
                                                    }
                                                    if car not in lane_changed_car:
                                                        lane_changed_car[car] = store
                                    else:
                                        # Car is not at the front
                                        pass
                                                
                
            print("cut in cars: {}".format(lane_changed_car.keys()))
            for car in lane_changed_car.keys():
                print(lane_changed_car[car])
                
            cars = lane_changed_car.keys()
            cars.append('ego')
            self.plot_multiple_car(cars = cars, png_name = 'combined_trajectory_cut_in')
        
        
        #*************************OVERTAKING scenario*************************
        if self.overtaking_scenario:
            # LOGIC: 
            # We choose 8 secod as the projected  data arequired. For normal lane change, a paper estimated that duration is 8 second. 
            # 1. Loop through ego_data per second
            # 2. Project the ego path for 8 second.
            # 3. We now the the projected path of ego for 8 second.
            # 4. Loop through each second untill the 8 second
            # 5. At each second find the euclidean distance of the vehicle nearby with ego path.
            # 6. If the euclidean distance is closer to zero of any vehicle at any second and checking accleration is greater than the threshold(0.50) and occured more than once. Then that has the lane change
            
            ego_lane_change = []
            # Project car data by looping through each car            
            for car in self.group_by_car.keys():
                print("--------------car: {}------------".format(car))     
                # Get all the data of the car
                for sec in self.other_objects_sec[car].keys():      
                    if sec in self.other_objects_sec[car]:
                        
                        # This is for getting the projected data for the car from sec to sec+8 seconds
                        projected_secs = [key for key in range(sec, sec+8)]
                        car_all_data_till_proj = []
                        for temp_sec in projected_secs:
                            if temp_sec in self.other_objects_sec[car]:
                                o_data_list_at_sec = self.other_objects_sec[car][temp_sec]
                                for temp_data in o_data_list_at_sec:
                                    car_all_data_till_proj.append(temp_data) 
                                
                    first_sec = sec
                    
                    #Looping through projected secs for each car
                    for proj_sec in projected_secs:
                        if proj_sec in self.ego_data:
                            ego_data_at_sec = self.ego_data[proj_sec]
                            for e_data in ego_data_at_sec:
                                acceleration_change = 0
                                
                                # Loop through ego_s trajectory till 8 second
                                for o_data in car_all_data_till_proj:
                                    x1, y1 = o_data.position_x, o_data.position_y
                                    x2, y2 = e_data.position_x, e_data.position_y
                                    
                                    # Finding the euclidean distance is using this formula square root((x2-x1)^2+ (y2-y1)^2)
                                    d = math.sqrt(
                                        (
                                            math.pow((x2-x1), 2)
                                            +
                                            math.pow((y2-y1), 2)
                                        )
                                    )
                                    
                                    # Checking accleration is greater than the threshold and occured more than once
                                    if abs(e_data.linear_y) >= 0.50:
                                        acceleration_change += 1
                                    
                                    if d < 0.50 and acceleration_change > 1:
                                        print("object_id: {}".format(e_data.object_id))
                                        print("position_x: {}".format(e_data.position_x))
                                        print("position_y: {}".format(e_data.position_y))
                                        print("linear_y: {}".format(e_data.linear_y))
                                        print(o_data.sec)
                                        print("________________")
                                        store = {
                                            'car_behind_object_id': o_data.object_id, 
                                            'euclidean_dist': d,
                                            'first_sec': first_sec,
                                            'lc_detected': o_data.sec,
                                            'position_x': e_data.position_x, 
                                            'position_y': e_data.position_y,
                                            'linear_y': e_data.linear_y
                                            
                                        }
                                        ego_lane_change.append(store)
                                
                            
            print(ego_lane_change)
        
        # Lane following and CUT-OUT scenario
        
        
            
            
    def test_plot(self, cars =  None):
        plt.figure()
        ax = plt.axes(projection='3d')
        z = []
        count = 0
        for data in self.car_processed_data['ego']['x']:
            z.append(count)
            count += 1
        for car in cars:
            x = self.car_processed_data[car]['x']
            y = self.car_processed_data[car]['y']
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('Timesteps')
            ax.scatter(x, y, z, c=z, cmap='viridis', linewidth=0.5)
  
        name = self.path_to_save_dir+"3d_test"+car
        plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
        plt.savefig(name)
        plt.close()
    def plot_a_car(self, car = None):
        name = self.path_to_save_dir+"trajectory_projection_"+str(car)
        plt.figure()
        plt.xlabel("x(m)")
        plt.ylabel("y(m)")
        plt.plot(self.car_processed_data[car]['x'], self.car_processed_data[car]['y'])
        plt.savefig(name)
        plt.close()
    
    def plot_multiple_car(self, cars = None, png_name = None):        
        
        if self.current_file is not None:
            name = self.path_to_dir+self.current_file+"_"+png_name
            print(name)
            plt.figure()
            fig, ax = plt.subplots()
            for car in cars:
                if car == 'ego':
                    ax.plot(self.car_processed_data[car]['x'], self.car_processed_data[car]['y'], label=car, linewidth=3)
                else:
                    ax.plot(self.car_processed_data[car]['x'], self.car_processed_data[car]['y'], label=car)
            plt.xlabel("x(m)")
            plt.ylabel("y(m)")
            plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
            plt.savefig(name)
            plt.close()
                  


if __name__=='__main__':
    p = Process()
