#!/usr/bin/env python

from __future__ import division
import rospy
from nav_msgs.msg import Odometry
import json
import tf
import math
import rosbag
from std_msgs.msg import String
from collections import OrderedDict
import os
from rss import RSS

class FeatureModel:
    
    def __init__(self):
        rospy.init_node("FeatureModel node initilization")
        self.scenario_file = rospy.get_param('~scenario_json_file')
        self.cars_frenet_file = rospy.get_param('~cars_frenet_json_file')
        self.ego_frenet_file = rospy.get_param('~ego_frenet_json_file')
        self.parameter_dir = rospy.get_param('~parameter_json_dir')
        self.bag_file = None
        self.parameterCutIn = []
        self.parameterCutOut = []
        self.group_by_sec = OrderedDict()
        self.group_ego_by_sec = OrderedDict()
        self.window = 1
        self.rss = RSS()
        rospy.Subscriber('/finish_extraction', String, self.process)
        self.process("true")
        rospy.on_shutdown(self.shutdown)  
        rospy.spin()
    
    def process(self, msg):
        allScenarios = None;
        with open(self.scenario_file) as json_file1:
            allScenarios = json.load(json_file1)

        self.bag_file = allScenarios['file'] 
        carsData = None
        with open(self.cars_frenet_file) as json_file2:
            carsData = json.load(json_file2)
        egoData = None
        with open(self.ego_frenet_file) as json_file3:
            egoData = json.load(json_file3)
        self.getGroupBySec(carsData);
        self.getGroupEgoBySec(egoData);
        
        
        if "cut-in scenario" in allScenarios.keys():
            cutInScenarios = allScenarios["cut-in scenario"]
            for scenario in cutInScenarios:
                print("_________Cut-in scenario car: {}______________".format(scenario["cutin_car"]))
                start = scenario["scenario_start"]
                end = scenario["scenario_end"]
                laneChangeCar = scenario["cutin_car"]
                cars = self.getAllCars(start=start, end=end, laneChangeCar=laneChangeCar) 
                group_by_car = self.groupByCar(cars)
                print("all cars: {}".format(group_by_car.keys()))
                if laneChangeCar in group_by_car.keys(): 
                    ego = self.getEgo(start, end)
                    lane_change_car_data_by_sec = self.getLaneChangeCarBySec(laneChangeCar, group_by_car)
                    self.getParameters(ego, group_by_car, laneChangeCar, lane_change_car_data_by_sec, scenario, "cutin")
            self.saveData(self.parameterCutIn, "cutin");

        if "cut-out scenario" in allScenarios.keys():
            cutOutScenarios = allScenarios["cut-out scenario"]
            for scenario in cutOutScenarios:
                laneChangeCar = scenario["cutout_car"]
                print("_________Cut-out scenario car: {}______________".format(laneChangeCar))
                start = scenario["scenario_start"]
                end = scenario["scenario_end"]
                cars = self.getAllCars(start=start, end=end, laneChangeCar=laneChangeCar) 
                group_by_car = self.groupByCar(cars)
                print("all cars: {}".format(group_by_car.keys()))
                if laneChangeCar in group_by_car.keys(): 
                    ego = self.getEgo(start, end)
                    lane_change_car_data_by_sec = self.getLaneChangeCarBySec(laneChangeCar, group_by_car)
                    self.getParameters(ego, group_by_car, laneChangeCar, lane_change_car_data_by_sec, scenario, "cutout")

            self.saveData(self.parameterCutOut, "cutout")

        '''
        if "lane-following scenario" in allScenarios.keys():
            laneFollowingScenarios = allScenarios["lane-following scenario"]
            for scenario in laneFollowingScenarios:
                laneFollowingCar = scenario["laneFollowing_car"]
                print("_________Cut-out scenario car: {}______________".format(laneChangeCar))
                start = scenario["scenario_start"]
                end = scenario["scenario_end"]
                cars = self.getAllCars(start=start, end=end, laneChangeCar=laneChangeCar) 
                group_by_car = self.groupByCar(cars)
                print("all cars: {}".format(group_by_car.keys()))
                if laneChangeCar in group_by_car.keys(): 
                    ego = self.getEgo(start, end)
                    lane_following_car_data_by_sec = self.getLaneChangeCarBySec(laneFollowingCar, group_by_car)
                    self.getParameters(ego, group_by_car, laneFollowingCar, lane_following_car_data_by_sec, scenario, "lane-following")
        '''
    
    def getLaneChangeCarBySec(self, laneChangeCar, group_by_car):
        laneChangeCarData = group_by_car[laneChangeCar]
        laneChangeCarData_by_sec = OrderedDict()
        for data in laneChangeCarData:
            if data['frenet_data']['sec'] in laneChangeCarData_by_sec:
                laneChangeCarData_by_sec[data['frenet_data']['sec']].append(data)
            else:
                laneChangeCarData_by_sec[data['frenet_data']['sec']] = []
                laneChangeCarData_by_sec[data['frenet_data']['sec']].append(data)
        
        return laneChangeCarData_by_sec
    
    def getParameters(self, ego, group_by_car, laneChangeCar, lane_change_car_data_by_sec, scenario, _type):
        if _type == "cutin":
            lane_change_start_time = scenario['scenario_start']
        elif _type == "cutout":
            lane_change_start_time = scenario['scenario_start']
        elif _type == "lane-following":
            lane_change_start_time = scenario['scenario_start']
        count = 0
        found = False
        while True:
            if count > 20:
                break
            if self.group_ego_by_sec[lane_change_start_time][0]['other']['long_speed'] < 0.1:
                lane_change_start_time += 1
                count +=1
                continue
            if lane_change_start_time in lane_change_car_data_by_sec.keys():
                found = True
                break

            lane_change_start_time += 1
            count +=1
        

        proceed = False
        #laneChangeCarData = group_by_car[laneChangeCar]
        param_ego_lane_no_init = self.group_ego_by_sec[lane_change_start_time][0]['other']['lane_no']
        param_adv_lane_no_init = lane_change_car_data_by_sec[lane_change_start_time][0]['other']['lane_no']
        
        if _type == "cutin" and param_ego_lane_no_init != param_adv_lane_no_init:
            proceed = True
        elif _type == "cutout" and param_ego_lane_no_init == param_adv_lane_no_init:
            proceed = True
        elif _type == "lane-following" and param_ego_lane_no_init == param_adv_lane_no_init:
            procced = True

        print(param_ego_lane_no_init)
        print(param_adv_lane_no_init)
        
        # Find the cutin start
        if _type == "cutin":
            cut_start = scenario['cutin_start']
            cut_end = scenario['cutin_end']
            ego_lane_at_cutin = self.group_ego_by_sec[scenario['cutin_end']][0]['other']['lane_no']
            for sec in lane_change_car_data_by_sec.keys():
                if sec < scenario['cutin_start']:
                    continue

                if self.group_ego_by_sec[sec][0]['other']['long_speed'] < 0.1:
                    continue

                data_per_sec = lane_change_car_data_by_sec[sec] 
                lane_no = data_per_sec[0]['other']['lane_no']
                if ego_lane_at_cutin == lane_no:
                    cut_start = sec-2
                    break
        elif _type == "cutout":
            cut_start = scenario['cutout_start']
            cut_end = scenario['cutout_end']
            ego_lane_at_cutout = self.group_ego_by_sec[scenario['cutout_end']][0]['other']['lane_no']
            secs = lane_change_car_data_by_sec.keys() 
            for sec in secs:
                if sec < scenario['cutout_start']:
                    continue
                
                if self.group_ego_by_sec[sec][0]['other']['long_speed'] < 0.1:
                    continue

                data_per_sec = lane_change_car_data_by_sec[sec] 
                lane_no = data_per_sec[0]['other']['lane_no']
                if ego_lane_at_cutout != lane_no:
                    cut_start = sec-2
                    if cut_start not in secs:
                        proceed = False
                    break
        if cut_start < lane_change_start_time:
            proceed = False

        if found == True and proceed == True:
            start_time =  scenario['scenario_start']
            if _type == "cutin":
                lane_change_end_time = scenario['scenario_end']
            elif _type == "cutout":
                lane_change_end_time = scenario['scenario_end']
            elif _type == "lane-following":
                lane_change_end_time = scenario['scenario_end']
                if lane_change_end_time in lane_change_car_data_by_sec.keys():
                    while True:
                        if count > 40:
                            break
                        if start_time == lane_change_end_time and start_time in lane_change_car_data_by_sec.keys():
                            break
                        start_time += 1
                        count +=1
            
            ego_cut_start = self.group_ego_by_sec[cut_start][0]['frenet_data']['s']
            ego_cut_end = self.group_ego_by_sec[cut_end][0]['frenet_data']['s']
            param_ego_speed_init = self.group_ego_by_sec[lane_change_start_time][0]['other']['long_speed']
            param_ego_start_pos = self.group_ego_by_sec[lane_change_start_time][0]['frenet_data']['s']
            param_adv_start_pos = lane_change_car_data_by_sec[lane_change_start_time][0]['frenet_data']['s']
            # In data, other objects has realtive speed, so add it with param_ego_speed_init
            param_adv_speed_init = param_ego_speed_init+lane_change_car_data_by_sec[lane_change_start_time][0]['other']['long_speed'] #param_ego_speed_init
            print("param_adv_speed_init: {}".format(param_adv_speed_init))
            
            increase_pos = False  
            increase_pos_amount = 30
            if param_ego_start_pos < 30:
                param_ego_start_pos += increase_pos_amount
                param_adv_start_pos += increase_pos_amount
                ego_cut_start += increase_pos_amount 
                ego_cut_end += increase_pos_amount 
                increase_pos = True
            
            ego_cut_start_time = cut_start
            #---------Ego middle speed, distance from ego start to middle, time taken till cut start from last point
            ego_to_cut_start_dist = ego_cut_start-param_ego_start_pos
            param_ego_to_middle_dist = int(ego_to_cut_start_dist/2)
            ego_to_cut_start_time = self.group_ego_by_sec[cut_start][0]['frenet_data']['sec'] - self.group_ego_by_sec[lane_change_start_time][0]['frenet_data']['sec']
            param_ego_to_middle_time = int(ego_to_cut_start_time/2)
            middle_time_sec = self.group_ego_by_sec[lane_change_start_time][0]['frenet_data']['sec']+param_ego_to_middle_time
            param_ego_to_middle_speed = self.group_ego_by_sec[middle_time_sec][0]['other']['long_speed']
            print("param_ego_to_middle_speed: {}, param_ego_to_middle_time: {}".format(param_ego_to_middle_speed, param_ego_to_middle_time))

             #---------Ego cut start speed, distance from middle to cutstart, time taken till cut start from last point
            param_ego_middle_to_cut_start_dist = ego_cut_start-param_ego_start_pos
            param_ego_middle_to_cut_start_speed = self.group_ego_by_sec[cut_start][0]['other']['long_speed']
            param_ego_middle_to_cut_start_time = self.group_ego_by_sec[cut_start][0]['frenet_data']['sec'] - self.group_ego_by_sec[middle_time_sec][0]['frenet_data']['sec']
            print("param_ego_middle_to_cut_start_speed: {}, param_ego_middle_to_cut_start_time: {}".format(param_ego_middle_to_cut_start_speed, param_ego_middle_to_cut_start_time))

            # --------Ego cut-end speed, time taken from cut-in to cut-end, distance till cut-end------

            param_ego_cut_start_to_end_time = self.group_ego_by_sec[cut_end][0]['frenet_data']['sec'] - self.group_ego_by_sec[ego_cut_start_time][0]['frenet_data']['sec']
            param_ego_to_cutend_dist = ego_cut_end-param_ego_start_pos
            param_ego_cutend_speed = self.group_ego_by_sec[cut_end][0]['other']['long_speed']
            print("param_ego_cutend_speed: {}".format(param_ego_cutend_speed))
            
            #-------Ego final speed, total diatnce to end, and time taken from cut-in end to scenario end ----------------------------
            
            ego_end = cut_end
            param_ego_cutend_to_scenario_end_time = 0 #self.group_ego_by_sec[lane_change_end_time][0]['frenet_data']['sec'] - self.group_ego_by_sec[ego_end][0]['frenet_data']['sec']
            param_ego_to_scenario_end_dist = 0 #self.group_ego_by_sec[lane_change_end_time][0]['frenet_data']['s'] - param_ego_start_pos
            param_ego_speed_final = self.group_ego_by_sec[ego_end][0]['other']['long_speed']
            print("param_ego_speed_final: {}".format(param_ego_speed_final))

            #---------------Starting difference----------------------------
            
            param_start_diff=(lane_change_car_data_by_sec[lane_change_start_time][0]['frenet_data']['s']-self.group_ego_by_sec[lane_change_start_time][0]['frenet_data']['s'])
            
            if param_start_diff == 0.0:
                param_start_diff = 0.01
            
            print("Start diff: {}, adversary start pos: {}, ego start pos: {}".format(param_start_diff, param_adv_start_pos, param_ego_start_pos))
           
            #---------------Find cut-in time and adversary's final lane number----------- 
            found = False
            param_cut_time = lane_change_end_time-lane_change_start_time
            param_adv_lane_no_final = param_adv_lane_no_init
            cutin_start_sec = lane_change_start_time
            cutin_end_sec = None
            for sec in lane_change_car_data_by_sec.keys():
                if sec in lane_change_car_data_by_sec.keys():
                    size = len(lane_change_car_data_by_sec[sec])-1
                    lane_no = lane_change_car_data_by_sec[sec][size]['other']['lane_no']
                    if lane_no != param_adv_lane_no_init:
                        param_adv_lane_no_final = lane_change_car_data_by_sec[sec][size]['other']['lane_no']
                        cutin_end_sec = sec
                        found = True
                        break
            if found == True:
                param_cut_time = (cutin_end_sec-cutin_start_sec)-1
            
            #param_cut_time = 3
            startDiffProceed = True
            time_is_not_found = False
            
            adv_end = cut_end
            break_count = 0
            while adv_end not in lane_change_car_data_by_sec.keys():
                if break_count > 10:
                    time_is_not_found = True
                    break;
                adv_end += 1
                break_count += 1

            break_count = 0
            adv_cut_start_time = cut_start
            while adv_cut_start_time not in lane_change_car_data_by_sec.keys():
                if break_count > 10:
                    time_is_not_found = True
                    break;

                adv_cut_start_time += 1
                break_count += 1

            break_count = 10
            adv_scenario_end_time = lane_change_end_time
            while adv_scenario_end_time not in lane_change_car_data_by_sec.keys():
                if break_count <= 0:
                    time_is_not_found = True
                    break;

                adv_scenario_end_time -= 1
                break_count -= 1

            
            if time_is_not_found == False:
                # Find cut-in distance:
                if increase_pos:
                    adv_cut_start = lane_change_car_data_by_sec[adv_cut_start_time][0]['frenet_data']['s']+ increase_pos_amount
                    adv_cut_end_pos = lane_change_car_data_by_sec[adv_end][0]['frenet_data']['s']+ increase_pos_amount
                    adv_scenario_end_pos = lane_change_car_data_by_sec[adv_scenario_end_time][0]['frenet_data']['s']+ increase_pos_amount

                else:
                    adv_cut_start = lane_change_car_data_by_sec[adv_cut_start_time][0]['frenet_data']['s']
                    adv_cut_end_pos = lane_change_car_data_by_sec[adv_end][0]['frenet_data']['s']
                    adv_scenario_end_pos = lane_change_car_data_by_sec[adv_scenario_end_time][0]['frenet_data']['s']+ increase_pos_amount
                
                
                #------------------------triggering distance--------------------------
                param_cut_triggering_dist = adv_cut_start - ego_cut_start
                print("param_cut_triggering_dist: {}".format(param_cut_triggering_dist))
                
                '''if _type == "cutin":
                    print("init_diff: {} and param_cutin_start_dist: {}".format(param_start_diff, param_cutin_start_dist))
                    start_diff = abs(param_start_diff-param_cutin_start_dist)
                    if start_diff > 45:
                        startDiffProceed = False
                elif _type == "cutout":
                    param_cutout_start_dist = adv_cut_start - ego_cutout_start
                    print("init_diff: {} and param_cutout_start_dist: {}".format(param_start_diff, param_cutout_start_dist))
                '''
                
                #---------Adversary middle to cut start speed, distance from adv to cutstart, time taken till cut start from last point
                adv_to_cut_start_dist = adv_cut_start-param_adv_start_pos
                param_adv_to_middle_dist = int(adv_to_cut_start_dist/2)
                adv_to_cut_start_time = lane_change_car_data_by_sec[adv_cut_start_time][0]['frenet_data']['sec'] - lane_change_car_data_by_sec[lane_change_start_time][0]['frenet_data']['sec']
                param_adv_to_middle_time = int(adv_to_cut_start_time/2) 
                middle_time_sec = lane_change_car_data_by_sec[lane_change_start_time][0]['frenet_data']['sec']+param_adv_to_middle_time
                param_adv_to_middle_speed = self.group_ego_by_sec[middle_time_sec][0]['other']['long_speed'] + lane_change_car_data_by_sec[middle_time_sec][0]['other']['long_speed']
                print("param_adv_to_middle_speed: {}, param_adv_to_middle_time: {}".format(param_adv_to_middle_speed, param_adv_to_middle_time))
                

                #---------Adversary cut start speed, distance from adv to cutstart, time taken till cut start from last point
                param_adv_middle_to_cut_start_dist = adv_cut_start-param_adv_start_pos
                param_adv_middle_to_cut_start_speed = self.group_ego_by_sec[adv_cut_start_time][0]['other']['long_speed'] + lane_change_car_data_by_sec[adv_cut_start_time][0]['other']['long_speed']
                param_adv_middle_to_cut_start_time = lane_change_car_data_by_sec[adv_cut_start_time][0]['frenet_data']['sec'] - lane_change_car_data_by_sec[middle_time_sec][0]['frenet_data']['sec']
                print("param_adv_middle_to_cut_start_speed: {}, param_adv_middle_to_cut_start_time: {}".format(param_adv_middle_to_cut_start_speed, param_adv_middle_to_cut_start_time))
                
                
                #---------Adversary cut end speed, distance from adv to cutend, time taken to cut end from  cut start from last point
                param_adv_to_cutend_dist = adv_cut_end_pos-param_adv_start_pos
                param_adv_cutend_speed = self.group_ego_by_sec[adv_end][0]['other']['long_speed'] + lane_change_car_data_by_sec[adv_end][0]['other']['long_speed']
                param_adv_cut_start_to_end_time = lane_change_car_data_by_sec[adv_end][0]['frenet_data']['sec'] - lane_change_car_data_by_sec[adv_cut_start_time][0]['frenet_data']['sec']
                param_adv_cut_start_to_end_time_taken = lane_change_car_data_by_sec[adv_end][0]['frenet_data']['sec'] - lane_change_car_data_by_sec[lane_change_start_time][0]['frenet_data']['sec']

                print("param_adv_cutend_speed: {}, param_adv_cut_start_to_end_time: {}".format(param_adv_cutend_speed, param_adv_cut_start_to_end_time))
                #-------Adversary speed final/ distance to sceanrio end, time taken from last point to final point------------------
                # Calculating the cutin/cutout final speed and time to reach that and
                # distance travelled to get there.
                # Time calculated is from cut-in end time to scenario end time
                param_adv_cutend_to_scenario_end_time = lane_change_car_data_by_sec[adv_scenario_end_time][0]['frenet_data']['sec'] - lane_change_car_data_by_sec[adv_end][0]['frenet_data']['sec']
                param_adv_to_scenario_end_dist = adv_scenario_end_pos-param_adv_start_pos
                print("param_adv_to_scenario_end_dist: {}".format(param_adv_to_scenario_end_dist))
                param_adv_speed_final = self.group_ego_by_sec[adv_scenario_end_time][0]['other']['long_speed'] + lane_change_car_data_by_sec[adv_scenario_end_time][0]['other']['long_speed']
                param_adv_cutend_to_scenario_end_time_taken = lane_change_car_data_by_sec[adv_scenario_end_time][0]['frenet_data']['sec'] - lane_change_car_data_by_sec[lane_change_start_time][0]['frenet_data']['sec']
                print("param_adv_speed_final: {}, param_adv_cutend_to_scenario_end_time: {}".format(param_adv_speed_final, param_adv_cutend_to_scenario_end_time))
               
                #--------------------------Trajectory data----------------------------
                param_relative_lane_pos = []
                time_diff = lane_change_end_time - lane_change_start_time
                start_time = lane_change_start_time
                start_time += self.window
                second = self.window
                param_all_data = {
                    'ego': [],
                    'other': [],
                    'other_sec': {}
                }
                while start_time <= lane_change_end_time:
                    if start_time in self.group_ego_by_sec.keys() and start_time in lane_change_car_data_by_sec.keys():
                        #-----------------------sec data------------------------
                        ego_s = self.group_ego_by_sec[start_time][0]['frenet_data']['s']
                        ego_d = self.group_ego_by_sec[start_time][0]['frenet_data']['d']
                        ego_speed = self.group_ego_by_sec[start_time][0]['other']['long_speed']
                        ego_lane_no = self.group_ego_by_sec[start_time][0]['other']['lane_no']
                        other_s = lane_change_car_data_by_sec[start_time][0]['frenet_data']['s']
                        other_d = lane_change_car_data_by_sec[start_time][0]['frenet_data']['d']
                        other_speed = ego_speed+lane_change_car_data_by_sec[start_time][0]['other']['long_speed']
                        other_lane_no = lane_change_car_data_by_sec[start_time][0]['other']['lane_no']
                                                
                        if increase_pos == True:
                            ego_s += increase_pos_amount
                            other_s += increase_pos_amount
                        
                        if second == 1:
                            param_adv_speed_init = other_speed
                            #param_adv_start_pos = other_s
                            param_adv_lane_no_init = other_lane_no
                            param_ego_speed_init = ego_speed
                            #param_ego_start_pos = ego_s
                            param_ego_lane_no_init = ego_lane_no

                        ego_to_dist = ego_s-param_ego_start_pos
                        adv_to_dist = other_s-param_adv_start_pos
                        
                        data = {
                            'ego': {
                                's': ego_s,
                                'd': ego_d,
                                'speed': ego_speed,
                                'lane': ego_lane_no,
                                'start_to_current_dist': ego_to_dist,
                                'sec_count': second
                            },
                            'other': {
                                's': other_s,
                                'd': other_d, 
                                'speed': other_speed,
                                'lane': other_lane_no,
                                'start_to_current_dist': adv_to_dist,
                                'sec_count': second
                            }
                        }
                        param_relative_lane_pos.append(data)

                        #--------------------all data------------------
                        for each in self.group_ego_by_sec[start_time]:
                            ego_s = each['frenet_data']['s']
                            ego_d = each['frenet_data']['d']
                            ego_speed = each['other']['long_speed']
                            ego_lane_no = each['other']['lane_no']
                            if increase_pos == True:
                                ego_s += increase_pos_amount
                            ego_to_dist = ego_s-param_ego_start_pos
                            data = {
                                's': ego_s,
                                'd': ego_d,
                                'speed': ego_speed,
                                'lane': ego_lane_no,
                                'start_to_current_dist': ego_to_dist-2
                            }
                            param_all_data['ego'].append(data)
                        ego_count = 0
                        param_all_data['other_sec'][second] = []
                        for each in lane_change_car_data_by_sec[start_time]:
                            other_s = each['frenet_data']['s']
                            other_d = each['frenet_data']['d']
                            other_speed = self.group_ego_by_sec[start_time][ego_count]['other']['long_speed']+each['other']['long_speed']
                            other_lane_no = each['other']['lane_no']
                            if increase_pos == True:
                                other_s += increase_pos_amount
                            adv_to_dist = other_s-param_adv_start_pos
                            data = {
                                's': other_s,
                                'd': other_d,
                                'speed': other_speed,
                                'lane': other_lane_no,
                                'start_to_current_dist': adv_to_dist-2
                            }
                            param_all_data['other'].append(data)
                            param_all_data['other_sec'][second].append(data)
                            
                            ego_count += 1


                        start_time += self.window
                        second += self.window
                    else:
                        start_time += 1
                        second += 1

                #and final pos from lane_change_end_time
                if time_diff%self.window != 0:
                    data = {
                        'ego': {
                            's': self.group_ego_by_sec[lane_change_end_time][0]['frenet_data']['s'],
                            'lane': self.group_ego_by_sec[lane_change_end_time][0]['other']['lane_no'],
                            'speed':self.group_ego_by_sec[lane_change_end_time][0]['other']['long_speed'], 
                            'sec_count': time_diff
                        },
                        'other': {
                            's': lane_change_car_data_by_sec[lane_change_end_time][0]['frenet_data']['s'],
                            'lane':lane_change_car_data_by_sec[lane_change_end_time][0]['other']['lane_no'],
                            'speed': self.group_ego_by_sec[lane_change_end_time][0]['other']['long_speed']+lane_change_car_data_by_sec[lane_change_end_time][0]['other']['long_speed'],
                            'sec_count': time_diff
                        }
                    }
                    param_relative_lane_pos.append(data)

                    #--------------------all data------------------
                    for each in self.group_ego_by_sec[lane_change_end_time]:
                        ego_s = each['frenet_data']['s']
                        ego_d = each['frenet_data']['d']
                        ego_speed = each['other']['long_speed']
                        ego_lane_no = each['other']['lane_no']
                        if increase_pos == True:
                            ego_s += increase_pos_amount
                        ego_to_dist = ego_s-param_ego_start_pos
                        data = {
                            's': ego_s,
                            'd': ego_d,
                            'speed': ego_speed,
                            'lane': ego_lane_no,
                            'start_to_current_dist': ego_to_dist
                        }
                        param_all_data['ego'].append(data)

                    for each in lane_change_car_data_by_sec[lane_change_end_time]:
                        other_s = each['frenet_data']['s']
                        other_d = each['frenet_data']['d']
                        other_speed = each['other']['long_speed']
                        other_lane_no = each['other']['lane_no']
                        if increase_pos == True:
                            other_s += increase_pos_amount
                        adv_to_dist = other_s-param_adv_start_pos
                        data = {
                            's': other_s,
                            'd': other_d,
                            'speed': other_speed,
                            'lane': other_lane_no,
                            'start_to_current_dist': adv_to_dist
                        }
                        param_all_data['other'].append(data)


                #print(param_relative_lane_pos)
                
                if _type == "cutin":
                    laneChangeCar = scenario["cutin_car"]
                elif _type == "cutout":
                    laneChangeCar = scenario["cutout_car"]
                
                #-------------------Caluclate RSS---------------------------
                param_rss_dist = self.rss.calculate_rss_safe_dist(param_ego_speed_final, param_adv_speed_final) 
               
                print("RSS dist: {}".format(param_rss_dist))

                
                if _type == "cutin" and len(param_relative_lane_pos) > 3:
                    # some modification
                    if param_cut_time == 0:
                        param_cut_time = 4

                    trigger_cond = 0
                    if param_start_diff > param_cut_triggering_dist:
                        trigger_cond = 0
                    elif param_start_diff == param_cut_triggering_dist:
                        trigger_cond = 0
                    else:
                        trigger_cond = 1
                     
                    if param_cut_triggering_dist >= 0: 
                        param = {
                                'param_ego_speed_init': param_ego_speed_init,
                                'param_ego_lane_no_init': param_ego_lane_no_init,
                                'param_ego_start_pos': param_ego_start_pos,
                                'param_adv_speed_init': param_adv_speed_init,
                                'param_adv_lane_no_init': param_adv_lane_no_init,
                                'param_start_diff': param_start_diff,
                                'param_adv_start_pos': param_adv_start_pos,
                                'param_cut_triggering_dist': param_cut_triggering_dist,
                                'param_cut_time': param_cut_time,
                                'param_trigger_cond': trigger_cond,
                                'param_relative_lane_pos': param_relative_lane_pos,
                                'param_lane_change_carid': laneChangeCar,
                                'param_ego_to_middle_dist': param_ego_to_middle_dist,
                                'param_ego_to_middle_time': param_ego_to_middle_time,
                                'param_ego_to_middle_speed': param_ego_to_middle_speed,
                                'param_ego_middle_to_cut_start_dist': param_ego_middle_to_cut_start_dist,
                                'param_ego_middle_to_cut_start_time': param_ego_middle_to_cut_start_time,
                                'param_ego_middle_to_cut_start_speed': param_ego_middle_to_cut_start_speed,
                                'param_ego_to_cutend_dist': param_ego_to_cutend_dist,
                                'param_ego_cut_start_to_end_time': param_ego_cut_start_to_end_time,
                                'param_ego_cutend_speed': param_ego_cutend_speed,
                                'param_ego_to_scenario_end_dist': param_ego_to_scenario_end_dist,
                                'param_ego_cutend_to_scenario_end_time': param_ego_cutend_to_scenario_end_time,
                                'param_ego_speed_final': param_ego_speed_final,
                                'param_adv_to_middle_dist': param_adv_to_middle_dist,
                                'param_adv_to_middle_time': param_adv_to_middle_time,
                                'param_adv_to_middle_speed': param_adv_to_middle_speed,
                                'param_adv_middle_to_cut_start_dist': param_adv_middle_to_cut_start_dist,
                                'param_adv_middle_to_cut_start_time': param_adv_middle_to_cut_start_time,
                                'param_adv_middle_to_cut_start_speed': param_adv_middle_to_cut_start_speed,
                                'param_adv_to_cutend_dist': param_adv_to_cutend_dist,
                                'param_adv_cut_start_to_end_time': param_adv_cut_start_to_end_time,
                                'param_adv_cutend_speed': param_adv_cutend_speed,
                                'param_adv_to_scenario_end_dist': param_adv_to_scenario_end_dist,
                                'param_adv_cutend_to_scenario_end_time': param_adv_cutend_to_scenario_end_time,
                                'param_adv_speed_final': param_adv_speed_final,
                                'param_all_data': param_all_data

                        }
                        self.parameterCutIn.append(param)
                        print("Saving cut-in data")
                    else:
                        print("Not saved cut-in data")
                
                elif _type == "cutout" and len(param_relative_lane_pos) > 3:
                    # some modification
                    if param_cut_time == 0:
                        param_cut_time = 4

                    trigger_cond = 0
                    if param_start_diff > param_cut_triggering_dist:
                        trigger_cond = 0
                    elif param_start_diff == param_cut_triggering_dist:
                        trigger_cond = 0
                    else:
                        trigger_cond = 1

                    param = {
                            'param_ego_speed_init': param_ego_speed_init,
                            'param_ego_lane_no_init': param_ego_lane_no_init,
                            'param_ego_start_pos': param_ego_start_pos,
                            'param_adv_speed_init': param_adv_speed_init,
                            'param_adv_lane_no_init': param_adv_lane_no_init,
                            'param_start_diff': param_start_diff,
                            'param_adv_start_pos': param_adv_start_pos,
                            'param_cut_triggering_dist': param_cut_triggering_dist,
                            'param_cut_time': param_cut_time,
                            'param_cut_time': param_cut_time,
                            'param_trigger_cond': trigger_cond,
                            'param_adv_lane_no_final': param_adv_lane_no_final,
                            'param_relative_lane_pos': param_relative_lane_pos,
                            'param_lane_change_carid': laneChangeCar,
                            'param_ego_to_middle_dist': param_ego_to_middle_dist,
                            'param_ego_to_middle_time': param_ego_to_middle_time,
                            'param_ego_to_middle_speed': param_ego_to_middle_speed,
                            'param_ego_middle_to_cut_start_dist': param_ego_middle_to_cut_start_dist,
                            'param_ego_middle_to_cut_start_time': param_ego_middle_to_cut_start_time,
                            'param_ego_middle_to_cut_start_speed': param_ego_middle_to_cut_start_speed,
                            'param_ego_to_cutend_dist': param_ego_to_cutend_dist,
                            'param_ego_cut_start_to_end_time': param_ego_cut_start_to_end_time,
                            'param_ego_cutend_speed': param_ego_cutend_speed,
                            'param_ego_to_scenario_end_dist': param_ego_to_scenario_end_dist,
                            'param_ego_cutend_to_scenario_end_time': param_ego_cutend_to_scenario_end_time,
                            'param_ego_speed_final': param_ego_speed_final,
                            'param_adv_to_middle_dist': param_adv_to_middle_dist,
                            'param_adv_to_middle_time': param_adv_to_middle_time,
                            'param_adv_to_middle_speed': param_adv_to_middle_speed,
                            'param_adv_middle_to_cut_start_dist': param_adv_middle_to_cut_start_dist,
                            'param_adv_middle_to_cut_start_time': param_adv_middle_to_cut_start_time,
                            'param_adv_middle_to_cut_start_speed': param_adv_middle_to_cut_start_speed,
                            'param_adv_to_cutend_dist': param_adv_to_cutend_dist,
                            'param_adv_cut_start_to_end_time': param_adv_cut_start_to_end_time,
                            'param_adv_cutend_speed': param_adv_cutend_speed,
                            'param_adv_to_scenario_end_dist': param_adv_to_scenario_end_dist,
                            'param_adv_cutend_to_scenario_end_time': param_adv_cutend_to_scenario_end_time,
                            'param_adv_speed_final': param_adv_speed_final,
                            'param_all_data': param_all_data

                    }
                    self.parameterCutOut.append(param)
                    print("Saving cut-out data")
                    
                else:
                    print("Not saved the lane change data")
            else:
                print("Not saved the data: time is not found")
        else:
            print("Not saved the data")

    def saveData(self, parameter, _type="cutin"):
        data = {
            'type': _type,
            'parameter': parameter 
        }
        base = os.path.basename(self.bag_file)
        filename = os.path.splitext(base)[0]
        with open(self.parameter_dir+filename+"."+_type, 'w') as outfile:
            json.dump(data, outfile)

    def getGroupBySec(self, carsData):
        for eachData in carsData:
            for data in eachData: 
                if data["frenet_data"]["sec"] in self.group_by_sec:
                    self.group_by_sec[data["frenet_data"]["sec"]].append(data);
                else:
                    self.group_by_sec[data["frenet_data"]["sec"]] = []
                    self.group_by_sec[data["frenet_data"]["sec"]].append(data)

    def getGroupEgoBySec(self, egoData):
        for data in egoData:
            if data["frenet_data"]["sec"] in self.group_ego_by_sec:
                self.group_ego_by_sec[data["frenet_data"]["sec"]].append(data);
            else:
                self.group_ego_by_sec[data["frenet_data"]["sec"]] = []
                self.group_ego_by_sec[data["frenet_data"]["sec"]].append(data)

    def getEgo(self, start, end):
        data = []
        for sec in self.group_ego_by_sec:
            if sec >= start and sec <= end:
                data.append(self.group_ego_by_sec[sec]);
        
        return data


    def getAllCars(self, start = None, end = None, laneChangeCar = None):
        data = []
        for sec in self.group_by_sec:
            if sec >= start and sec <= end:
                data.append(self.group_by_sec[sec]);
        
        return data

    def groupByCar(self, listData):
        group_by_car = OrderedDict()
        for eachData in listData:
            for data in eachData:
                if data["car_id"] in group_by_car:
                    group_by_car[data["car_id"]].append(data);
                else:
                    group_by_car[data["car_id"]] = [];
                    group_by_car[data["car_id"]].append(data);
        
        return group_by_car
            
    def shutdown(self):
        print("FeatureModel node is shutting down!!!")

if __name__ == '__main__':
    try:
        FeatureModel()
    except rospy.ROSInterruptException:
	rospy.logerr('Could not start FeatureModel node.')
