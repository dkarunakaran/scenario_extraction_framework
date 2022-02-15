from scenariogeneration import xodr
from scenariogeneration import xosc, prettyprint
import os
import json
import sys
import ctypes
from ctypes import *

class Generate:
    def __init__(self):
        self.save_loc = '/home/beastan/Documents/phd/scenario_extraction/parameters/scenario_files/'
        self.open_loc = '/home/beastan/Documents/phd/scenario_extraction/parameters/scenario_data/'
        #self.shape = xosc.DynamicsShapes.linear
        self.shape = xosc.DynamicsShapes.sinusoidal
        #self.shape = xosc.DynamicsShapes.cubic 
    
    def generate_openscenario_trajectory(self, _file, filename):
        with open(_file) as f:
            data = json.loads(f.read())
        print("Processing the file: {}".format(_file))
        self.cutinoutScenarioTraj(_file, filename, data)
    
    def cutinoutScenarioTraj(self, _file, filename, data):
        _type = data['type']
        count = 1
        for param in data['parameter']:
            name = self.save_loc+filename+"_"+_type+"_"+str(count)+".xosc"

            ### create catalogs
            catalog = xosc.Catalog()
            catalog.add_catalog('VehicleCatalog','../xosc/Catalogs/Vehicles')

            road = xosc.RoadNetwork(roadfile=filename+'.xodr')

            ### create parameters
            ego_vehicle = 'hero'
            ego_velocity_init = param['param_ego_speed_init'] # param 0
            ego_start_pos = param['param_ego_start_pos'] # param 1 - s coordinate of road frame
            #ego_start_pos = param['param_relative_lane_pos'][0]['ego']['s'] 
            ego_start_lane = param['param_ego_lane_no_init'] # param 2 - lane number
            adversary_vehicle = 'adversary'
            adversary_velocity_init = param['param_adv_speed_init'] # param 3
            adversary_pos = param['param_adv_start_pos'] # param 4 - front=0 or back=1
            #adversary_pos = param['param_relative_lane_pos'][0]['other']['s']
            adversary_lane = param['param_adv_lane_no_init'] # param 6 - lane number
            adversary_vehicle_model = 0  # param 9 - car=0, van=1, motorbike=2
            lanechange_car_id = param['param_lane_change_carid']

            paramdec = xosc.ParameterDeclarations()
            paramdec.add_parameter(xosc.Parameter('$egoVehicle',xosc.ParameterType.string, ego_vehicle))
            paramdec.add_parameter(xosc.Parameter('$egoVelocityInit',xosc.ParameterType.double, ego_velocity_init))
            paramdec.add_parameter(xosc.Parameter('$egoStartPos',xosc.ParameterType.double, ego_start_pos))
            paramdec.add_parameter(xosc.Parameter('$egoStartLane',xosc.ParameterType.double, ego_start_lane))
            paramdec.add_parameter(xosc.Parameter('$adversaryVehicle',xosc.ParameterType.string, adversary_vehicle))
            paramdec.add_parameter(xosc.Parameter('$adversaryVelocityInit',xosc.ParameterType.double, adversary_velocity_init))
            paramdec.add_parameter(xosc.Parameter('$adversaryPos',xosc.ParameterType.integer, adversary_pos))
            paramdec.add_parameter(xosc.Parameter('$adversaryLane',xosc.ParameterType.double,adversary_lane))
            paramdec.add_parameter(xosc.Parameter('$lanechangeCarId',xosc.ParameterType.double, lanechange_car_id))
            
            # vehicle.tesla.model3
            bb = xosc.BoundingBox(2.1,4.5,1.8,1.5,0,0.9)
            fa = xosc.Axle(0.5,0.6,1.8,3.1,0.3)
            ba = xosc.Axle(0.0,0.6,1.8,0.0,0.3)
            ego_veh = xosc.Vehicle('$egoVehicle',xosc.VehicleCategory.car,bb,fa,ba,100,10,10)
            ego_veh.add_property('model_id','0')
            ego_veh.add_property('type','ego_vehicle')
            bb = None
            fa = None
            ba = None
            if adversary_vehicle_model == 0:
                bb = xosc.BoundingBox(2.0,5.0,1.8,1.4,0.0,0.9)
                fa = xosc.Axle(0.5,0.8,1.68,2.98,0.4)
                ba = xosc.Axle(0.0,0.8,1.68,0.0,0.4)
            elif adversary_vehicle_model == 1:
                bb = xosc.BoundingBox(1.8,4.5,1.5,1.3,0.0,0.8)
                fa = xosc.Axle(0.5,0.8,1.68,2.98,0.4)
                ba = xosc.Axle(0.0,0.8,1.68,0.0,0.4)
            elif adversary_vehicle_model == 2:
                bb = xosc.BoundingBox(0.9,2.2,1.3,0.40,0.0,0.65)
                fa = xosc.Axle(1.5,0.7,0.1,1.5,0.35)
                ba = xosc.Axle(0.0,0.7,0.1,0.0,0.35)

            adversary_veh = xosc.Vehicle('$adversaryVehicle',xosc.VehicleCategory.car,bb,fa,ba,100,10,10)
            adversary_veh.add_property('model_id','1')
            
            # Adding carla controller
            prop = xosc.Properties()
            prop.add_property(name="module", value="external_control")
            cont = xosc.Controller('carlaControler',prop)

            entities = xosc.Entities()
            entities.add_scenario_object('$egoVehicle',ego_veh, controller=cont)
            entities.add_scenario_object('$adversaryVehicle',adversary_veh)

            ### create init
            init = xosc.Init()
            step_time = xosc.TransitionDynamics(xosc.DynamicsShapes.step,xosc.DynamicsDimension.time,1)
            egospeed = xosc.AbsoluteSpeedAction('$egoVelocityInit',step_time)
            egostart = xosc.TeleportAction(xosc.LanePosition('$egoStartPos',0, '$egoStartLane',"0"))

            targetspeed = xosc.AbsoluteSpeedAction('$adversaryVelocityInit',step_time)
            targetstart = xosc.TeleportAction(xosc.LanePosition('$adversaryPos',0,'$adversaryLane',"0"))

            init.add_init_action('$egoVehicle',egospeed)
            init.add_init_action('$egoVehicle',egostart)
            init.add_init_action('$adversaryVehicle',targetspeed)
            init.add_init_action('$adversaryVehicle',targetstart)

            
            #trigcond = xosc.TimeHeadwayCondition('$adversaryVehicle',0.4,xosc.Rule.greaterThan)
            #trigger = xosc.EntityTrigger('advCutInTrajTrigger',0.2,xosc.ConditionEdge.rising,trigcond,'$egoVehicle')
            trig_cond1 = xosc.TraveledDistanceCondition(0.1)
            trigger = xosc.EntityTrigger('advCutInOutTrigger',0.2,xosc.ConditionEdge.rising,trig_cond1,'$egoVehicle')

            event = xosc.Event('advCutInOutTrajEvent',xosc.Priority.overwrite)
            event.add_trigger(trigger)
            positionlist = []
            sec = []
            for data in param['param_relative_lane_pos']:
                positionlist.append(xosc.LanePosition(data['other']['s'], 0, data['other']['lane'], "0"))
                sec.append(data['other']['sec_count']);
            print(sec)
            polyline = xosc.Polyline(sec,positionlist)
            traj = xosc.Trajectory('advCutInOutTraj',False)
            traj.add_shape(polyline)
            trajact = xosc.FollowTrajectoryAction(traj,xosc.FollowMode.position,xosc.ReferenceContext.relative,1,0)

            event.add_action('advEventActionTraj',trajact)

            man = xosc.Maneuver('advCutInOutManeuver')
            man.add_event(event)
            mangr = xosc.ManeuverGroup('advCutInOutMangroup')
            mangr.add_actor('$adversaryVehicle')
            mangr.add_maneuver(man)
            starttrigger = xosc.ValueTrigger('advCutInOutStartTrigger',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
            actAdv = xosc.Act('advCutInOutAct',starttrigger)
            actAdv.add_maneuver_group(mangr)

            
            #---------trajectory ego----------------
            trig_cond1 = xosc.TraveledDistanceCondition(0.1)
            trigger = xosc.EntityTrigger('egoCutInOutTrigger',0.2,xosc.ConditionEdge.rising,trig_cond1,'$egoVehicle')

            event = xosc.Event('egoCutInOutTrajEvent',xosc.Priority.overwrite)
            event.add_trigger(trigger)
            positionlist = []
            sec = []
            for data in param['param_relative_lane_pos']:
                positionlist.append(xosc.LanePosition(data['ego']['s'], 0, data['ego']['lane'], "0"))
                sec.append(data['ego']['sec_count']);
            polyline = xosc.Polyline(sec,positionlist)
            traj = xosc.Trajectory('egoCutInOutTraj',False)
            traj.add_shape(polyline)
            trajact = xosc.FollowTrajectoryAction(traj,xosc.FollowMode.position,xosc.ReferenceContext.relative,1,0)

            event.add_action('egoEventActionTraj',trajact)

            man = xosc.Maneuver('egoCutInOutManeuver')
            man.add_event(event)
            mangr = xosc.ManeuverGroup('egoCutInOutMangroup')
            mangr.add_actor('$egoVehicle')
            mangr.add_maneuver(man)
            starttrigger = xosc.ValueTrigger('egoCutInOutStartTrigger',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
            actEgo = xosc.Act('egoCutInOutAct',starttrigger)
            actEgo.add_maneuver_group(mangr)

            story = xosc.Story('cutInStory')
            story.add_act(actAdv)
            story.add_act(actEgo)

            ## create the storyboard
            sb = xosc.StoryBoard(init, xosc.ValueTrigger('stop_simulation',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(30,xosc.Rule.greaterThan),'stop'))
            sb.add_story(story)

            ## create the scenario
            sce = xosc.Scenario('cutinScenario','Dhanoop',paramdec,entities=entities,storyboard=sb,roadnetwork=road,catalog=catalog)

            # Print the resulting xml
            #prettyprint(sce.get_element())

            # write the OpenSCENARIO file as xosc using current script name
            #sce.write_xml(os.path.basename(__file__).replace('.py','.xosc'))
            sce.write_xml(name)
            count += 1

         

    def generate_openscenario(self, _file, filename):
        with open(_file) as f:
            data = json.loads(f.read())
        print("Processing the file: {}".format(_file))
        _type = data['type']
        if _type == 'cutin':
            self.cutinScenario(_file, filename, data)
        #if _type == 'cutout':
        #    self.cutOutScenario(_file, filename, data)
        
    def cutOutScenario(self, _file, filename, data):
        # GLOBAL PARAMETERS
        param_adv_percentage_dist_to_cutout_dist = 100
        param_ego_percentage_dist_to_cutout_dist = 100
        _type = data['type']
        count = 1
        for param in data['parameter']:
            name = self.save_loc+filename+"_"+_type+"_"+str(count)+".xosc"
            print(param)

            ### create catalogs
            catalog = xosc.Catalog()
            catalog.add_catalog('VehicleCatalog','../xosc/Catalogs/Vehicles')

            road = xosc.RoadNetwork(roadfile=filename+'.xodr')

            ### create parameters
            ego_vehicle = 'hero'
            ego_velocity_init = param['param_ego_speed_init'] # param 0
            ego_start_pos = param['param_ego_start_pos'] # param 1 - s coordinate of road frame
            ego_start_lane = param['param_ego_lane_no_init'] # param 2 - lane number
            adversary_vehicle = 'adversary'
            adversary_velocity_init = param['param_adv_speed_init'] # param 3
            # Difference between ego vehicle and adversary, positive value indicates the
            # Target location is ahead of ego vehicle
            adversary_pos = param['param_adv_start_pos'] # param 4 - front=0 or back=1
            adversary_start_diff = param['param_start_diff'] # param 5  - difference in s coordinate of road frame
            adversary_lane = param['param_adv_lane_no_init'] # param 6 - lane number
            adversary_cutout_speed = 40 # param 7 
            adversary_cutout_trigger_dist = param['param_cutout_start_dist'] # param 8 - distance in s from target to ego
            adversary_vehicle_model = 0  # param 9 - car=0, van=1, motorbike=2

            adversary_cutout_time = param['param_cutout_time'] # Param 10 time taken to complete the lane change
            adversary_to_cutout_time = param['param_adv_to_cutout_time']
            adversary_to_cutout_dist = param['param_adv_to_cutout_dist']
            adversary_avg_cutout_speed = param['param_adv_cutout_speed']
            ego_to_cutout_dist = param['param_ego_to_cutout_dist']
            ego_to_cutout_time = param['param_ego_to_cutout_time']
            ego_avg_cutout_speed = param['param_ego_cutout_speed']
            trigger_cond = param['param_trigger_cond']
            adversary_lane_final = param['param_adv_lane_no_final'] # param 6 - lane number
            adversary_to_cutout_end_dist = param['param_adv_to_cutout_end_dist']
            adversary_speed_final = param['param_adv_speed_final']
            adversary_to_cutout_end_time = param['param_adv_to_cutout_end_time']
            ego_to_cutout_end_dist = param['param_ego_to_cutout_end_dist']
            ego_speed_final = param['param_ego_speed_final']
            ego_to_cutout_end_time = param['param_ego_to_cutout_end_time']
            lanechange_car_id = param['param_lane_change_carid']

            paramdec = xosc.ParameterDeclarations()
            paramdec.add_parameter(xosc.Parameter('$egoVehicle',xosc.ParameterType.string, ego_vehicle))
            paramdec.add_parameter(xosc.Parameter('$egoVelocityInit',xosc.ParameterType.double, ego_velocity_init))
            paramdec.add_parameter(xosc.Parameter('$egoStartPos',xosc.ParameterType.double, ego_start_pos))
            paramdec.add_parameter(xosc.Parameter('$egoStartLane',xosc.ParameterType.double, ego_start_lane))
            paramdec.add_parameter(xosc.Parameter('$adversaryVehicle',xosc.ParameterType.string, adversary_vehicle))
            paramdec.add_parameter(xosc.Parameter('$adversaryVelocityInit',xosc.ParameterType.double, adversary_velocity_init))
            paramdec.add_parameter(xosc.Parameter('$adversaryPos',xosc.ParameterType.double, adversary_pos))
            paramdec.add_parameter(xosc.Parameter('$adversaryLane',xosc.ParameterType.integer,adversary_lane))
            paramdec.add_parameter(xosc.Parameter('$adversaryCutoutSpeed',xosc.ParameterType.double,adversary_cutout_speed))
            paramdec.add_parameter(xosc.Parameter('$adversaryStartDiff',xosc.ParameterType.double,adversary_start_diff))
            paramdec.add_parameter(xosc.Parameter('$adversaryCutoutTriggerDist',xosc.ParameterType.double,adversary_cutout_trigger_dist))
            paramdec.add_parameter(xosc.Parameter('$adversaryCutoutTime',xosc.ParameterType.double,adversary_cutout_time))
            paramdec.add_parameter(xosc.Parameter('$adversaryToCutoutDist',xosc.ParameterType.double,adversary_to_cutout_dist))
            paramdec.add_parameter(xosc.Parameter('$adversaryToCutoutTime',xosc.ParameterType.double,adversary_to_cutout_time))
            paramdec.add_parameter(xosc.Parameter('$adversaryAvgCutoutSpeed',xosc.ParameterType.double,adversary_avg_cutout_speed))
            paramdec.add_parameter(xosc.Parameter('$egoToCutoutDist',xosc.ParameterType.double,ego_to_cutout_dist))
            paramdec.add_parameter(xosc.Parameter('$egoToCutoutTime',xosc.ParameterType.double, ego_to_cutout_time))
            paramdec.add_parameter(xosc.Parameter('$egoAvgCutoutSpeed',xosc.ParameterType.double, ego_avg_cutout_speed))
            paramdec.add_parameter(xosc.Parameter('$adversaryLaneFinal',xosc.ParameterType.integer,adversary_lane_final))
            paramdec.add_parameter(xosc.Parameter('$adversaryToCutoutEndDist',xosc.ParameterType.double,adversary_to_cutout_end_dist))
            paramdec.add_parameter(xosc.Parameter('$adversarySpeedFinal',xosc.ParameterType.double, adversary_speed_final))
            paramdec.add_parameter(xosc.Parameter('$egoToCutoutEndDist',xosc.ParameterType.double,ego_to_cutout_end_dist))
            paramdec.add_parameter(xosc.Parameter('$egoSpeedFinal',xosc.ParameterType.double, ego_speed_final))
            paramdec.add_parameter(xosc.Parameter('$adversaryToCutoutEndTime',xosc.ParameterType.double,adversary_to_cutout_end_time))
            paramdec.add_parameter(xosc.Parameter('$egoToCutoutEndTime',xosc.ParameterType.double, ego_to_cutout_end_time))
            paramdec.add_parameter(xosc.Parameter('$lanechangeCarId',xosc.ParameterType.double, lanechange_car_id))
            
            # vehicle.tesla.model3
            bb = xosc.BoundingBox(2.1,4.5,1.8,1.5,0,0.9)
            fa = xosc.Axle(0.5,0.6,1.8,3.1,0.3)
            ba = xosc.Axle(0.0,0.6,1.8,0.0,0.3)
            ego_veh = xosc.Vehicle('$egoVehicle',xosc.VehicleCategory.car,bb,fa,ba,100,10,10)
            ego_veh.add_property('model_id','0')
            ego_veh.add_property('type','ego_vehicle')
            bb = None
            fa = None
            ba = None
            if adversary_vehicle_model == 0:
                bb = xosc.BoundingBox(2.0,5.0,1.8,1.4,0.0,0.9)
                fa = xosc.Axle(0.5,0.8,1.68,2.98,0.4)
                ba = xosc.Axle(0.0,0.8,1.68,0.0,0.4)
            elif adversary_vehicle_model == 1:
                bb = xosc.BoundingBox(1.8,4.5,1.5,1.3,0.0,0.8)
                fa = xosc.Axle(0.5,0.8,1.68,2.98,0.4)
                ba = xosc.Axle(0.0,0.8,1.68,0.0,0.4)
            elif adversary_vehicle_model == 2:
                bb = xosc.BoundingBox(0.9,2.2,1.3,0.40,0.0,0.65)
                fa = xosc.Axle(1.5,0.7,0.1,1.5,0.35)
                ba = xosc.Axle(0.0,0.7,0.1,0.0,0.35)

            adversary_veh = xosc.Vehicle('$adversaryVehicle',xosc.VehicleCategory.car,bb,fa,ba,100,10,10)
            adversary_veh.add_property('model_id','1')
            
            # Adding carla controller
            prop = xosc.Properties()
            prop.add_property(name="module", value="external_control")
            cont = xosc.Controller('carlaControler',prop)

            entities = xosc.Entities()
            entities.add_scenario_object('$egoVehicle',ego_veh, controller=cont)
            entities.add_scenario_object('$adversaryVehicle',adversary_veh)

            ## create init
            init = xosc.Init()
            step_time = xosc.TransitionDynamics(xosc.DynamicsShapes.step,xosc.DynamicsDimension.time,1)
            egospeed = xosc.AbsoluteSpeedAction('$egoVelocityInit',step_time)
            egostart = xosc.TeleportAction(xosc.LanePosition('$egoStartPos',0, '$egoStartLane',"0"))

            targetspeed = xosc.AbsoluteSpeedAction('$adversaryVelocityInit',step_time)
            targetstart = xosc.TeleportAction(xosc.LanePosition('$adversaryPos',0,'$adversaryLane',"0"))

            init.add_init_action('$egoVehicle',egospeed)
            init.add_init_action('$egoVehicle',egostart)
            init.add_init_action('$adversaryVehicle',targetspeed)
            init.add_init_action('$adversaryVehicle',targetstart)

            # create event and act1 - cutout
            trig_cond1 = None
            if trigger_cond == 0:
                trig_cond1=xosc.RelativeDistanceCondition('$adversaryCutoutTriggerDist',xosc.Rule.lessThan,dist_type=xosc.RelativeDistanceType.longitudinal,entity='$adversaryVehicle')
            else:
                trig_cond1=xosc.RelativeDistanceCondition('$adversaryCutoutTriggerDist',xosc.Rule.greaterThan,dist_type=xosc.RelativeDistanceType.longitudinal,entity='$adversaryVehicle')
            trigger = xosc.EntityTrigger('cutOutTrigger',0,xosc.ConditionEdge.rising,trig_cond1,'$egoVehicle')
            event = xosc.Event('cutOnevent',xosc.Priority.overwrite)
            event.add_trigger(trigger)

            sin_time = xosc.TransitionDynamics(self.shape,xosc.DynamicsDimension.time,'$adversaryCutoutTime')
            action = xosc.AbsoluteLaneChangeAction('$adversaryLaneFinal',sin_time)
            event.add_action('cutOutLaneAction',action)
            man = xosc.Maneuver('cutOutManeuver')
            man.add_event(event)
            mangr = xosc.ManeuverGroup('cutOutMangroup')
            mangr.add_actor('$adversaryVehicle')
            mangr.add_maneuver(man)
            starttrigger = xosc.ValueTrigger('cutOutStartTrigger',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
            act = xosc.Act('cutOutAct',starttrigger)
            act.add_maneuver_group(mangr)
           
            # create event and act 2 - adversary vehicle
            #adv_start_to_cutout_dist = (adversary_to_cutout_dist*param_adv_percentage_dist_to_cutout_dist)/100
            trig_cond1 = xosc.TraveledDistanceCondition(adversary_to_cutout_dist)
            trigger = xosc.EntityTrigger('cutOutTriggerAdv',0.2,xosc.ConditionEdge.rising,trig_cond1,'$adversaryVehicle')
            event = xosc.Event('cutOuteventAdv',xosc.Priority.parallel)
            event.add_trigger(trigger)
            lin_time = xosc.TransitionDynamics(self.shape ,xosc.DynamicsDimension.time,'$adversaryToCutoutTime')
            action = xosc.AbsoluteSpeedAction('$adversaryAvgCutoutSpeed',lin_time)
            event.add_action('cutOutSpeedActionAdv',action)
           
            man = xosc.Maneuver('cutOutManeuverAdv')
            man.add_event(event)
            mangr = xosc.ManeuverGroup('cutOutMangroupAdv')
            mangr.add_actor('$adversaryVehicle')
            mangr.add_maneuver(man)
            starttrigger = xosc.ValueTrigger('cutOutStartTriggerAdv',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
            actAdv = xosc.Act('cutOutActAdv',starttrigger)
            actAdv.add_maneuver_group(mangr)

            # create event and act 3 - ego vehicle
            '''
            if ego_to_cutout_dist > 20:
                ego_start_to_cutout_dist = 20
            else:
                ego_start_to_cutout_dist = (ego_to_cutout_dist*param_ego_percentage_dist_to_cutout_dist)/100
            '''
            trig_cond1 = xosc.TraveledDistanceCondition(ego_to_cutout_dist)
            trigger = xosc.EntityTrigger('cutOutTriggerEgo',0.2,xosc.ConditionEdge.rising,trig_cond1,'$egoVehicle')
            event = xosc.Event('cutOuteventEgo',xosc.Priority.parallel)
            event.add_trigger(trigger)
            lin_time = xosc.TransitionDynamics(self.shape,xosc.DynamicsDimension.time,'$egoToCutoutTime')
            action = xosc.AbsoluteSpeedAction('$egoAvgCutoutSpeed',lin_time)
            event.add_action('cutOutSpeedActionEgo',action)
            man = xosc.Maneuver('cutOutManeuverEgo')
            man.add_event(event)
            mangr = xosc.ManeuverGroup('cutOutMangroupEho')
            mangr.add_actor('$egoVehicle')
            mangr.add_maneuver(man)
            starttrigger = xosc.ValueTrigger('cutOutStartTriggerEgo',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
            actEgo = xosc.Act('cutOutActEgo',starttrigger)
            actEgo.add_maneuver_group(mangr)
            
            # create event and act 4 - adversary vehicle final speed
            trig_cond1 = xosc.TraveledDistanceCondition('$adversaryToCutoutEndDist')
            trigger = xosc.EntityTrigger('cutoutTriggerAdvFinal',0.2,xosc.ConditionEdge.rising,trig_cond1,'$adversaryVehicle')
            event = xosc.Event('cutOuteventAdvFinal',xosc.Priority.parallel)
            event.add_trigger(trigger)
            lin_time = xosc.TransitionDynamics(self.shape,xosc.DynamicsDimension.time,'$adversaryToCutoutEndTime')
            action = xosc.AbsoluteSpeedAction('$adversarySpeedFinal',lin_time)
            event.add_action('cutOutSpeedActionAdvFinal',action)
           
            man = xosc.Maneuver('cutOutManeuverAdvFinal')
            man.add_event(event)
            mangr = xosc.ManeuverGroup('cutOutMangroupAdvFinal')
            mangr.add_actor('$adversaryVehicle')
            mangr.add_maneuver(man)
            starttrigger = xosc.ValueTrigger('cutOutStartTriggerAdvFinal',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
            actAdvFinal = xosc.Act('cutOutActAdvFinal',starttrigger)
            actAdvFinal.add_maneuver_group(mangr)

            # create event and act 5 - ego vehicle final speed
            trig_cond1 = xosc.TraveledDistanceCondition('$egoToCutoutEndDist')
            trigger = xosc.EntityTrigger('cutoutTriggerEgoFinal',0.2,xosc.ConditionEdge.rising,trig_cond1,'$egoVehicle')
            event = xosc.Event('cutOuteventEgoFinal',xosc.Priority.parallel)
            event.add_trigger(trigger)
            lin_time = xosc.TransitionDynamics(self.shape,xosc.DynamicsDimension.time,'$egoToCutoutEndTime')
            action = xosc.AbsoluteSpeedAction('$egoSpeedFinal',lin_time)
            event.add_action('cutOutSpeedActionEgoFinal',action)
            man = xosc.Maneuver('cutOutManeuverEgoFinal')
            man.add_event(event)
            mangr = xosc.ManeuverGroup('cutOutMangroupEgoFinal')
            mangr.add_actor('$egoVehicle')
            mangr.add_maneuver(man)
            starttrigger = xosc.ValueTrigger('cutOutStartTriggerEgoFinal',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
            actEgoFinal = xosc.Act('cutOutActEgoFinal',starttrigger)
            actEgoFinal.add_maneuver_group(mangr)



            ## create the story
            #storyparam = xosc.ParameterDeclarations()
            #storyparam.add_parameter(xosc.Parameter('$owner',xosc.ParameterType.string,'$adversaryVehicle'))
            #story = xosc.Story('cutInStory',storyparam)
            story = xosc.Story('cutOutStory')
            story.add_act(act)
            story.add_act(actAdv)
            story.add_act(actEgo)
            story.add_act(actAdvFinal)
            story.add_act(actEgoFinal)


            ## create the storyboard
            sb = xosc.StoryBoard(init, xosc.ValueTrigger('stop_simulation',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(30,xosc.Rule.greaterThan),'stop'))
            sb.add_story(story)

            ## create the scenario
            sce = xosc.Scenario('cutOutScenario','Dhanoop',paramdec,entities=entities,storyboard=sb,roadnetwork=road,catalog=catalog)

            # Print the resulting xml
            #prettyprint(sce.get_element())

            # write the OpenSCENARIO file as xosc using current script name
            #sce.write_xml(os.path.basename(__file__).replace('.py','.xosc'))
            sce.write_xml(name)
            count += 1

    
    def cutinScenario(self, _file, filename, data):

        # GLOBAL PARAMETERS
        param_adv_percentage_dist_to_cutin_dist = 100
        param_ego_percentage_dist_to_cutin_dist = 100
        _type = data['type']
        count = 1
        for param in data['parameter']:
            print(count)
            #if count ==2 or count == 3:
            #    count += 1
            #    continue
            name = self.save_loc+filename+"_"+_type+"_"+str(count)+".xosc"
            #print(param)

            ### create catalogs
            catalog = xosc.Catalog()
            catalog.add_catalog('VehicleCatalog','../xosc/Catalogs/Vehicles')
            road = xosc.RoadNetwork(roadfile=filename+'.xodr')

            ### create parameters
            ego_vehicle = 'hero'
            ego_velocity_init = param['param_ego_speed_init'] # param 0
            ego_start_pos = param['param_ego_start_pos'] # param 1 - s coordinate of road frame
            ego_start_lane = param['param_ego_lane_no_init'] # param 2 - lane number
            adversary_vehicle = 'adversary'
            adversary_velocity_init = param['param_adv_speed_init'] # param 3
            # Difference between ego vehicle and adversary, positive value indicates the
            # Target location is ahead of ego vehicle
            adversary_pos = param['param_adv_start_pos'] # param 4 - front=0 or back=1
            adversary_start_diff = param['param_start_diff'] # param 5  - difference in s coordinate of road frame
            adversary_lane = param['param_adv_lane_no_init'] # param 6 - lane number
            adversary_cutin_speed = 40 # param 7 
            adversary_cutin_trigger_dist = param['param_cut_triggering_dist']-5 # param 8 - distance in s from target to ego
            adversary_vehicle_model = 0  # param 9 - car=0, van=1, motorbike=2
            adversary_cutin_time = param['param_cut_time'] # Param 10 time taken to complete the lane change
            trigger_cond = param['param_trigger_cond']
            lanechange_car_id = param['param_lane_change_carid']
            param_ego_to_middle_dist = param['param_ego_to_middle_dist']
            param_ego_to_middle_time = param['param_ego_to_middle_time']
            param_ego_to_middle_speed = param['param_ego_to_middle_speed']
            param_ego_middle_to_cut_start_dist = param['param_ego_middle_to_cut_start_dist']
            param_ego_middle_to_cut_start_time = param['param_ego_middle_to_cut_start_time']
            param_ego_middle_to_cut_start_speed = param['param_ego_middle_to_cut_start_speed']
            param_ego_to_cutend_dist = param['param_ego_to_cutend_dist']
            param_ego_cut_start_to_end_time = param['param_ego_cut_start_to_end_time']
            param_ego_cutend_speed = param['param_ego_cutend_speed']
            param_ego_to_scenario_end_dist = param['param_ego_to_scenario_end_dist']
            param_ego_cutend_to_scenario_end_time = param['param_ego_cutend_to_scenario_end_time']
            param_ego_speed_final = param['param_ego_speed_final']
            
            param_adv_to_middle_dist = param['param_adv_to_middle_dist']
            param_adv_to_middle_time = param['param_adv_to_middle_time']
            param_adv_to_middle_speed = param['param_adv_to_middle_speed']
            param_adv_middle_to_cut_start_dist = param['param_adv_middle_to_cut_start_dist']
            param_adv_middle_to_cut_start_time = param['param_adv_middle_to_cut_start_time']
            param_adv_middle_to_cut_start_speed = param['param_adv_middle_to_cut_start_speed']

            param_adv_to_cutend_dist = param['param_adv_to_cutend_dist']
            param_adv_cut_start_to_end_time = param['param_adv_cut_start_to_end_time']
            param_adv_cutend_speed = param['param_adv_cutend_speed']
            param_adv_to_scenario_end_dist = param['param_adv_to_scenario_end_dist']
            param_adv_cutend_to_scenario_end_time = param['param_adv_cutend_to_scenario_end_time']
            param_adv_speed_final = param['param_adv_speed_final']

            # Some modification
            if ego_start_pos < 1.0:
                ego_start_pos +=30
                adversary_pos +=30


            paramdec = xosc.ParameterDeclarations()
            paramdec.add_parameter(xosc.Parameter('$egoVehicle',xosc.ParameterType.string, ego_vehicle))
            paramdec.add_parameter(xosc.Parameter('$egoVelocityInit',xosc.ParameterType.double, ego_velocity_init))
            paramdec.add_parameter(xosc.Parameter('$egoStartPos',xosc.ParameterType.double, ego_start_pos))
            paramdec.add_parameter(xosc.Parameter('$egoStartLane',xosc.ParameterType.double, ego_start_lane))
            paramdec.add_parameter(xosc.Parameter('$adversaryVehicle',xosc.ParameterType.string, adversary_vehicle))
            paramdec.add_parameter(xosc.Parameter('$adversaryVelocityInit',xosc.ParameterType.double, adversary_velocity_init))
            paramdec.add_parameter(xosc.Parameter('$adversaryPos',xosc.ParameterType.integer, adversary_pos))
            paramdec.add_parameter(xosc.Parameter('$adversaryLane',xosc.ParameterType.double,adversary_lane))
            paramdec.add_parameter(xosc.Parameter('$adversaryCutinSpeed',xosc.ParameterType.double,adversary_cutin_speed))
            paramdec.add_parameter(xosc.Parameter('$adversaryStartDiff',xosc.ParameterType.double,adversary_start_diff))
            paramdec.add_parameter(xosc.Parameter('$adversaryCutinTriggerDist',xosc.ParameterType.double,adversary_cutin_trigger_dist))
            paramdec.add_parameter(xosc.Parameter('$adversaryCutinTime',xosc.ParameterType.double,adversary_cutin_time))
            paramdec.add_parameter(xosc.Parameter('$lanechangeCarId',xosc.ParameterType.double, lanechange_car_id))

            
            # vehicle.tesla.model3
            bb = xosc.BoundingBox(2.1,4.5,1.8,1.5,0,0.9)
            fa = xosc.Axle(0.5,0.6,1.8,3.1,0.3)
            ba = xosc.Axle(0.0,0.6,1.8,0.0,0.3)
            ego_veh = xosc.Vehicle('$egoVehicle',xosc.VehicleCategory.car,bb,fa,ba,100,10,10)
            ego_veh.add_property('model_id','0')
            ego_veh.add_property('type','ego_vehicle')
            bb = None
            fa = None
            ba = None
            if adversary_vehicle_model == 0:
                bb = xosc.BoundingBox(2.0,5.0,1.8,1.4,0.0,0.9)
                fa = xosc.Axle(0.5,0.8,1.68,2.98,0.4)
                ba = xosc.Axle(0.0,0.8,1.68,0.0,0.4)
            elif adversary_vehicle_model == 1:
                bb = xosc.BoundingBox(1.8,4.5,1.5,1.3,0.0,0.8)
                fa = xosc.Axle(0.5,0.8,1.68,2.98,0.4)
                ba = xosc.Axle(0.0,0.8,1.68,0.0,0.4)
            elif adversary_vehicle_model == 2:
                bb = xosc.BoundingBox(0.9,2.2,1.3,0.40,0.0,0.65)
                fa = xosc.Axle(1.5,0.7,0.1,1.5,0.35)
                ba = xosc.Axle(0.0,0.7,0.1,0.0,0.35)

            adversary_veh = xosc.Vehicle('$adversaryVehicle',xosc.VehicleCategory.car,bb,fa,ba,100,10,10)
            adversary_veh.add_property('model_id','1')
            
            # Adding carla controller
            prop = xosc.Properties()
            prop.add_property(name="module", value="external_control")
            cont = xosc.Controller('carlaControler',prop)

            entities = xosc.Entities()
            entities.add_scenario_object('$egoVehicle',ego_veh, controller=cont)
            entities.add_scenario_object('$adversaryVehicle',adversary_veh)

            ### create init
            init = xosc.Init()
            step_time = xosc.TransitionDynamics(xosc.DynamicsShapes.step,xosc.DynamicsDimension.time,1)
            egospeed = xosc.AbsoluteSpeedAction('$egoVelocityInit',step_time)
            egostart = xosc.TeleportAction(xosc.LanePosition('$egoStartPos',0, '$egoStartLane',"0"))

            targetspeed = xosc.AbsoluteSpeedAction('$adversaryVelocityInit',step_time)
            targetstart = xosc.TeleportAction(xosc.LanePosition('$adversaryPos',0,'$adversaryLane',"0"))

            init.add_init_action('$egoVehicle',egospeed)
            init.add_init_action('$egoVehicle',egostart)
            init.add_init_action('$adversaryVehicle',targetspeed)
            init.add_init_action('$adversaryVehicle',targetstart)

            # create event and act1 - cutin
            trig_cond1 = None
            if trigger_cond == 0:
                trig_cond1=xosc.RelativeDistanceCondition('$adversaryCutinTriggerDist',xosc.Rule.lessThan,dist_type=xosc.RelativeDistanceType.longitudinal,entity='$adversaryVehicle')
            else:
                trig_cond1=xosc.RelativeDistanceCondition('$adversaryCutinTriggerDist',xosc.Rule.greaterThan,dist_type=xosc.RelativeDistanceType.longitudinal,entity='$adversaryVehicle')
            trigger = xosc.EntityTrigger('cutinTrigger',0,xosc.ConditionEdge.rising,trig_cond1,'$egoVehicle')
            event = xosc.Event('cutInevent',xosc.Priority.overwrite)
            event.add_trigger(trigger)
            sin_time= xosc.TransitionDynamics(self.shape,xosc.DynamicsDimension.time,'$adversaryCutinTime')
            action = xosc.AbsoluteLaneChangeAction('$egoStartLane',sin_time)
            event.add_action('cutInLaneAction',action)
            man = xosc.Maneuver('cutInManeuver')
            man.add_event(event)
            mangr = xosc.ManeuverGroup('cutInMangroup')
            mangr.add_actor('$adversaryVehicle')
            mangr.add_maneuver(man)
            starttrigger=xosc.ValueTrigger('cutinStartTrigger',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0.01,xosc.Rule.greaterThan))
            act = xosc.Act('cutInAct',starttrigger)
            act.add_maneuver_group(mangr)
            
            # create the story
            #storyparam = xosc.ParameterDeclarations()
            #storyparam.add_parameter(xosc.Parameter('$owner',xosc.ParameterType.string,'$adversaryVehicle'))
            #story = xosc.Story('cutInStory',storyparam)
            story = xosc.Story('cutInStory')
            story.add_act(act)
            sampling_type = 1 #1: second based sampling 2: millisecond sampling
           
            if sampling_type == 1:
                actEgoList = []
                actAdvList = []
                #Equally dividing
                allowed = []
                num = len(param['param_relative_lane_pos'])
                print(num) 
                #value = 1 # 2 parameters
                #value = num*(2/4) #0.5
                #value = num*(4/4) # full
                value = num
                div = int(value)
                section = self.split(num, div)
                add = 0;
                allowed.append(0)
                prev = 0
                s_count = 1
                for each in section:
                    add += each 
                    temp = add-1
                    if s_count == len(section):
                        if (add-1) not in allowed: 
                            allowed.append(add-1) #add-1
                    else:
                        allowed.append(add)
                    s_count += 1
                print(allowed)
                
                prev = 0 
                print("----------------------------------------------")
                for index in range(len(param['param_relative_lane_pos'])):
                    
                    #if index != 0 and index != len(param['param_relative_lane_pos'])-2:
                    #    continue
                    
                    # This controls the number of parameters
                    if index not in allowed:
                        continue

                    if (index+1) > len(param['param_relative_lane_pos']):
                        break
                    if (index+1) == len(param['param_relative_lane_pos']):
                        data_current = param['param_relative_lane_pos'][index]
                        data_next = param['param_relative_lane_pos'][index]
                        data_prev = param['param_relative_lane_pos'][index-1]
                    else:
                        data_current = param['param_relative_lane_pos'][index]
                        data_next = param['param_relative_lane_pos'][index+1]
                        data_prev = param['param_relative_lane_pos'][index-1]
                    
                    print(index)
                    print("speed:{}".format(data_current['other']['speed']))
                    print(data_current['other']['start_to_current_dist'])
                    print("*")

                    #if index == 0:
                    #    continue
                    
                    actCount = index
                    
                    #Ego
                    if index == 0:
                        trig_cond1 = xosc.TraveledDistanceCondition(1)
                    else:
                        trig_cond1 = xosc.TraveledDistanceCondition(data_current['ego']['start_to_current_dist'])
                    #trig_cond1 = xosc.TraveledDistanceCondition(data_current['ego']['start_to_current_dist'])
                    trigger=xosc.EntityTrigger('cutinTriggerEgo'+str(actCount),0.01,xosc.ConditionEdge.risingOrFalling,trig_cond1,'$egoVehicle')
                    event = xosc.Event('cutIneventEgo'+str(actCount),xosc.Priority.parallel)
                    event.add_trigger(trigger)
                    duration = (index+1) - index
                    lin_time=xosc.TransitionDynamics(self.shape,xosc.DynamicsDimension.time,0.01)
                    action = xosc.AbsoluteSpeedAction(data_next['ego']['speed'],lin_time)
                    event.add_action('cutInSpeedActionEgo'+str(actCount),action)
                    man = xosc.Maneuver('cutInManeuverEgo'+str(actCount))
                    man.add_event(event)
                    mangr = xosc.ManeuverGroup('cutInMangroupEgo'+str(actCount))
                    mangr.add_actor('$egoVehicle')
                    mangr.add_maneuver(man)
                    starttrigger=xosc.ValueTrigger('cutinStartTriggerEgo'+str(actCount),0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0.01,xosc.Rule.greaterThan))
                    actEgo = xosc.Act('cutInActEgo'+str(actCount),starttrigger)
                    actEgo.add_maneuver_group(mangr)
                    actEgoList.append(actEgo)
                    
                    #Adversary
                    if index == 0:
                        trig_cond1 = xosc.TraveledDistanceCondition(1)
                    else:
                        trig_cond1=xosc.TraveledDistanceCondition(data_current['other']['start_to_current_dist'])
                    #trig_cond1 = xosc.TraveledDistanceCondition(data_current['other']['start_to_current_dist'])
                    trigger=xosc.EntityTrigger('cutinTriggerAdv'+str(actCount),0.01,xosc.ConditionEdge.risingOrFalling,trig_cond1,'$adversaryVehicle')
                    event = xosc.Event('cutIneventAdv'+str(actCount),xosc.Priority.parallel)
                    event.add_trigger(trigger)
                    duration = (index+1) - index
                    lin_time=xosc.TransitionDynamics(self.shape,xosc.DynamicsDimension.time,0.01)
                    action = xosc.AbsoluteSpeedAction(data_next['other']['speed'],lin_time)
                    event.add_action('cutInSpeedActionAdv'+str(actCount),action)
                    man = xosc.Maneuver('cutInManeuverAdv'+str(actCount))
                    man.add_event(event)
                    mangr = xosc.ManeuverGroup('cutInMangroupAdv'+str(actCount))
                    mangr.add_actor('$adversaryVehicle')
                    mangr.add_maneuver(man)
                    starttrigger=xosc.ValueTrigger('cutinStartTriggerAdv'+str(actCount),0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0.01,xosc.Rule.greaterThan))
                    actAdv = xosc.Act('cutInActAdv'+str(actCount),starttrigger)
                    actAdv.add_maneuver_group(mangr)
                    actAdvList.append(actAdv)
                    prev = index 
                
                for index in range(len(actEgoList)):
                    story.add_act(actEgoList[index])
                    story.add_act(actAdvList[index])

            if sampling_type == 2:
                actCount = 0
                actEgoList = []
                actAdvList = []
                #Equally dividing
                #value = 1 # 2 parameters
                #value = (1/4) #0.25
                #value = (2/4) #0.5
                #value = (4/4) # full
                value = 0.05


                print("----------------------Ego------------------------")
                allowed_ego = []
                num_ego = len(param['param_all_data']['ego'])
                print(num_ego)
                div = int(num_ego*value)
                section = self.split(num_ego, div)
                add = 0;
                s_count = 1
                for each in section:
                    add += each
                    temp = add-1
                    if s_count == len(section):
                        if (add-1) not in allowed_ego:
                            allowed_ego.append(add-1)
                    else:
                        allowed_ego.append(add)
                    s_count += 1
                print(allowed_ego)
                prev = 0 
                params = param['param_all_data']['ego']
                for index in range(len(params)):
                    
                    # This controls the number of parameters
                    if index not in allowed_ego:
                        continue

                    if (index+1) > len(params):
                        break
                    if (index+1) == len(params):
                        data_current = params[index]
                        data_next = params[index]
                        data_prev = params[index-1]
                    else:
                        data_current = params[index]
                        data_next = params[index+1]
                        data_prev = params[index-1]
                    
                    if index < 30:
                        print(index)
                        print(data_current['speed'])
                        print(data_current['start_to_current_dist'])
                        print("-----------")

                    #if index == 0:
                    #    continue
                    
                    #Ego
                    if index < 40:
                        trig_cond1 = xosc.TraveledDistanceCondition(0.1)
                    else: 
                        trig_cond1= xosc.TraveledDistanceCondition(data_current['start_to_current_dist'])
                    trigger=xosc.EntityTrigger('cutinTriggerEgo'+str(actCount),0.1,xosc.ConditionEdge.risingOrFalling,trig_cond1,'$egoVehicle')
                    event= xosc.Event('cutIneventEgo'+str(actCount),xosc.Priority.parallel)
                    event.add_trigger(trigger)
                    duration = (index+1) - index
                    lin_time=xosc.TransitionDynamics(xosc.DynamicsShapes.step,xosc.DynamicsDimension.rate,0.1)
                    action = xosc.AbsoluteSpeedAction(data_next['speed'],lin_time)
                    event.add_action('cutInSpeedActionEgo'+str(actCount),action)
                    man = xosc.Maneuver('cutInManeuverEgo'+str(actCount))
                    man.add_event(event)
                    mangr = xosc.ManeuverGroup('cutInMangroupEgo'+str(actCount))
                    mangr.add_actor('$egoVehicle')
                    mangr.add_maneuver(man)
                    starttrigger= xosc.ValueTrigger('cutinStartTriggerEgo'+str(actCount),0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0.1,xosc.Rule.greaterThan))
                    actEgo = xosc.Act('cutInActEgo'+str(actCount),starttrigger)
                    actEgo.add_maneuver_group(mangr)
                    actEgoList.append(actEgo)
                    actCount +=1
                    
                for index in range(len(actEgoList)):
                    story.add_act(actEgoList[index])

                print("-------------------Adv---------------------------")
                actCount = 0
                allowed_adv = []
                num_adv = len(param['param_all_data']['other'])
                print(num_adv)
                div = int(num_adv*value)
                section = self.split(num_adv, div)
                add = 0;
                s_count = 1
                for each in section:
                    add += each
                    temp = add-1
                    if s_count == len(section):
                        if (add-1) not in allowed_adv:
                            allowed_adv.append(add-1)
                    else:
                        allowed_adv.append(add)
                    s_count += 1
                print(allowed_adv)
                prev = 0 
                params = param['param_all_data']['other']
                for index in range(len(params)):
                    
                    # This controls the number of parameters
                    if index not in allowed_adv:
                        continue

                    if (index+1) > len(params):
                        break
                    if (index+1) == len(params):
                        data_current = params[index]
                        data_next = params[index]
                        data_prev = params[index-1]
                    else:
                        data_current = params[index]
                        data_next = params[index+1]
                        data_prev = params[index-1]
                    
                    if index < 0:
                        print(index)
                        print(data_current['speed'])
                        print(data_current['start_to_current_dist'])
                        print("*")

                    #Adversary
                    
                    if index < 20:
                        trig_cond1 = xosc.TraveledDistanceCondition(0.1)
                    else: 
                        trig_cond1 = xosc.TraveledDistanceCondition(data_current['start_to_current_dist'])
                    trigger = xosc.EntityTrigger('cutinTriggerAdv'+str(actCount),0.1,xosc.ConditionEdge.rising,trig_cond1,'$adversaryVehicle')
                    event = xosc.Event('cutIneventAdv'+str(actCount),xosc.Priority.parallel)
                    event.add_trigger(trigger)
                    duration = (index+1) - index
                    lin_time = xosc.TransitionDynamics(xosc.DynamicsShapes.linear,xosc.DynamicsDimension.time, 0.1)
                    action = xosc.AbsoluteSpeedAction(data_current['speed'],lin_time)
                    event.add_action('cutInSpeedActionAdv'+str(actCount),action)
                    man = xosc.Maneuver('cutInManeuverAdv'+str(actCount))
                    man.add_event(event)
                    mangr = xosc.ManeuverGroup('cutInMangroupAdv'+str(actCount))
                    mangr.add_actor('$adversaryVehicle')
                    mangr.add_maneuver(man)
                    starttrigger = xosc.ValueTrigger('cutinStartTriggerAdv'+str(actCount),0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(0,xosc.Rule.greaterThan))
                    actAdv = xosc.Act('cutInActAdv'+str(actCount),starttrigger)
                    actAdv.add_maneuver_group(mangr)
                    actAdvList.append(actAdv)
                    prev = index 
                    actCount +=1
                
                for index in range(len(actAdvList)):
                    story.add_act(actAdvList[index])


            ## create the storyboard
            sb = xosc.StoryBoard(init, xosc.ValueTrigger('stop_simulation',0,xosc.ConditionEdge.rising,xosc.SimulationTimeCondition(15,xosc.Rule.greaterThan),'stop'))
            sb.add_story(story)

            ## create the scenario
            sce = xosc.Scenario('cutinScenario','Dhanoop',paramdec,entities=entities,storyboard=sb,roadnetwork=road,catalog=catalog)

            # Print the resulting xml
            #prettyprint(sce.get_element())

            # write the OpenSCENARIO file as xosc using current script name
            #sce.write_xml(os.path.basename(__file__).replace('.py','.xosc'))
            sce.write_xml(name)
            count += 1

    def split(self, x, n):
        allowed = [] 
        # If we cannot split the
        # number into exactly 'N' parts
        if(x < n):
            return -1
     
        # If x % n == 0 then the minimum
        # difference is 0 and all
        # numbers are x / n
        elif (x % n == 0):
            for i in range(n):
                allowed.append(x//n)
        else:
            # upto n-(x % n) the values
            # will be x / n
            # after that the values
            # will be x / n + 1
            zp = n - (x % n)
            pp = x//n
            for i in range(n):
                if(i>= zp):
                    allowed.append(pp + 1)
                else:
                    allowed.append(pp)

        return allowed
           
    def generate_opendrive(self, _file, filename):
        with open(_file) as f:
            data = json.loads(f.read())
        print("Processing the file: {}".format(_file))

        # create the planview and the geometry
        planview = xodr.PlanView()
        prev = 0.
        for param in data['data']:
            planview.add_geometry(xodr.Spiral(param['mDiff'], prev, param['length']))
            prev = param['mDiff']

        rm_solid = xodr.RoadMark(xodr.RoadMarkType.solid,0.2)
        centerlane = xodr.Lane(a=2)
        centerlane.add_roadmark(rm_solid)

        prevLaneList = []
        count = 0
        laneSecList = []
        lanelinker = xodr.LaneLinker()
        prevNoLanes = 0 # this has to be replaced with param data
        noOfLanesStart = data['noOfLanesStart']
        for param in data['data']:
            noOfLanes = noOfLanesStart #param['noLanes']

            # create the lanesections
            lanesec = xodr.LaneSection(param['sStart'],centerlane)
            laneList = []
            for index in range(0, noOfLanes):
                lane = xodr.Lane(a=3)
                lane.add_roadmark(rm_solid)
                lanesec.add_right_lane(lane)
                laneList.append(lane)
            laneSecList.append(lanesec)

            # create the lane links
            if count > 0:
                for index in range(0, prevNoLanes):
                    lanelinker.add_link(predlane=prevLaneList[index],succlane=laneList[index])

            prevLaneList = laneList
            count += 1
            prevNoLanes = noOfLanes

        # create the lanes with the correct links
        lanes = xodr.Lanes()
        for index in range(0, len(laneSecList)):
            lanes.add_lanesection(laneSecList[index],lanelinker)

        # create the road
        road = xodr.Road(0,planview,lanes)

        # create the opendrive
        odr = xodr.OpenDrive('road')
        odr.add_road(road)

        # adjust the roads and lanes
        odr.adjust_roads_and_lanes()
        odr.write_xml(self.save_loc+filename+".xodr")



if __name__ == '__main__':
    gen = Generate()
    loc = '/home/beastan/Documents/phd/scenario_extraction/parameters/scenario_data/'
    _dir = os.listdir(loc)
    for _file in _dir:
        base = os.path.basename(_file)
        fileDetails = os.path.splitext(base)
        if fileDetails[1] == '.cutin' or fileDetails[1] == '.cutout':
            gen.generate_openscenario(loc+_file, fileDetails[0])
            #gen.generate_openscenario_trajectory(loc+_file, fileDetails[0])
            gen.generate_opendrive(loc+fileDetails[0]+".laneletToOD", fileDetails[0])
