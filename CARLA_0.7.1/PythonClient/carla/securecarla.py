# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Secure CARLA."""


from contextlib import contextmanager

from . import sensor
from . import settings
from . import tcp
from . import util
from . import image_converter
from . import camera_attack
import csv
import logging
import numpy as np
import time
import sys
import configparser
import os.path
import datetime
import Queue
from securecarlasensors import SCSensor
from liveplotter import Plotter
from collections import OrderedDict

try:
    from . import carla_server_pb2 as carla_protocol
except ImportError:
    raise RuntimeError('cannot import "carla_server_pb2.py", run the protobuf compiler to generate this file')


class SecureCarla(object):
    def __init__(self, config_file=None, config_camera=None):
	self.wait_counter = 0
	self.step = 0
	self.output_time = datetime.datetime.now()
	self.agent_num = 0
	self.first_reading = True
	self.past_player = None
        self.player_id = '00000000'
        self.plotter = Plotter()
        self.target_vehicle_id = 0
        # Load variance, mean, and offset parameters here:
        # Parse config file if config file provided 
        if config_file is not None:
            self.config = self.parse_config(config_file)
        if config_camera is not None:
            self.config_camera = self.parse_config(config_camera)
        self.csv_file = '../../securecarla_details.csv'

        # self.scsensors holds SCSensor objects that are encountered by the player
	# init an object for the player 
        self.scsensors = {}
        self.scsensors[self.player_id] = SCSensor()
        self.scsensors[self.player_id].agent_type = 'player'

        self._dict_distances = OrderedDict([('step', ''),
				('src_node', ''),
				('dest_node', ''),
                                ('sensor', ''),
                                ('agent_type', ''),
                                ('noise_distance', ''),
				('adversarial_distance', ''),
				('true_distance', ''),
                                ('noise_speed', ''),
                                ('adversarial_speed', ''),
                                ('true_speed', ''),
                                ('true_accel_x', ''),
                                ('true_accel_y', ''),
                                ('true_accel_z', ''),
                                ('adv_accel_x', ''),
                                ('adv_accel_y', ''),
                                ('adv_accel_z', ''),
                                ('pitch', ''),
                                ('yaw', ''),
                                ('roll', '')
                                ])
	#for s in range(0,int(self.config['all']['num_sensors'])):
	#	self._dict_distances['sensor{}'.format(s)] = -1

	self.meas_fifo = Queue.Queue() #to hold measurements during delay attack
        self.sensor_fifo = Queue.Queue() #to hold camera frames during delay attack
        self.meas_buf = []
        self.sensor_buf = []

	print self._dict_distances

	with open(self.csv_file, 'w') as rfd:

                rw = csv.DictWriter(rfd, self._dict_distances.keys())
                rw.writeheader()
        return
        
    def clearDict(self):
        for key in self._dict_distances:
            self._dict_distances[key] = ''

    def parse_config(self, config_file):

        config_params = {}
        
        # Initialize config parser object
        config = configparser.ConfigParser()

        if not os.path.isfile(config_file):
            print('Cannot find', config_file+'. ', 'Exiting now.')
            exit(1)
        else:
            # Read and parse config file
            config.read(config_file)
            
            for config_key in config.items():
                
                if config_key[0] == 'DEFAULT':
                    continue
                config_params[config_key[0]] = dict(config.items(config_key[0]))
                for key,val in config_params[config_key[0]].items():
                    if key == 'image_path' or key == 'use_attack_type':
                        config_params[config_key[0]][key] = str(val)
                    else:
                        config_params[config_key[0]][key] = float(val)
            
            # Print configuration parameters loaded
            print('Finished loading configs:')
            for key in config_params.keys():
                print('---',key,'---')
                print(config_params[key])

        return config_params

    def get_distance_to_agent(self, player, agent):
        distance = np.sqrt((player.transform.location.x - agent.transform.location.x)**2 + (player.transform.location.y - agent.transform.location.y)**2 + (player.transform.location.z - agent.transform.location.z)**2)

        return distance

    def gauss_var_from_dist(self, distance, fixed_variance):
        grad = fixed_variance/10
        variance = grad*distance
	return variance

    def uniform_parms_from_dist(self, distance, fixed_low, fixed_high):
        grad_low = fixed_low/10
        low = grad_low*distance

	grad_high = fixed_high/10
        high = grad_high*distance
	return low, high

    # Return which sensor can see the agent based on the player's yaw and agent's x,y location
    def is_in_sensors_fov(self, player, agent, distance_to_agent):
        sensor_fov = self.config['all']['sensor_fov']
        player_yaw = player.transform.rotation.yaw
        player_x = player.transform.location.x
        player_y = player.transform.location.y
        agent_x = agent.transform.location.x
        agent_y = agent.transform.location.y
        
        player_yaw = player_yaw if player_yaw > 0 else player_yaw + 360
        sensor_front_yaw = 0
        sensor_back_yaw = 180
        sensor_right_yaw = 90
        sensor_left_yaw = 270

        # Calculate agent's angle from player, convert to be within -360 to 360
        agent_angle = np.arctan2((agent_y - player_y), (agent_x - player_x)) * 180 / np.pi - player_yaw
        if agent_angle > 360:
            agent_angle = agent_angle - 360
        elif agent_angle < -sensor_fov:
            agent_angle = agent_angle + 360
        elif agent_angle == 360 or agent_angle == -360:
            agent_angle = 0
        

        # Determine which sensor's FOV the agent falls in. For now, just select one of the sensors
        # (basically assuming that only one distance sensor picking up the agent)
        # 0: SENSOR_FRONT, 1: SENSOR_BACK, 2: SENSOR_RIGHT, 3: SENSOR_RIGHT
        sensors = [sensor_front_yaw, sensor_back_yaw, sensor_right_yaw, sensor_left_yaw]
        #print agent_y, player_y, agent_x, player_x
        #print 'agent_angle', agent_angle
        #print 'player yaw', player_yaw, 'agent angle from x', np.arctan2((player_y-agent_y),(player_x - agent_x)) * 180 / np.pi
        
        which_sensor = None
        for i,sensor in enumerate(sensors):
            left_bound = sensor + sensor_fov
            right_bound = sensor - sensor_fov
            left_bound = left_bound if left_bound < 360 else left_bound - 360
            right_bound = right_bound if right_bound > -360 else right_bound + 360
            #print 'left_bound:',left_bound, 'right_bound',right_bound, 'agent_angle', agent_angle
            if agent_angle < left_bound and agent_angle > right_bound:
                which_sensor = i
                break

        return which_sensor, agent_angle
    
    # Returns the distance value under noise and attack 
    def distance_threshold_attack(self, this_config, agent, agent_id, player=None):
        sensors = ['front', 'back', 'right', 'left']
	distance = self.get_distance_to_agent(agent, player)
        	
	#if distance < 1000:
            #print("Distance: {}, Agent_ID: {}".format(distance,agent_id))
        """
        TODO: [pseudocode]
        get yaw, get FOV from config file, theta measured as angle of the sensor direction 
        loop through 4 sensors, if in the FOV of any of them, apply the attack
        is_in_sensors_fov function (player, agent, distance_to_agent):
            see if the agent's position relative to the player falls within any (choose 1) of the following:
            sensor_front: yaw = self.yaw
            sensor_left: yaw - 90
            sensor_right: yaw + 90
            sensor_back: yaw - 180
            remember to convert to within 0-360
        """
	which_sensor = None
        if(distance < self.config['all']['distance_threshold']):
            adversarial_distances = []
            noise_distances = []
            
            # Get the sensor that the agent is currently in the FOV of
            which_sensor, angle_from_player = self.is_in_sensors_fov(player, agent, distance)
            
            # Only attack and log if agent detected by a sensor
            if which_sensor is not None:
                print type(agent), 'sensor:', sensors[which_sensor], angle_from_player, 'deg', 'at', distance

                for s in range(0,int(self.config['all']['num_sensors'])):
                    use_gaussian = int(this_config['use_gaussian_noise'])
                    if use_gaussian:
                        # The variance is distance dependent
                        variance = self.gauss_var_from_dist(distance, this_config['dist_noise_var'])
                        noise = np.random.normal(this_config['dist_noise_mean'], variance)
                    else:
                        # The low and high parameters are distance dependent
                        low, high = self.uniform_parms_from_dist(distance, this_config['dist_noise_low'], this_config['dist_noise_high'])
                        noise = np.random.uniform(low, high) 
                    noise_distances.append(distance + noise)

                    use_attack = int(this_config['use_attack'])
                    if use_attack:
                        attack = np.random.normal(this_config['dist_attack_mean'], this_config['dist_attack_var'])
                    else:
                        attack = 0
                    adversarial_distances.append(distance + noise + attack)
                    
                self.scsensors[agent_id] = SCSensor()
                self.scsensors[agent_id].updateDistances(distance, noise_distances, adversarial_distances)
                self.scsensors[agent_id].setDistanceSensor(which_sensor)
        return which_sensor
        

    # Modifies the accel value of the agent/player with noise and attack
    def accel_attack(self, this_config, agent, agent_id):
        use_gaussian = int(this_config['use_gaussian_noise'])
        if use_gaussian:
            noise = np.random.normal(this_config['accel_noise_mean'], this_config['accel_noise_var'])
        else:
            noise = np.random.uniform(this_config['accel_noise_low'], this_config['accel_noise_high']) 
        
        use_attack = int(this_config['use_attack'])
        if use_attack:
            attack = np.random.normal(this_config['accel_attack_mean'], this_config['accel_attack_var'])
        else:
            attack = 0
        
        noise_accel_x = agent.acceleration.x + noise
        noise_accel_y = agent.acceleration.y + noise
        noise_accel_z = agent.acceleration.z + noise
         
        adv_accel_x = agent.acceleration.x + noise + attack
	adv_accel_y = agent.acceleration.y + noise + attack
	adv_accel_z = agent.acceleration.z + noise + attack
        
        self.scsensors[agent_id].updateAccel(noise_accel_x,
                                             noise_accel_y,
                                             noise_accel_z,
                                             adv_accel_x,
                                             adv_accel_y,
                                             adv_accel_z)

        agent.acceleration.x = adv_accel_x
	agent.acceleration.y = adv_accel_y
	agent.acceleration.z = adv_accel_z

    # Modifies the forward speed value of the agent/player with noise and attack
    def speed_attack(self, this_config, agent, agent_id):
        true_speed = agent.forward_speed
        use_gaussian = int(this_config['use_gaussian_noise'])
	if use_gaussian:
            noise = np.random.normal(this_config['speed_noise_mean'], this_config['speed_noise_var'])
        else:
            noise = np.random.uniform(this_config['speed_noise_low'], this_config['speed_noise_high']) 
        
        use_attack = int(this_config['use_attack'])
        if use_attack:
            attack = np.random.normal(this_config['speed_attack_mean'], this_config['speed_attack_var'])
        else:
            attack = 0
        
        noise_speed = agent.forward_speed + noise
        adversarial_speed = agent.forward_speed + noise + attack
        self.scsensors[agent_id].updateSpeeds(true_speed, noise_speed, adversarial_speed)
        agent.forward_speed = adversarial_speed

    def traffic_light_inject(self, agent, agent_type, player):
        this_config = self.config['trafficlight']
        which_sensor = self.distance_threshold_attack(this_config, agent.traffic_light, agent.id, player)
        if which_sensor is not None:
            self.scsensors[agent.id].agent_type = agent_type

    def speed_limit_sign_inject(self, agent, agent_type, player):
        this_config = self.config['speedlimit']
	which_sensor = self.distance_threshold_attack(this_config, agent.speed_limit_sign, agent.id, player)
        if which_sensor is not None:
            self.scsensors[agent.id].agent_type = agent_type
    
    def vehicle_inject(self, agent, agent_type, player):
        this_config = self.config['vehicle']
	which_sensor = self.distance_threshold_attack(this_config, agent.vehicle, agent.id, player)
	# Only need to launch speed attack if vehicle within sensor bounds
        if which_sensor is not None:
            self.speed_attack(this_config, agent.vehicle, agent.id)
            self.scsensors[agent.id].agent_type = agent_type
            if self.target_vehicle_id == 0:
                self.target_vehicle_id = agent.id
            else:
                self.plotter.setValues(self.scsensors[agent.id].true_distance, self.scsensors[agent.id].adversarial_distances[0])
    
    def pedestrian_inject(self, agent, agent_type, player):
        this_config = self.config['pedestrian']
	which_sensor = self.distance_threshold_attack(this_config, agent.pedestrian, agent.id, player)
	# Only need to launch speed attack if vehicle within sensor bounds
        if which_sensor is not None:
            self.speed_attack(this_config, agent.pedestrian, agent.id)
            self.scsensors[agent.id].agent_type = agent_type

    def agent_inject(self, player, agent_type, agent):

        if(agent_type == 'traffic_light'):
		self.traffic_light_inject(agent, agent_type, player)
	elif(agent_type == 'speed_limit_sign'):
		self.speed_limit_sign_inject(agent, agent_type, player)
	elif(agent_type == 'vehicle'):
		self.vehicle_inject(agent, agent_type, player)
	elif(agent_type == 'pedestrian'):
		self.pedestrian_inject(agent, agent_type, player)
	#agents ={
	#	'traffic_light' : self.traffic_light_inject(agent),
	#	'speed_limit_sign' : self.speed_limit_sign_inject(agent),
	#	'vehicle' : self.vehicle_inject(agent),
	#	'pedestrian' : self.pedestrian_inject(agent)}[agent_type](agent

	#print(agents[agent_type](agent))

    def player_inject(self, player):
	this_config = self.config['player']
	self.accel_attack(this_config, player, self.player_id)
	self.speed_attack(this_config, player, self.player_id)

    def log_measurements(self, measurements):
        #logging.info('Player speed = %f ',measurements.player_measurements.forward_speed)
        #logging.info('Player accel = %f ',measurements.player_measurements.acceleration.x)
        #logging.info('Player pitch = %f ',player_orientation.pitch)
        #logging.info('Player yaw = %f ',player_orientation.yaw)
        #logging.info('Player roll = %f ',player_orientation.roll)
        
        # Logging for agents 
        for i,agent_id in enumerate(self.scsensors.keys()):
            #self.agent_num = self.agent_num +1
            #logging.info('agent_ID: {}'.format(self.agent_num))
            #logging.info('vehicle forward speed = %f ', a.vehicle.forward_speed)
            #logging.info('vehicle x = %f ', a.vehicle.transform.location.x)
            #logging.info('vehicle y = %f ', a.vehicle.transform.location.y)
            #logging.info('vehicle z = %f ', a.vehicle.transform.location.z)
            # Only print distance values if attack already launched and new distances have
            # already been stored in true_distances and adversarial_distances
            if agent_id == self.player_id:
                continue
            self.log_measurements_csv(agent_id, self.scsensors[agent_id])
            
            #logging.info('true distance to agent: %f', self.true_distances[agent_id])
            #logging.info('false distance to agent: %f', self.adversarial_distances[agent_id][0])
            

    def log_measurements_csv(self, agent_id, scsensor):

	#now = datetime.datetime.now()
	#if(now.second != self.output_time.second):
	#self._dict_distances['datetime'] = now
        
        # Only report distances for agents not player
        if scsensor.agent_type is not 'player':
            src_node = self.step%int(self.config['all']['num_sensors'])
            self._dict_distances['src_node'] = src_node
            self._dict_distances['dest_node'] = agent_id
            self._dict_distances['sensor'] = scsensor.detected_by_sensor
            self._dict_distances['noise_distance'] = scsensor.noise_distances[src_node]
            self._dict_distances['adversarial_distance'] = scsensor.adversarial_distances[src_node]
            self._dict_distances['true_distance'] = scsensor.true_distance
        
        # Parameters that can be reported for both agents and player
	self._dict_distances['step'] = self.step
        self._dict_distances['agent_type'] = scsensor.agent_type
        self._dict_distances['noise_speed'] = scsensor.noise_speed
        self._dict_distances['adversarial_speed'] = scsensor.adversarial_speed
        self._dict_distances['agent_type'] = scsensor.agent_type
        self._dict_distances['true_speed'] = scsensor.true_speed
        
        self._dict_distances['true_accel_x'] = scsensor.true_accel_x
        self._dict_distances['true_accel_y'] = scsensor.true_accel_y
        self._dict_distances['true_accel_z'] = scsensor.true_accel_z
        self._dict_distances['adv_accel_x'] = scsensor.adv_accel_x
        self._dict_distances['adv_accel_y'] = scsensor.adv_accel_y
        self._dict_distances['adv_accel_z'] = scsensor.adv_accel_z

        self._dict_distances['pitch'] = scsensor.pitch
        self._dict_distances['yaw'] = scsensor.yaw
        self._dict_distances['roll'] = scsensor.roll

       
	#for s in range(0,int(self.config['all']['num_sensors'])):
	#	self._dict_distances['sensor{}'.format(s)] = adversarial_distance[s]
	
	with open(self.csv_file, 'a+') as rfd:
		w = csv.DictWriter(rfd, self._dict_distances.keys())
		w.writerow(self._dict_distances)
	#self.output_time = now
    
        return

    def delay_attack(self, curr_frame, nframes, measurements, sensor_data):
        # Don't launch attack until nframes (set in config file) reached
        if curr_frame < nframes:
            self.wait_counter = curr_frame + 1
            self.meas_fifo.put(measurements)
            self.sensor_fifo.put(sensor_data)
            delayed_measurements = measurements
            delayed_sensor_data = sensor_data
        else: # Pop a value and push on latest value
            delayed_measurements = self.meas_fifo.get()
            delayed_sensor_data = self.sensor_fifo.get()
            self.meas_fifo.put(measurements)
            self.sensor_fifo.put(sensor_data)
        return delayed_measurements, delayed_sensor_data

    def frame_swap_attack(self, curr_frame, nframes, measurements, sensor_data):
        # Don't execute frame swap until nframes (set in config file) reached
        if curr_frame < nframes:
            self.wait_counter = curr_frame + 1
            self.meas_buf.append(measurements)
            self.sensor_buf.append(sensor_data)
            swapped_measurements = measurements
            swapped_sensor_data = sensor_data
        else: 
            # Randomly select an index of the buffer to return
            # Populate that index with the current measurement
            index = np.random.random_integers(0,nframes-1)
	    print "index:{}".format(index)
            swapped_measurements = self.meas_buf[index]
            swapped_sensor_data = self.sensor_buf[index]
            self.meas_buf[index] = measurements
            self.sensor_buf[index] = sensor_data
        return swapped_measurements, swapped_sensor_data

    def inject_adversarial(self, measurements, sensor_data):

        #Refresh data structures
        self.clearDict()
        self.scsensors = {}
        self.scsensors[self.player_id] = SCSensor()
        self.scsensors[self.player_id].agent_type = 'player'
        
        #Delay attack modifies measurements and sensor_data to values from a previous frame
        if self.config['time']['delay_attack'] == 1:
            measurements, sensor_data = self.delay_attack(self.wait_counter, self.config['time']['nframes'], measurements, sensor_data) 
        #Frame swap attack swaps measurements and sensor_data of adjacent frames
        elif self.config['time']['frame_swap_attack'] == 1:
            measurements, sensor_data = self.frame_swap_attack(self.wait_counter, self.config['time']['nframes'], measurements, sensor_data)
        
        #Inject noise into the player measurements
	self.player_inject(measurements.player_measurements)
        player_orientation = measurements.player_measurements.transform.rotation
        self.scsensors[self.player_id].updateOrientation(player_orientation.pitch,
                                                    player_orientation.yaw,
                                                    player_orientation.roll)
        self.log_measurements_csv(self.player_id, self.scsensors[self.player_id])

        #Inject noise into agent measurements
        for a in measurements.non_player_agents:
            self.agent_inject(measurements.player_measurements, a.WhichOneof('agent'), a)
        
        logging.info("Adversarial Measurement Values:")
        self.log_measurements(measurements)

#	sensor_data['CameraRGB'].raw_data = camera_attack.perform_attack(sensor_data['CameraRGB'])
	
	#self.wait_counter = self.wait_counter + 1
	#print self.wait_counter
	if self.wait_counter >= 80:
		sensor_data['CameraRGB'].save_to_disk("/home/carla/Documents/carla_outputs/camera_outputs/pedestrian.png")
		print("Done")
		time.sleep(5)
	
	self.step = self.step + 1
        return measurements, sensor_data
	

