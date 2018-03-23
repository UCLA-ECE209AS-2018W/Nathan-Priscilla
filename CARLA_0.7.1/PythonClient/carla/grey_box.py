

"""Grey box """

import numpy as np
import copy

class GreyBox(object):
    def __init__(self):

        return
    
    def final_scene_attack(final_scene):

        return

    def depth_map_attack(depth_map):

        return

    def semantic_segmentation_attack(semantic_segmentation):

        return

    def lidar_attack(lidar):

        return

    def measurement_attack(measurements):

        return 
    
    def perform_attack(self, measurements, sensor_data):
        attack_measurements = copy.deepcopy(measurements)
        attack_sensor_data = copy.deepcopy(sensor_data)

        final_scene_attack(attack_sensor_data['CameraRGB'])
        depth_map_attack(attack_sense_data['CameraDepth'])
        semantic_segmentation_attack(attack_sensor_data['CameraSemSeg'])
        lidar_attack(attack_sensor_data['lidar'])
        measurement_attack(attack_measurements)
        
        write_to_visualizer(measurements, sensor_data, attack_measurements, attack_sensor_data)

        return measurements, sensor_data
