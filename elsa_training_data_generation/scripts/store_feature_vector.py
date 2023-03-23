#!/usr/bin/env python3

'''
Compute state difference between consecutive messages
'''

import math
import numpy as np
import sklearn as skl
#import neupy

import rospy
#from elsa_msgs.msg import RichState
from elsa_perception_msgs.msg import *
from std_msgs.msg import String, Bool

class ProcessNode:
    def __init__(self):
        self.state_sub = rospy.Subscriber("scene_description", PhysicalScene, self.state_cb)
        
        self.feature_vec_history = []

        self.current_feature_vec = {}

        self.state_received = False              

    def state_cb(self, msg):
        print("---------------")
        print("State received!")
        # if current & previous are None => current = msg
        # if current is set and previous is None =>  previous = current ; current = msg ; compute effect
        # if current and previous are set => previous = current ; current = msg
        print(len(msg.physical_scene))

        # update current with new
        seen_objects = []
        for obj in msg.physical_scene:
            self.current_feature_vec[obj.obj_identity] = obj
            seen_objects.append(obj.obj_identity)

        for k, obj in self.current_feature_vec.items():
            print(k, obj.obj_identity)

        if not self.current_feature_vec:
            if not self.previous_feature_vec:
                print("Both are None")
            else:
                print("Current is None, Previous is set")
        else:
            if not self.previous_feature_vec:
                print("Previous is None, Current is set")
            else:
                # update received data
                print("Updating effects")

                #for k, v in self.current_state.items():
                for k in seen_objects:
                    if k in self.previous_feature_vec.keys():
                        array_effect = self.create_feature_array(self.previous_state[k])
                    else:
                        print("New object in fov : \033[33m", k, "\033[39m")

                for k, v in self.previous_feature_vec.items():
                    #if k not in self.current_state.keys():
                    if k not in seen_objects:
                        print("\033[31m", k,"\033[39m was removed from the scene")


    # Compute difference between current and previous ; sx assumed to be a PhysicalFeatures msg
    def create_feature_array(self, previous):
        feature_vector = []
        feature_vector.append(previous.spatial_features.x)
        feature_vector.append(previous.spatial_features.y)
        feature_vector.append(previous.spatial_features.z)
        feature_vector.append(previous.spatial_features.phi)
        feature_vector.append(previous.spatial_features.dx)
        feature_vector.append(previous.spatial_features.dy)
        feature_vector.append(previous.spatial_features.dz)
        feature_vector.append(previous.surface_features.normal_azimut.min)
        feature_vector.append(previous.surface_features.normal_azimut.max)
        feature_vector.append(previous.surface_features.normal_azimut.avg)
        feature_vector.append(previous.surface_features.normal_azimut.var)
        feature_vector.append(previous.surface_features.normal_azimut.dev)
        feature_vector = feature_vector + previous.surface_features.normal_azimut.his
        feature_vector.append(previous.surface_features.normal_zenith.min)
        feature_vector.append(previous.surface_features.normal_zenith.max)
        feature_vector.append(previous.surface_features.normal_zenith.avg)
        feature_vector.append(previous.surface_features.normal_zenith.var)
        feature_vector.append(previous.surface_features.normal_zenith.dev)
        feature_vector = feature_vector + previous.surface_features.normal_zenith.his
        feature_vector.append(previous.surface_features.min_curvature.min)
        feature_vector.append(previous.surface_features.min_curvature.max)
        feature_vector.append(previous.surface_features.min_curvature.avg)
        feature_vector.append(previous.surface_features.min_curvature.var)
        feature_vector.append(previous.surface_features.min_curvature.dev)
        feature_vector = feature_vector + previous.surface_features.min_curvature.his
        feature_vector.append(previous.surface_features.max_curvature.min)
        feature_vector.append(previous.surface_features.max_curvature.max)
        feature_vector.append(previous.surface_features.max_curvature.avg)
        feature_vector.append(previous.surface_features.max_curvature.var)
        feature_vector.append(previous.surface_features.max_curvature.dev)
        feature_vector = feature_vector + previous.surface_features.max_curvature.his
        feature_vector.append(previous.surface_features.shape_index.min)
        feature_vector.append(previous.surface_features.shape_index.max)
        feature_vector.append(previous.surface_features.shape_index.avg)
        feature_vector.append(previous.surface_features.shape_index.var)
        feature_vector.append(previous.surface_features.shape_index.dev)
        feature_vector = feature_vector + previous.surface_features.shape_index.his
        
        return np.array(feature_vector)

    def monitor_cb(self, msg):
        if msg.data:
            print("Current effect history")
            for e in self.effect_history:
                print(e)

    def run(self):
        print("Running")
        pass

    def effect_categorization(self):
        # using effect list, use (R)GNG to find a good clustering
        print(len(self.effect_history))

        pass

'''
    # trigger the loop when called
    def trigger_cb(self, msg):
        print("Trigger Callback")
        try:
            self.run()
        except KeyboardInterrupt:
            print("Caught user STOP signal, returning.")
        print("runned!")

    def periodic_cb(self,event):
        #print("top")
        pass
'''



if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="data_collection_node")
    parser.add_argument('--location', help="Where to store the generated dataset")
    args = parser.parse_args()
    
    rospy.init_node("elsa_effect_node")

    pn = ProcessNode()
    print("Process node running ...")
    rospy.spin()

    pn.run()
