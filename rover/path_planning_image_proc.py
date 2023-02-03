#path_planning_image_proc.py

# Copyright 2023 Sachin Kumar - sachin.kumar003@stud.fh-dortmund.de
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.



#Last Update: 31-01-2023

#Description: Executes the system to get the imiage from the camera, detect the borders
#rectify the ROI (area within the borders), read the scenario image and overlay the 
#obstacles and targets defined there in the real scenario (as a graph).



#Note: This script requires as input the image that describes the scenario. 
#You have to design an image describing the 
#position of the obstacles and the targets, which will be mapped to the image gotten from the camera. 
#The image must be created based on the provided template.
#In the image scenario, rectangles will be interpreted as obstacles and circles as targets and triangles as starting point. 

#ARGUMENTS:
#
# -i / --image  --> the scenario image to upload
# -id           --> the id of the marker on your crawler
# -c / --aws    --> if the application connects or not to aws [True/False] default:False
# -m / --mode   --> the game mode ["bottom"/"top"/"mixed"] default:bottom

#Note: you can exit the program pressing "q" when a window is active.

import numpy as np
import cv2
import time 
import argparse
import transform as tr
import get_xaxis_image_points_opt as marker
import os
import drawScenario
import shapedetector
import configureSystem
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import json
from my_rrt_algo import RRT, Dijkstra
from enum import Enum

#*** SYSTEM CONFIGURATION
#-----------------------------------------------------------------
myScenarioConfig=configureSystem.configureScenario() #use this object to apply the configurations 
                                                     #of the system in the configureSystem.py 


class Vertice(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Edge(object):
    def __init__(self, x1, y1, x2, y2):
        self.start_vertice = Vertice(x1, y1)
        self.stop_vertice = Vertice(x2, y2)



class PathPlanning(object):
    '''
    pseudocode
    RRT Pseudo Code
    Qgoal //region that identifies success
    Counter = 0 //keeps track of iterations
    lim = n //number of iterations algorithm should run for
    G(V,E) //Graph containing edges and vertices, initialized as empty
    While counter < lim:
        Xnew  = RandomPosition()
        if IsInObstacle(Xnew) == True:
            continue
        Xnearest = Nearest(G(V,E),Xnew) //find nearest vertex
        Link = Chain(Xnew,Xnearest)
        G.append(Link)
        if Xnew in Qgoal:
            Return G
    Return G
    '''
    def __init__(self, robot_position: list, target_position: list, obstacles: list):
        self.start_pos = Vertice(robot_position[0], robot_position[1])
        self.goal_pos = Vertice(target_position[0], target_position[1])
        self.obstacles_list = list()
        for obstacle in obstacles:
            self.obstacles_list.append(Vertice(obstacle[0], obstacle[1]))
        
        pass

    def find_trajectory(self):
        pass




class AWSCommunication(object):
    def __init__(self):
        myAWSConfig=configureSystem.configureAWS()													 
        self.host = myAWSConfig.get_aws_host()
        self.rootCAPath = myAWSConfig.get_root_file()
        self.certificatePath = myAWSConfig.get_cert_file()
        self.privateKeyPath = myAWSConfig.get_priv_file()
        self.clientId = myAWSConfig.get_thing_name()
        self.topic_rover = myAWSConfig.get_topic("rover")
        self.topic_targer = myAWSConfig.get_topic("target")
        self.topic_cmd_vel = myAWSConfig.get_topic("cmd_vel")
        self.useWebsocket=myAWSConfig.useWebsocket

    def connect_aws(self):
        if self.useWebsocket and self.certificatePath and self.privateKeyPath:
            print("X.509 cert authentication and WebSocket are mutual exclusive. Please pick one.")
            exit(2)

        if not self.useWebsocket and (not self.certificatePath or not self.privateKeyPath):
            print("Missing credentials for authentication.")
            exit(2)

    # Port defaults
        if self.useWebsocket:  # When no port override for WebSocket, default to 443
            port = 443
        if not self.useWebsocket:  # When no port override for non-WebSocket, default to 8883
            port = 8883	
        
        
        self.myAWSIoTMQTTClient = None
        if self.useWebsocket:
            self.myAWSIoTMQTTClient = AWSIoTMQTTClient(self.clientId, useWebsocket=True)
            self.myAWSIoTMQTTClient.configureEndpoint(self.host, port)
            self.myAWSIoTMQTTClient.configureCredentials(self.rootCAPath)
        else:
            self.myAWSIoTMQTTClient = AWSIoTMQTTClient(self.clientId)
            self.myAWSIoTMQTTClient.configureEndpoint(self.host, port)
            self.myAWSIoTMQTTClient.configureCredentials(self.rootCAPath, self.privateKeyPath, self.certificatePath)

        # AWSIoTMQTTClient connection configuration
        self.myAWSIoTMQTTClient.configureAutoReconnectBackoffTime(1, 32, 20)
        self.myAWSIoTMQTTClient.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
        self.myAWSIoTMQTTClient.configureDrainingFrequency(2)  # Draining: 2 Hz
        self.myAWSIoTMQTTClient.configureConnectDisconnectTimeout(10)  # 10 sec
        self.myAWSIoTMQTTClient.configureMQTTOperationTimeout(5)  # 5 sec

    # Connect and subscribe to AWS IoT
        self.myAWSIoTMQTTClient.connect()
        return self.myAWSIoTMQTTClient
    
    def send_cmd_vel(self, linear: float, angular: float):
        current_cmd_vel_coordinate = {"linear": str(int(linear*0.053475936)), "angular": str(angular)}
        # mes_cmd_vel = json.dumps({'cmd_vel': '{}'.format(current_cmd_vel_coordinate)})		
        mes_cmd_vel = json.dumps(current_cmd_vel_coordinate)
        self.myAWSIoTMQTTClient.publish(self.topic_cmd_vel, mes_cmd_vel, 1)	
        print("publishing the data")
    
    def wait_for_response(self, topic_name):
        self.myAWSIoTMQTTClient.subscribe(topic_name, 1, self.customCallback)
        print("aws subscriber connected")

    def customCallback(self, client, userdata, message):
        print("Received a new message: ")
        print(message.payload)
        print("from topic: ")
        print(message.topic)
        print("--------------\n\n")    

class State(Enum):
    IDLE = 1
    MOVE = 2
    

class CameraController(object):
    def __init__(self, image_scenario, connect_to_aws, rover_id, game_mode):
        self.first_time = False
        self.aws_object = AWSCommunication()
        self.rover_id = rover_id
        self.connect_to_aws = connect_to_aws
        self.current_state = State.IDLE
        self.rover_pose = []

        self.path_points = []
        

        sd = shapedetector.ShapeDetector() #creates an object to detect the forms existent in the image scenario	
                
        #Init variables:

        image_scenario, self.obstacles_scenario, self.targets_scenario, self.start_point=sd.get_shapes_coordinates(image_scenario)
        cv2.imshow("Image_scenario", image_scenario)
        self.size_image_virtual_scenario=tuple(image_scenario.shape[1::-1])
        print("image_virtual_scenario_size: ",self.size_image_virtual_scenario)	
        print ("detected obstacles in scenario: ", self.obstacles_scenario)
        print ("detected targets in scenario: ", self.targets_scenario)
        print ("rover starts in: ", self.start_point)	

        #::::::::::::::::::::::check aws connection::::::::::::::::
        if connect_to_aws:
            self.myAWSIoTMQTTClient=self.aws_object.connect_aws()
            
        myCameraConfig=configureSystem.configureCamera(2) #Change to "1" to get the USB camera in configure.System.py

        self.cap = cv2.VideoCapture(myCameraConfig.get_resource()) #select resource 1 (webcam) --> 0 integrated cam in laptop
        # self.cap = cv2.VideoCapture("video.webp") #select resource 1 (webcam) --> 0 integrated cam in laptop
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, myCameraConfig.get_camera_width()) #this value could change if using another webcam
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, myCameraConfig.get_camera_height()) #this value could change if using another webcam
        time.sleep(1) #wait until webcam is ready

        self.IMAGE_MAX_WH = myScenarioConfig.get_scenario_image_size()#declare the maximum pixels admited (<=1080)
        resize_ratio = 1	#initializes the variable

        self.myBorders=drawScenario.Borders() #creates an object of the class Borders
        self.myObstacles=drawScenario.Obstacles() #creates an object of the class Obstacles
        self.myTargets= drawScenario.Targets() #creates an object of the class Targets
        self.myStart= drawScenario.StartPoint() #creates an object of the class StartPoint	
        self.border_list_default=[(20,10),(120,10),(120,120),(20,120)]	#default value for the border coordinates (just in case)
        current_target_coordinate=None #Init the target coordinate
        
        self.myTargets.set_target_list(self.targets_scenario,game_mode) #formats the target list to determine which one will be shown
        self.border_list=[] #init the variable where we will put the points of the border markers

    def get_distance(self, point_a, point_b):
        return np.sqrt(pow((point_a[0]-point_b[0]),2)+pow((point_a[1]-point_b[1]),2))

    def get_angle(self, point_a, point_b):
        print("point a", point_a)
        print("point b", point_b)
        print(np.arctan2(point_a[0]-point_b[0], point_a[1]-point_b[1]))
        return int(np.rad2deg(np.arctan2(point_a[0]-point_b[0], point_a[1]-point_b[1])))
                                              
    def capture_image(self):
        ret = False
        while not ret:
            ret, img = self.cap.read() #get the frame from the camera
            print("ret", ret)
            
        
        # cv2.imshow("imag", img)
        # cv2.waitKey(1)
        resize_ratio=1
        #if the received image from the camera is bigger, then resize it
        print(img.shape)
        print("max", max(img.shape))
        if max(img.shape) > self.IMAGE_MAX_WH:
            resize_ratio = float(self.IMAGE_MAX_WH) / max(img.shape[0], img.shape[1])
            img = cv2.resize(img, (0, 0),fx=resize_ratio,fy=resize_ratio,interpolation=cv2.INTER_AREA)
        return img, resize_ratio
    

    def local_point_path(self, point: tuple, target_point: tuple):
        '''
        sends the local point path to the aws
        '''
        print("point", point)
        angle = self.get_angle(point, target_point)
        # angle = self.get_angle(point, self.rover_pose[5][0])
        distance = self.get_distance(point, target_point)
        self.aws_object.send_cmd_vel(int(distance), int(angle-self.current_angle))
        self.current_angle = angle
        print("angle", angle)
        print("absolute angle", self.rover_pose[5][1])
        print("distance", distance)
        # input("enter")
        

    def find_path(self, start_pos: tuple, end_pos: tuple, obstacles: list, radius, n_iter, step_size):
        rrt_obj = RRT(start_pos, end_pos, n_iter, radius, step_size, obstacles)
        rrt_graph = rrt_obj.run()
        if rrt_graph.success:
            trajectory = Dijkstra(rrt_graph).run()
            self.path_points = trajectory
            previous_point = self.rover_pose[5][0]
            i = 0

            # if not self.first_time:
            #     self.aws_object.send_cmd_vel(0.0, 90)
            #     self.first_time = True
            
            # input("enter")
            # self.current_angle = 0.0
            # self.current_angle = self.rover_pose[5][1]
            self.current_angle = 270 - self.rover_pose[5][1]
            for path in trajectory:
                self.local_point_path(path, previous_point)
                previous_point = path
                i = i+1
                # input("enter")
                time.sleep(5)
        else:
            print("not success")
        

        
        # self.current_state = State.MOVE
    
    def update_rover_pos(self):
        img, _ = self.capture_image() #gets a clean capture of the scene (without border markers recognition)
        warped = tr.four_point_transform(img, self.pts) #cuts the ROI (region of interest) delimited by the
                                                    #four markers and rectifies it.
        
        size_image_real_scenario=tuple(warped.shape[1::-1]) #gets the size in pixels of the rectified ROI
        cv2.imshow("warp", warped)
            
        #Resizes the rectified ROI to form a square equal to the scenario image
        resize_ratio_x=self.size_image_virtual_scenario[0]/size_image_real_scenario[0] 
        resize_ratio_y=self.size_image_virtual_scenario[1]/size_image_real_scenario[1] 
        warped = cv2.resize(warped, (0, 0),fx=resize_ratio_x,fy=resize_ratio_y,interpolation=cv2.INTER_AREA)
                
        my_rover_coordinates=(marker.get_markers_info(warped)) #detects the markers in the rectified ROI image 
        self.rover_pose = my_rover_coordinates
        
    def test_forward_pixel(self):
        self.update_rover_pos()
        self.previous_pos = self.rover_pose[5][0]
        
        self.aws_object.send_cmd_vel(80, 0)
        time.sleep(10)
        self.update_rover_pos()
        self.new_pos = self.rover_pose[5][0]
        print('previous pose', self.previous_pos)
        print('new pose', self.new_pos)
        


    def find_border_markers(self):
        # iterate until we find all the border markers
        while len(self.border_list)!= len(myScenarioConfig.get_border_ids()):
            self.border_list.clear() #initializes the variable where we will put the points of the border markers
            img, resize_ratio=self.capture_image() #gets a frame from the camera
            #size_image_real_scenario=tuple(img.shape[1::-1]) #comment
            #print("image_real_scenario_size: ",size_image_real_scenario) #comment
            my_markers=(marker.get_markers_info(img)) #get markers info (coordinates and angles)			
            #cv2.imshow("Original", img) #shows the original image with the detected markers as a reference
            if my_markers:
                self.myBorders.draw_borders(img, my_markers, False) #draws the borders 
                                                                #-->False means we dont use real time border recognition
                borders_coordinates=self.myBorders.get_borders() #returns the coordinates of the border markers
                #print (borders_coordinates)
                
                for coordinate_id in borders_coordinates:
                    self.border_list.append(borders_coordinates[coordinate_id]) #appends only the points into a list to be sent to the 
                # cv2.imshow('img', img)
                # cv2.waitKey(0)
                pts = np.array(self.border_list_default, dtype = "float32") #just in case
                print ("not all borders detected:",my_markers)
                
            time.sleep(0.2)
        
        self.pts = np.array(self.border_list, dtype = "float32") #creates an array with the border points gotten before

    def find_rover_target_pos(self):
        #once we detected the working area, display the scenario and start game	
        continue_game=True
        # while(True):
        if True:
            #rectification method
            img, _ = self.capture_image() #gets a clean capture of the scene (without border markers recognition)
            warped = tr.four_point_transform(img, self.pts) #cuts the ROI (region of interest) delimited by the
                                                        #four markers and rectifies it.
            
            size_image_real_scenario=tuple(warped.shape[1::-1]) #gets the size in pixels of the rectified ROI
            cv2.imshow("warp", warped)
                
            #Resizes the rectified ROI to form a square equal to the scenario image
            resize_ratio_x=self.size_image_virtual_scenario[0]/size_image_real_scenario[0] 
            resize_ratio_y=self.size_image_virtual_scenario[1]/size_image_real_scenario[1] 
            warped = cv2.resize(warped, (0, 0),fx=resize_ratio_x,fy=resize_ratio_y,interpolation=cv2.INTER_AREA)
                    
            my_rover_coordinates=(marker.get_markers_info(warped)) #detects the markers in the rectified ROI image 
            self.myStart.draw_start(warped,self.start_point[0])	#draw the starting point for the rover	
            print("obstacle ", self.obstacles_scenario)
            #iterates through the coordinates of the recognized obstacles in the image scenario
            for obstacle_coord in self.obstacles_scenario:
                warped=self.myObstacles.draw_obstacles(warped, obstacle_coord) #draws the obstacle from the image scenario
                                                                                #in its equivalent position in the rectified ROI 																
            if self.targets_scenario:
                warped,target_list, current_target_coordinate = self.myTargets.draw_current_target(warped) #draws the obstacle from the image scenario
                self.rover_pose = my_rover_coordinates
                print("rover pose", my_rover_coordinates)
                print("target pose", current_target_coordinate)
                start_pos = my_rover_coordinates
                goal_pos = (current_target_coordinate)
                self.find_path(start_pos[5][0], goal_pos, self.obstacles_scenario, 50, 200, 50)
                # Radius of circle
                radius = 10
                    
                # Red color in BGR
                color = (0, 0, 255)
                    
                # Line thickness of -1 px
                thickness = -1
                for point in self.path_points:
                    print('point', point)
                    cv2.circle(warped, (int(point[0]),int(point[1])), radius, color, thickness)
                cv2.imshow("img", warped)
                cv2.waitKey(0)
                # input("stop")
                img, _ = self.capture_image() #gets a clean capture of the scene (without border markers recognition)
                warped = tr.four_point_transform(img, self.pts) #cuts the ROI (region of interest) delimited by the
                                                            #four markers and rectifies it.
                
                size_image_real_scenario=tuple(warped.shape[1::-1]) #gets the size in pixels of the rectified ROI
                cv2.imshow("warp", warped)
                    
                #Resizes the rectified ROI to form a square equal to the scenario image
                resize_ratio_x=self.size_image_virtual_scenario[0]/size_image_real_scenario[0] 
                resize_ratio_y=self.size_image_virtual_scenario[1]/size_image_real_scenario[1] 
                warped = cv2.resize(warped, (0, 0),fx=resize_ratio_x,fy=resize_ratio_y,interpolation=cv2.INTER_AREA)
                        
                my_rover_coordinates=(marker.get_markers_info(warped)) #detects the markers in the rectified ROI image 
                
                if my_rover_coordinates:
                    try:
                        continue_game = self.myTargets.check_caught(target_list, my_rover_coordinates[int(self.rover_id)][0]) 
                        print("target coord", current_target_coordinate)
                        print("robot coord", my_rover_coordinates)
                    except:
                        print ("no rover_marker in image")														
                            
                    print("Target:", current_target_coordinate," my rover: ", my_rover_coordinates) #prints {"markerID":[(x coord,y coord), angle]} of the rover in the rectified ROI image

            cv2.imshow("Rectified", warped) #uncomment
            print("rover", my_rover_coordinates)
            print("target", current_target_coordinate)
            if self.connect_to_aws:
                print("aws published")
                # mes_rover = json.dumps({'rover': '{}'.format(my_rover_coordinates)})		
                # self.myAWSIoTMQTTClient.publish(self.aws_object.topic_rover, mes_rover, 1)	
                # mes_target = json.dumps({'target': '{}'.format(current_target_coordinate)})		
                # self.myAWSIoTMQTTClient.publish(self.aws_object.topic_targer, mes_target, 1)
                #print("publishing")
            
            end=time.time()
            #print (end-start )
            time.sleep(0.03)
            
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
            if not continue_game:
                print ("All targets caught...")
                time.sleep (2)
                # break
                exit(1)
        cv2.destroyAllWindows()

    def run(self):
        self.find_border_markers()
        while True:
            if self.current_state == State.IDLE:
                self.find_rover_target_pos()
                # self.test_forward_pixel()
                # input("enter")
        

        self.cap.release()
        
        
def main():
    #Requires the image of the scenario as input
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", required=True,
        help="path to the input image")
    ap.add_argument("-c", "--aws", default=False,
        help="connect to aws?")
    ap.add_argument("-id", required=True,
        help="rover id")		
    ap.add_argument("-m", "--mode", default="bottom",
        help="game mode: bottom, top, mixed")		
        
        
    args = vars(ap.parse_args())
    image_scenario = cv2.imread(args["image"])
    connect_to_aws = args["aws"]
    rover_id = args["id"]
    game_mode = args["mode"]
    camera_controller_object = CameraController(image_scenario,connect_to_aws, rover_id, game_mode)
    camera_controller_object.run()
    
if __name__ == "__main__":
    main()
    