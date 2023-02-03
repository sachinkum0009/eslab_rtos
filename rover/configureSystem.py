#configureSystem.py

#DATE: 27-09-2020

import os


#*** CONFIGURATIONS
#-----------------------------------------------------------------

class configureCamera:


	camera_path ='/home/asus/Fh_Dortmund/ESE/semester_1/ESE/Lab/milestone3/rover/exampleCamera.yml'
	out_path ='/home/asus/Fh_Dortmund/ESE/semester_1/ESE/Lab/milestone3/rover/image_test.jpg'
	# camera_width=640 #1280x720 or 640x360
	# camera_height=360 #720 
	camera_width=1920 #1280x720 or 640x360
	camera_height=1080 #720 
	def __init__(self, camera_resource=0):
		self.__camera_resource = camera_resource
		
		
		
	def get_resource(self):
		return self.__camera_resource
	def set_resource(self, camera_resource):
		self.__camera_resource = camera_resource

	def get_camera_path(self):
		return self.camera_path
	def set_camera_path(self, camera_path):
		self.camera_path = str(camera_path)

	def get_out_path(self):
		return self.out_path
	def set_out_path(self, out_path):
		self.out_path = str(out_path)

	def get_camera_width(self):
		return self.camera_width
	def set_camera_width(self, camera_width):
		self.__camera_width = camera_width

	def get_camera_height(self):
		return self.camera_height
	def set_camera_height(self, camera_height):
		self.__camera_height = camera_height



		
class configureMarkers:	
	marker_size= 0.095 	#The size in meters of the marker. constant DON'T  CHANGE!
	resize_ratio=1 		#Leave this value as default


	def __init__(self):
		pass
		
	def get_resize_ratio(self):
		return self.resize_ratio
	def set_resize_ratio(self, resize_ratio):
		self.resize_ratio = resize_ratio		
				
	def get_marker_size(self):
		return self.marker_size
	def set_marker_size(self, marker_size):
		self.marker_size = marker_size	

	
#   | ID  |	----------L1---------| ID  |
#   |  1  |						        |  2  |
#	   |							            |
#	   |-->L4					        	| -->L2
#	   |						            	|
#	   |						          	    |
#   | ID |						        | ID |
#   |  4  |	----------L3-------- |  3  |				
class configureScenario:			
	border_marker_ids= (1,2,3,4)	 	#The IDs of the markers located in the borders
	camera_image_size=1080 #max allowed so the system can recognie the markers. constant DON'T  CHANGE!
	virtual_scenario_image_size=480 #the size of the loaded scenario. constant DON'T  CHANGE!
	pixels_target_tolerance=100  #Program constant DON'T  CHANGE!
	def __init__(self):
		pass

	def get_border_ids(self):
		return self.border_marker_ids	
		
	def get_scenario_image_size(self):
		return self.camera_image_size
	def set_scenario_image_size(self, camera_image_size):
		self.camera_image_size = camera_image_size	

	def get_virtual_scenario_image_size(self):
		return self.virtual_scenario_image_size
	def set_virtual_scenario_image_size(self, virtual_scenario_image_size):
		self.virtual_scenario_image_size = virtual_scenario_image_size
		
	def get_pixels_target_tolerance(self):
		return self.pixels_target_tolerance
	def set_pixels_target_tolerance(self, pixels_target_tolerance):
		self.pixels_target_tolerance = pixels_target_tolerance			
		
class configureAWS:
	host = 'a6y3ms0k7agqq-ats.iot.us-east-1.amazonaws.com' # set here your end-point
	rootCAPath = 'AmazonRootCA1.pem'
	certificatePath = '327feece57089ce207681de3c2d7ee4f87697aff8b52b0ea8fc0d974296f135b-certificate.pem.crt'  #update to the name of your certificate
	privateKeyPath = '327feece57089ce207681de3c2d7ee4f87697aff8b52b0ea8fc0d974296f135b-private.pem.key'  #update to the name of your certificate
	clientId =  ' PublisherG10'  # set your thing name
	topic_rover = 'esp32/rover'
	topic_target = 'esp32/target'
	topic_cmd_vel = 'esp32/cmd_vel'
	#port=8883
	useWebsocket=False


					
	def __init__(self):
		pass

	def get_aws_host(self):
		return self.host
	def set_aws_host(self, aws_host):
		self.host = aws_host	
		
	def get_root_file(self):
		return self.rootCAPath
	def set_root_file(self, root_file_path):
		self.rootCAPath = root_file_path
		
	def get_cert_file(self):
		return self.certificatePath
	def set_cert_file(self, cert_file_path):
		self.certificatePath = cert_file_path

	def get_priv_file(self):
		return self.privateKeyPath
	def set_priv_file(self, priv_file_path):
		self.privateKeyPath = priv_file_path		
		
	def get_thing_name(self):
		return self.clientId
	def set_thing_name(self, thing_name):
		self.clientId = thing_name

	def get_topic(self, keyword):
		if keyword=="rover":
			return self.topic_rover
		elif keyword=="target":
			return self.topic_target
		elif keyword=="cmd_vel":
			return self.topic_cmd_vel
		else:
			return 'default/topic'
	def set_topic(self, keyword):
		if keyword=="rover":
			self.topic_rover = keyword
		elif keyword=="target":
			self.topic_target= keyword
		elif keyword=="cmd_vel":
			self.topic_cmd_vel=keyword
		





		
if __name__ == '__main__':
	myCamera=configureCamera()
	myMarker=configureMarkers()	
	
	print (myCamera.get_resource())
	myCamera.set_resource(0)
	print ("camera Resource", myCamera.get_resource())
	print ("camera file path", myCamera.get_camera_path())
	print ("Setting new path")
	myCamera.set_camera_path("/test/des/")
	print ("camera file path",myCamera.get_camera_path())

	print ("Marker size:", myMarker.get_marker_size())
	print ("Setting marker value")
	myMarker.set_marker_size(8)
	print ("Marker size:",myMarker.get_marker_size())








