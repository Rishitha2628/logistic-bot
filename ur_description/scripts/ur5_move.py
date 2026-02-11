#!/usr/bin/env python3



import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
from ur_description.msg import BoxOrientation, BoxTransformation, Eefpos
from std_msgs.msg import Bool



##################### FUNCTION DEFINITIONS #######################

def distance_calc(points):
    distance = math.sqrt((points[0][0]-points[1][0])**2 + (points[0][1]-points[1][1])**2)

    return distance 


def calculate_rectangle_area(coordinates):
    w = distance_calc([coordinates[0],coordinates[1]])
    h = distance_calc([coordinates[1],coordinates[2]])
    area = h*w

    return area


def quaternion_to_euler(quaternion):

    w, x, y, z = quaternion

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw



def euler_to_rotation_matrix(roll, pitch, yaw):
    
    R_roll = np.array([[1, 0, 0],
                       [0, math.cos(roll), -math.sin(roll)],
                       [0, math.sin(roll), math.cos(roll)]])

    R_pitch = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                        [0, 1, 0],
                        [-math.sin(pitch), 0, math.cos(pitch)]])

    R_yaw = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                      [math.sin(yaw), math.cos(yaw), 0],
                      [0, 0, 1]])

   
    R = np.dot(R_yaw, np.dot(R_pitch, R_roll))
    
    return R


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return [x,y,z,w]




class aruco_tf(Node):
    

    def __init__(self):
        

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        self.publisher_1 = self.create_publisher(BoxOrientation,'/box_orientation',10)
        self.publisher_2 = self.create_publisher(BoxTransformation,'/box_transformation',10)
        self.publisher_3 = self.create_publisher(Eefpos,'/eef_pos',10)
        self.publisher_4 = self.create_publisher(Bool,'/arm_start',10)
        self.subscriber_1 = self.create_subscription(Bool,'/dock_end',self.callback_1,10)
        self.subscriber_2 = self.create_subscription(Bool,'/arm_cond',self.callback_2,10)


        self.box_pos = []

        self.colour_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None       
        self.iter = 0
        self.j = 0    
        self.condition_1 = None                                      # depth image variable (from depthimagecb())
        self.condition_2 = None                                      # depth image variable (from depthimagecb())





        
    def callback_1(self,msg):
        self.condition_1 = msg.data

    def callback_2(self,msg):
        self.condition_2 = msg.data


    def depthimagecb(self, data):
    
        self.depth_image = self.bridge.imgmsg_to_cv2(data,desired_encoding = 'passthrough')



    def colorimagecb(self, data):
    
        self.colour_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')



        
    def detect_aruco(self, image):
        
        aruco_area_threshold = 1500

        cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

        dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

        # We are using 150x150 aruco marker size
        size_of_aruco_m = 0.15
        # You can remove these variables after reading the instructions. These are just for sample.
        center_aruco_list = []
        # distance_from_rgb_list = []
        angle_aruco_list = []
        width_aruco_list = []
        ids = []  

        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        arucoParams = cv2.aruco.DetectorParameters()
        arucoParams.adaptiveThreshConstant = 10
        arucoParams.minMarkerPerimeterRate = 0.01
        arucoParams.maxMarkerPerimeterRate = 4.0
        arucoParams.polygonalApproxAccuracyRate = 0.05
        arucoParams.minCornerDistanceRate = 0.05
        arucoParams.minDistanceToBorder = 3
        blurred = cv2.GaussianBlur(image, (3, 3), 0)

        # Apply Laplacian filter for edge enhancement
        laplacian = cv2.Laplacian(blurred, cv2.CV_64F)

        # Convert back to uint8
        sharp = np.clip(image - 0.7 * laplacian, 0, 255).astype(np.uint8)

        # try:
        if self.condition_1 == True: #and self.condition_2 == True:
            (corners, ids, rejected) = cv2.aruco.detectMarkers(sharp, arucoDict, parameters=arucoParams)
            msg = Bool()
            msg.data = True
            print("publish")
            self.publisher_4.publish(msg)

        elif self.condition_1 == False:
            print("else")
            msg = Bool()
            msg.data = False
            self.publisher_4.publish(msg)
        old_corners = list(corners)
        new_corners = []
        for markerCorner in corners:
		# extract the marker corners (which are always returned in
		# top-left, top-right, bottom-right, and bottom-left order)
            new_corners.append(markerCorner.reshape((1,8)))       

        corners = new_corners
        self.corners2D = []
        
        corners = [arr.tolist() for arr in corners]  
        
        for i in range(len(corners)):

            bruh = [(corners[i][0][0],corners[i][0][1]),(corners[i][0][2],corners[i][0][3]),(corners[i][0][4],corners[i][0][5]),(corners[i][0][6],corners[i][0][7])]
            self.corners2D.append(bruh)
          
            self.center_ar = [(self.corners2D[i][0][0]+self.corners2D[i][2][0]+self.corners2D[i][1][0]+self.corners2D[i][3][0])/4, (self.corners2D[i][0][1]+self.corners2D[i][2][1]+self.corners2D[i][3][1]+self.corners2D[i][1][1])/4]
            center_aruco_list.append(self.center_ar)
         

            self.width_ar = distance_calc([self.corners2D[i][0],self.corners2D[i][1]])
            width_aruco_list.append(self.width_ar)


        new_ids = ids
        new_ids = new_ids.flatten().tolist()

        j = 0
        while j<len(ids):
                      
            if len(self.corners2D)<=j:
                break

            else:
                area_ar = calculate_rectangle_area(self.corners2D[j])
                if area_ar<aruco_area_threshold:
                   
                    del old_corners[j]
                    del self.corners2D[j]
                    del center_aruco_list[j]
                    del width_aruco_list[j]
                    del new_ids[j]

                else:
                    j = j+1       
      
        
        corners = tuple(old_corners)
        if new_ids is not None:
            cv2.aruco.drawDetectedMarkers(self.colour_image, corners, np.array(new_ids))  
   
        (angle_aruco_list, tvecs,p) = cv2.aruco.estimatePoseSingleMarkers(corners, size_of_aruco_m, cam_mat, dist_mat)
        
        for l in range(len(tvecs)):
            cv2.drawFrameAxes(self.colour_image,cam_mat,dist_mat,angle_aruco_list[l],tvecs[l],0.1)

        return center_aruco_list, angle_aruco_list, width_aruco_list, new_ids, tvecs

    def lookup(self,trans,center,ids):
        self.transform = TransformStamped()
        self.transform.header.stamp = self.get_clock().now().to_msg()  # Use the current timestamp

        self.transform.header.frame_id = 'camera_link'
        self.transform.child_frame_id = 'cam_' + str(ids)

   
        self.transform.transform.translation.x = trans[2]
        self.transform.transform.translation.y = trans[0]
        self.transform.transform.translation.z = trans[1]
   
        y = center[1]
        x = center[0]

            
        if x>=0 and x<1280/3:
            print("left")
            a = "left"

        elif x>=1280/3 and x<(2*1280/3):
            print("center")
            a = "center"

        elif x>=2*1280/3:
            print("right")
            a = "right"

        else:
            print("\n I D K")

        roll = 0
        pitch = 0 
        yaw = (0.788*self.ang_list[self.iter][0][2]) - ((self.ang_list[self.iter][0][2]**2)/3160) 
        r1 = np.array([0.96613241,0,-0.25804681,0,1,0,0.25804681,0,0.96613241])
        r1 = r1.reshape(3,3)

        r = euler_to_rotation_matrix(roll,pitch,-yaw)
        r = np.dot(r1,r)
        r2 = euler_to_rotation_matrix(np.deg2rad(90),0,0)
        r3 = euler_to_rotation_matrix(0,np.deg2rad(90),0)

        r = np.dot(r,r2)
        r = np.dot(r,r3)

        r = R.from_matrix(r)
        r = r.as_quat()
        self.transform.transform.rotation.x = r[0]
        self.transform.transform.rotation.y = r[1]
        self.transform.transform.rotation.z = r[2]
        self.transform.transform.rotation.w = r[3]

        msg = BoxOrientation()
        msg.id = ids
        msg.position = a
        self.br.sendTransform(self.transform)
        self.publisher_1.publish(msg)

 


    def on_timer(self):
        try:
            # Listen to the transform
            t = self.tf_buffer.lookup_transform('base_link','cam_'+str(self.new_ids[self.iter]), rclpy.time.Time())
            t2 = self.tf_buffer.lookup_transform('base_link','tool0', rclpy.time.Time())

            
            self.tm = TransformStamped()
            self.tm.header.stamp = self.get_clock().now().to_msg()  # Use the current timestamp

            self.tm.header.frame_id = 'base_link'
            self.tm.child_frame_id = 'obj_'+str(self.new_ids[self.iter])
  
            self.tm.transform.translation.x = t.transform.translation.x
            self.tm.transform.translation.y = t.transform.translation.y
            self.tm.transform.translation.z = t.transform.translation.z
            self.tm.transform.rotation.x = t.transform.rotation.x
            self.tm.transform.rotation.y = t.transform.rotation.y
            self.tm.transform.rotation.z = t.transform.rotation.z
            self.tm.transform.rotation.w = t.transform.rotation.w
          
            self.br.sendTransform(self.tm)

            msg = Eefpos()
            msg.x = t2.transform.translation.x
            msg.y = t2.transform.translation.y
            msg.z = t2.transform.translation.z
            self.publisher_3.publish(msg)


            self.box_pos = [[self.new_ids[self.iter],[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]]]
    
            
        except:
            self.get_logger().error(f"Error while looking up transform")

    def process_image(self):
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375

        try:
            (center_aruco_list,  angle_aruco_list, width_aruco_list, new_ids,tvecs) = self.detect_aruco(self.colour_image)
            print(self.j)

            if self.j == 0:
                self.center_aruco_list = center_aruco_list
                self.new_ids = new_ids

            self.quat = []
            self.ang_correc = []

            
            self.correc_cent = []
            self.aruco_center = []
            self.depth_center = []
            self.ang_list = []
            self.ang_list = angle_aruco_list

            for l1 in range(len(self.center_aruco_list)):

                depth_center = self.depth_image[int(self.center_aruco_list[l1][1]),int(self.center_aruco_list[l1][0])]/1000
                x = depth_center * (sizeCamX - (self.center_aruco_list[l1][0]) - centerCamX) / focalX
                y = depth_center * (sizeCamY - (self.center_aruco_list[l1][1]) - centerCamY) / focalY
                z = depth_center
                center = (self.center_aruco_list[l1][0],self.center_aruco_list[l1][1])
                self.aruco_center.append((x,y,z))
            
                center = (int(center[0]),int(center[1]))
        
                self.correc_cent.append(center)
                cv2.circle(self.colour_image,center,2,(255, 255, 255),2)



            print(self.iter)
            self.lookup(self.aruco_center[self.iter],self.center_aruco_list[self.iter],self.new_ids[self.iter])
            self.timer2 = self.create_timer(1.0, self.on_timer)
            
            self.msg = BoxTransformation()
            self.msg.x = self.box_pos[self.iter][1][0]
            self.msg.y = self.box_pos[self.iter][1][1]
            self.msg.z = self.box_pos[self.iter][1][2]
            self.msg.id = self.box_pos[self.iter][0]
            self.publisher_2.publish(self.msg)
           
        except:
            print("we are here to stay :)")
            

def main():


    rclpy.init(args=sys.argv)                                       # initialisation
    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node
    node.get_logger().info('Node created: Aruco tf process')        # logging information
    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

     
    try:
        rclpy.spin(aruco_tf_class)  
    except KeyboardInterrupt:
        pass  
                
    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                    
 


if __name__ == '__main__':
    
    main()
    # cv2.destroyAllWindows()