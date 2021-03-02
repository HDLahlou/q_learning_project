#!/usr/bin/env python3

import rospy, cv2, numpy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import keras_ocr
import matplotlib.pyplot as plt
from io import BytesIO



class Perception:

    def __init__(self):
        # set up ROS / OpenCV bridge
        self.bridge = CvBridge()

        lower_red = numpy.array([0, 100, 100])
        upper_red = numpy.array([10, 255, 255])
        self.red_ranges = [lower_red, upper_red]

        lower_blue = numpy.array([110, 100, 100])
        upper_blue = numpy.array([130, 255, 255])
        self.blue_ranges = [lower_blue, upper_blue]

        lower_green = numpy.array([50, 100, 100])
        upper_green = numpy.array([70, 255, 255])
        self.green_ranges = [lower_green, upper_green]

        lower_black = numpy.array([0, 0, 0])
        upper_black = numpy.array([1, 10, 10])
        self.black_ranges = [lower_black, upper_black]

        # Flag if facing towards cubes, triggers search for cubes/black colors
        self.facing_towards_blocks = False
        # Flag for reading cubes and detecting matching text
        self.reading_cubes = False
        # Flag to help stop callback from constantly running and computing
        self.seen_first_image = False
        # Flag for once found the desired cube
        self.found = False
        # Flag for when robot is correctly orriented towards cube goal
        self.facing_target = False
        # Flag for when robot is in front of cube

        self.has_arrived = False
        self.scan_regions = {}
        self.prediction_group = []

        # Flags to determine if searching for cube or dumbells
        self.searching_for_color = False
        self.read_text = True
    
        self.pipeline = keras_ocr.pipeline.Pipeline()

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)

        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.init_flag = True
        # download pre-trained model


    def scan_callback(self, msg):
        size = len(msg.ranges)
        # Determines the ranges for scan, finds each "side"
        regions = {
            'right':  min(min(msg.ranges[0:(int(size/5)-1)]), 10),
            'fright': min(min(msg.ranges[(int(size/5)):(int(2*size/5)-1)]), 10),
            'front':  min(min(msg.ranges[(int(2*size/5)):(int(3*size/5)-1)]), 10),
            'fleft':  min(min(msg.ranges[(int(3*size/5)):(int(4*size/5)-1)]), 10),
            'left':   min(min(msg.ranges[(int(4*size/5)): (size-1)]), 10),
        }
        self.scan_regions = regions


    def detect_black(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,'bgr8')

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.black_ranges[0], self.black_ranges[1])

        h, w, d = image.shape

        # using moments() function, the center of the yellow pixels is determined
        M = cv2.moments(mask)
        # print(M)
        # if there are any yellow pixels found
        if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                xx = w/2
                if(cx == xx):
                    self.facing_towards_blocks = True
                else:
                    vel_msg = Twist()
                    vel_msg.linear.x = 0
                    kp = .001
                    # Calc of offset on center of yellow
                    et = xx - cx
                    # Calculation to center
                    pc = kp*et
                    vel_msg.angular.z = pc
                    self.velocity_pub.publish(vel_msg)
        else:
            vel_msg = Twist()
            vel_msg.linear.x = 0
            vel_msg.angular.z = .1
            self.velocity_pub.publish(vel_msg)

    def face_cubes(self, msg):
        if (not self.seen_first_image):
            vel_msg = Twist()
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_pub.publish(vel_msg)
            self.seen_first_image = True
            image = self.bridge.imgmsg_to_cv2(msg,'bgr8')
            
            # call the recognizer on the list of images
            prediction_groups = self.pipeline.recognize([image])
            input = '3'

            while prediction_groups is None:
                print("Waiting")
                pass

            self.found = False
            if len(prediction_groups[0]) == 0:
                self.seen_first_image = False
                self.facing_towards_blocks = False
                print("Reseting orientation")
                return 0
            if len(prediction_groups[0]) > 0:
                print("Inspecting predictions")
                for x in prediction_groups[0]:
                    if x[0] == input:
                        print(x[0])
                        self.found = True
                        self.prediction_group = x
                        self.reading_cubes = True 
            if (not self.found):
                print("Adjusting")
                self.seen_first_image = False
                vel_msg = Twist()
                vel_msg.linear.x = 0
                vel_msg.angular.z = .2
                self.velocity_pub.publish(vel_msg)
                rospy.sleep(1)

    def approach_cube(self, msg):
        if(not self.facing_target):
            image = self.bridge.imgmsg_to_cv2(msg,'bgr8')
            h, w, d = image.shape
            print(self.prediction_group)
            center_x = w/2
            adjust = (w - (self.prediction_group[1][0][0] + self.prediction_group[1][1][0]))/2
            print(adjust)
            if abs(adjust) > 80:
                kp = .001
                pc = kp*adjust
                vel_msg = Twist()
                vel_msg.angular.z = pc
                self.velocity_pub.publish(vel_msg)
                rospy.sleep(0.5)
                vel_msg.angular.z = 0
                self.velocity_pub.publish(vel_msg)
                self.found = False
                self.reading_cubes = False
                self.seen_first_image = False
            else:
                print("Centered")
                self.facing_target = True
        else:
            # Orientation found
            vel_msg = Twist()
            # Desired distance to stop from nearest object
            desired_distance = .75
            vel_msg.linear.x = .0
            print("Left")
            print(self.scan_regions["left"])
            print("Right")
            print(self.scan_regions["right"])

            if self.scan_regions["left"] > desired_distance or self.scan_regions["right"] > desired_distance:
                vel_msg.linear.x = .2
            else:
                vel_msg.linear.x = 0
                self.has_arrived = True
            self.velocity_pub.publish(vel_msg)





    def char_image_callback(self, msg):
        if (not self.facing_towards_blocks):
            self.detect_black(msg)
        elif (not self.reading_cubes):
            # print("Face Cubes")
            self.face_cubes(msg)
        elif (self.found and (not self.has_arrived)):
            self.approach_cube(msg)
        elif self.has_arrived:
            print("Drop dumbell")

    def color_detect_and_move(self, msg):
        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(msg,'bgr8')

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        input = "green"
        if input == "red":
            color_range = self.red_ranges
        elif input == "blue":
            color_range = self.blue_ranges
        elif input == "green":
            color_range = self.green_ranges


        mask = cv2.inRange(hsv,color_range[0], color_range[1])

        h, w, d = image.shape

        # using moments() function, the center of the yellow pixels is determined
        M = cv2.moments(mask)
        # print(M)
        # if there are any yellow pixels found
        if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                xx = w/2
                yy = 370
                vel_msg = Twist()
                if cy < 280:
                    vel_msg.linear.x = .1
                else:
                    vel_msg.linear.x = 0

                print("x: " + str(cx) + "y: " + str(cy))
                kp = .005
                # Calc of offset on center of yellow
                et = xx - cx
                # Calculation to center
                pc = kp*et
                vel_msg.angular.z = pc
                self.velocity_pub.publish(vel_msg)
        else:
            vel_msg = Twist()
            vel_msg.linear.x = 0
            vel_msg.angular.z = .3
            self.velocity_pub.publish(vel_msg)


    def image_callback(self, msg):
        if self.init_flag:
            if self.read_text:
                self.char_image_callback(msg)
            elif self.searching_for_color:
                self.color_detect_and_move(msg)
        


    def run(self):
        rospy.spin()
            
if __name__ == '__main__':
    rospy.init_node('Perception')
    follower = Perception()
    follower.run()
