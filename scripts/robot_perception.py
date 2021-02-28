#!/usr/bin/env python3

import rospy, cv2, numpy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
# import keras_ocr


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

        self.searching_for_color = True
        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)

        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # download pre-trained model
        # self.pipeline = keras_ocr.pipeline.Pipeline()

    # def char_image_callback(self, img):
        # Once you have the pipeline, you can use it to recognize characters,

        # images is a list of images in the cv2 format
        # images = [img1, img2, ...]

        # call the recognizer on the list of images
        # prediction_groups = pipline.recognize(images)

        # prediction_groups is a list of predictions for each image
        # prediction_groups[0] is a list of tuples for recognized characters for img1
        # the tuples are of the formate (word, box), where word is the word
        # recognized by the recognizer and box is a rectangle in the image where the recognized words reside

    def image_callback(self, msg):
        if self.searching_for_color:
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
                    # center of the yellow pixels in the image
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                    # a red circle is visualized in the debugging window to indicate
                    # the center point of the yellow pixels
                    # cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                    # TODO: based on the location of the line (approximated
                    #       by the center of the yellow pixels), implement
                    #       proportional control to have the robot follow
                    #       the yellow line
                    # Constants
                    xx = w/2
                    yy = 370
                    vel_msg = Twist()
                    if cy < 280:
                        vel_msg.linear.x = .1
                    else:
                        vel_msg.linear.x = 0

                    print("x: " + str(cx) + "y: " + str(cy))
                    kp = .01
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
        


    def run(self):
        rospy.spin()
            
if __name__ == '__main__':
    rospy.init_node('Perception')
    follower = Perception()
    follower.run()
