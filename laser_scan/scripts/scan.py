#! /usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Range


class scaner(object):
    def __init__(self):
        self.pub_front_obstacle= rospy.Publisher('/laser/front_obstacle_turn', String, queue_size=10)
        self.pub_park= rospy.Publisher('/laser/park', String, queue_size=10)
        self.sub_front_obstacle=rospy.Subscriber('/scan_filtered_angle', LaserScan, self.callback_frontal_obstacle)
        #self.sub_all_angles=rospy.Subscriber('/scan_filtered_interpolation', LaserScan, self.callback_all_angles)
        self.sub_all_angles=rospy.Subscriber('/scan', LaserScan, self.callback_all_angles)
        self.sub_left_line_type=rospy.Subscriber('/vision/left_line_type', String, self.callback_left_line_type)
        self.sub_right_line_type=rospy.Subscriber('/vision/right_line_type', String, self.callback_right_line_type)
        self.sub_control_align=rospy.Subscriber('/control/alignment', String, self.callback_control_alignment)
        self.sub_right_sonar=rospy.Subscriber('/adc_car/sensors/range/side_right_sonar', Range, self.callback_right_sonar)


        """Variables"""
        self.left_line_type=""
        self.right_line_type=""
        self.front_obstacle_turn=""
        self.front_obstacle_enable=1
        self.parking_enable=1
        self.out_enable=1
        self.high_corner=None
        self.low_corner=None
        self.counter_refresh=0
        #self.counter_aligned=0
        self.estat_parking="detect"
        self.enable_sonar=False
        self.return_callback_sonar=""
        self.return_callback_sonar_right=None
        self.correction_done=False
        self.counter_open_car=0
        self.parking_side = ""
        self.control_alignment=""
        self.counter_center=0
        #self.first_parking_right_enable=1
        #self.first_parking_right=0
        #self.counter_maq=0
        self.timer_start= None


    def callback_left_line_type(self, msg):
        self.left_line_type=msg.data

    def callback_right_line_type(self, msg):
        self.right_line_type=msg.data

    def callback_all_angles(self,msg):
        if (self.parking_enable==1):
            self.maquina_estats_parking(msg)
            self.maquina_estats_out_parking(msg)

    def callback_control_alignment(self,msg):
        self.control_alignment=msg.data

    def callback_right_sonar(self, msg):
        if self.enable_sonar==True:
            self.return_callback_sonar_right=msg.range
            if ( msg.range > 1.5):
                self.return_callback_sonar="out"
                self.enable_sonar=False
            else:
                self.return_callback_sonar="continue"
            #print (self.return_callback_sonar)
        #ERROR, SEMPRE ENTRA EN IMPACTE. ELS VALORS QUE DONA EL SONAR AL ESTAR DINTRE EL PARKING NO SON pub_front_obstacle
        """if (0.7 < msg.range < 0.8 or msg.range< 0.5):
            self.correction_done=False
            self.return_callback_sonar="impact"
        else:
            if (0.55<msg.range<0.65):
                self.counter_aligned+=1
                self.correction_done=True
            if self.counter_aligned==5:
                self.return_callback_sonar="aligned"
            else:
                print("continue")
                self.return_callback_sonar="continue"
                """


    def maquina_estats_parking(self,msg):

        if self.estat_parking == "detect":
            #self.estat_parking="park_done" #test line
            self.detect_parking(msg)
        elif self.estat_parking == "parking_right_detected" or self.estat_parking == "parking_left_detected":
            self.high_corner=None
            self.low_corner=None
            self.front_obstacle_enable=0
            if self.estat_parking == "parking_right_detected":
                self.parking_side="right"
            elif self.estat_parking == "parking_left_detected":
                self.parking_side="left"
            """if self.first_parking_right==1:
                self.pub_park.publish("back")
                self.detect_parking(msg)
            else:
                self.park_fase1(msg)"""
            self.park_fase1(msg)
        elif self.estat_parking == "back_left_maneuver" or self.estat_parking == "back_right_maneuver":
            self.counter_open_car=0
            self.park_fase2(msg)
        elif self.estat_parking == "right_align" or self.estat_parking == "left_align":

            self.park_fase3(msg)
        elif self.estat_parking == "aligned":
            if not(self.detect_near_object_front(msg)):
                self.pub_park.publish("front")
            else:
                self.pub_park.publish("stop")
                self.high_corner=None
                self.low_corner=None
                self.estat_parking="park_done"

    def maquina_estats_out_parking(self, msg):
        if self.estat_parking=="park_done" and self.out_enable==1:
            self.out(msg)
        elif self.estat_parking=="out_fase_2":
            self.corner_alignment(msg)
        elif self.estat_parking == "out_fase_3":
            self.out_fase_3(msg)
        elif self.estat_parking=="line_alignment":
            self.line_alignment(msg)
        elif self.estat_parking=="out_done":
            self.estat_parking = "detect"




    def out(self, msg):
        self.enable_sonar=True
        if self.return_callback_sonar =="out":
            self.enable_sonar=False
            self.pub_park.publish("stop")
            print ("out")
            self.return_callback_sonar=""
            self.estat_parking="out_fase_2"
        else:
            self.pub_park.publish("back")

    """def detect_end_out(self, msg):
        for i in range (0, len(msg.ranges)):
            if ((i < 20 and msg.ranges[i] > 1 ) or (i>280 and msg.ranges[i] > 1) ):
                return "stop"
            else:
                return "continue"
                """
    def corner_alignment(self, msg):
        align_ret= self.align_parking(msg, "back_out")
        if (align_ret=="continue" ):
            if self.parking_side=="right":
                self.pub_park.publish("right_back")
            else:
                self.pub_park.publish("left_back")
        elif (align_ret=="change"):
            self.pub_park.publish("stop")
            self.timer_start = time.time()
            self.estat_parking="out_fase_3"
    def line_alignment(self, msg):
        self.pub_park.publish("alignment_info")
        print(self.control_alignment)
        if self.control_alignment != "center":
            self.counter_center=0
            if self.parking_side=="right":
                #print ("right_back")
                self.pub_park.publish("right_back")
            else:
                #print ("left_back")
                self.pub_park.publish("left_back")
        else:
            self.counter_center+=1
            if self.counter_center==4:
                self.pub_park.publish("stop")
                self.pub_park.publish("end")
                self.estat_parking="out_done"

    def out_fase_3(self, msg):
        """self.enable_sonar=True;
        if not(0.9<self.return_callback_sonar_right<1):
            if self.parking_side=="right":
                self.pub_park.publish("left_front")
            elif self.parking_side=="left":
                self.pub_park.publish("right_front")
        else:
            print ("stopping")
            self.enable_sonar=False
            self.pub_park.publish("stop")
            self.estat_parking="line_alignment"
            """
        if time.time()-self.timer_start < 16:
            if self.parking_side=="right":
                self.pub_park.publish("left_front")
            elif self.parking_side=="left":
                self.pub_park.publish("right_front")
        else:
            self.pub_park.publish("stop")
            self.estat_parking="line_alignment"


    def park_fase1(self, msg):
        if not(self.detect_near_object(msg)):
            self.counter_open_car+=1
            if self.estat_parking == "parking_right_detected":
                if self.counter_open_car<1:
                    self.pub_park.publish("left_front")
                else:
                    self.pub_park.publish("right_front")
            elif self.estat_parking == "parking_left_detected":
                if self.counter_open_car<1:
                    self.pub_park.publish("right_front")
                else:
                    self.pub_park.publish("left_front")

        else:
            self.pub_park.publish("stop")
            if self.estat_parking ==  "parking_right_detected":
                self.estat_parking = "back_left_maneuver"
            elif self.estat_parking ==  "parking_left_detected":
                self.estat_parking = "back_right_maneuver"

    def detect_near_object(self,msg):
        counter_near=0
        for i in range(len(msg.ranges)):
            #menos de 0.25 el laser no detecta bn
            if msg.ranges[i] < 0.2:
                counter_near+=1
            if counter_near > 5 :
                return True
        return False

    def detect_near_object_front(self,msg):
        counter_near=0
        for i in range(100, 300):
            #menos de 0.25 el laser no detecta bn
            if msg.ranges[i] < 0.2:
                counter_near+=1
            if counter_near > 5 :
                return True
        return False

    def detect_near_object_range(self, start, finish, msg):
        counter_near=0
        for i in range(start, finish):
            #menos de 0.25 el laser no detecta bn
            if msg.ranges[i] < 0.2:
                counter_near+=1
            if counter_near > 5 :
                return True
        return False

    def park_fase2(self, msg):
        align_ret= self.align_parking(msg, "back")
        if (align_ret=="continue" ):
            if self.estat_parking == "back_left_maneuver":
                self.pub_park.publish("left_back")
            elif self.estat_parking == "back_right_maneuver":
                self.pub_park.publish("right_back")
        elif (align_ret=="change"):
            self.pub_park.publish("stop")
            self.high_corner=None
            self.low_corner=None
            print ("change_from_2")
            if self.estat_parking ==  "back_left_maneuver":
                self.estat_parking = "right_align"
            elif self.estat_parking ==  "back_right_maneuver":
                self.estat_parking = "left_align"

    def align_parking(self, msg, action):
        if (action =="back"):
            top_corner_limits=[190,205]
            #low_corner_limits=[360,390]
        elif (action =="back_out"):
            top_corner_limits=[200,220]
            #low_corner_limits=[360,390]
        elif (action=="front"):
            top_corner_limits=[150, 170]
            low_corner_limits=[230, 250]
        """
        if (action =="back"):
            top_corner_limits=[190,210]
            #low_corner_limits=[360,390]
        elif (action=="front"):
            top_corner_limits=[150, 170]
            #low_corner_limits=[230, 250]
            """
        self.counter_refresh+=1

        if (self.counter_refresh % 10) ==0:
            self.high_corner=None
            self.low_corner=None



        for i in range (10, len(msg.ranges)-10):
            if (msg.ranges[i] > 0.3):
                if (action=="back"):
                    #detect top corner
                        if  (msg.ranges[i-1] > msg.ranges[i] < msg.ranges[i+1]):
                            if (top_corner_limits[0] < i < top_corner_limits[1]):
                                past_high_corner=self.high_corner
                                self.high_corner=i
                                for j in range (i-10 , i+10):
                                    if msg.ranges[j]<msg.ranges[i]:
                                        self.high_corner=past_high_corner
                                        break



                        #detect low corner

                        """if  (msg.ranges[i-1] > msg.ranges[i] < msg.ranges[i+1]):
                            if  (low_corner_limits[0] < i <low_corner_limits[1]):
                                past_low_corner=self.low_corner
                                self.low_corner=i
                                for j in range (i-10 , i+10):
                                    if msg.ranges[j]<msg.ranges[i]:
                                        self.low_corner=past_low_corner
                                        break"""
                elif action == "back_out":
                        if  (msg.ranges[i-1] > msg.ranges[i] < msg.ranges[i+1]) and msg.ranges[i]<0.65:
                            if (top_corner_limits[0] < i < top_corner_limits[1]):
                                past_high_corner=self.high_corner
                                self.high_corner=i
                                for j in range (i-15 , i+15):
                                    if msg.ranges[j]<msg.ranges[i]:
                                        self.high_corner=past_high_corner
                                        break

                elif action=="front":
                    if  (msg.ranges[i-1] < msg.ranges[i] > msg.ranges[i+1]):
                        if (top_corner_limits[0] < i < top_corner_limits[1]):
                            past_high_corner=self.high_corner
                            self.high_corner=i
                            for j in range (i-10 , i+10):
                                if msg.ranges[j]>msg.ranges[i]:
                                    self.high_corner=past_high_corner
                                    break



                    #detect low corner
                    if  (msg.ranges[i-1] < msg.ranges[i] > msg.ranges[i+1]):
                        if  (low_corner_limits[0] < i <low_corner_limits[1]):
                            past_low_corner=self.low_corner
                            self.low_corner=i
                            for j in range (i-10 , i+10):
                                if msg.ranges[j]>msg.ranges[i]:
                                    self.low_corner=past_low_corner
                                    break





            #detect parking at right or left depending on angle
            """
        if self.high_corner !=None:
            print "high" , self.high_corner
        if self.low_corner !=None:
            print "low", self.low_corner
            """

        if action=="back" or action == "back_out":
            if self.high_corner !=None :
                self.high_corner=None
                self.low_corner=None
                return "change"
            else:
                return "continue"
        elif action=="front":
            if self.high_corner !=None and self.low_corner !=None:
                self.high_corner=None
                self.low_corner=None
                print ("aligned1")
                return "aligned"
            else:
                return "continue"
        """
        if self.high_corner !=None :
            self.high_corner=None
            #self.low_corner=None
            if action=="back":
                return "change"
            else:
                return "aligned"
        else:
            return "continue"
            """


    def park_fase3(self,msg):
        align_ret = self.align_parking(msg, "front")
        if (align_ret =="continue" ):
            if self.estat_parking == "right_align":
                self.pub_park.publish("right_front")
            elif self.estat_parking == "left_align":
                self.pub_park.publish("left_front")
        elif (align_ret =="aligned"):
            print ("aligned")
            self.pub_park.publish("stop")
            self.estat_parking = "aligned"


        """
        impact= self.detect_impact_sides(msg)
        if impact == None:
            if self.estat_parking == "right_align":
                self.pub_park.publish("right_front")
            elif self.estat_parking == "left_align":
                self.pub_park.publish("left_front")
        elif impact == "right":
            self.pub_park.publish("left_front")
        elif impact == "left":
            self.pub_park.publish("right_front")
        elif impact == "aligned":
            self.pub_park.publish("stop")
        """
        """
        #WHITH SONAR
        self.enable_sonar=True
        if self.return_callback_sonar=="continue":
            if self.estat_parking == "right_align":
                print "continue right"
                self.pub_park.publish("right_front")
            elif self.estat_parking == "left_align":
                print "continue left"
                self.pub_park.publish("left_front")
        elif self.return_callback_sonar=="aligned":
            self.pub_park.publish("stop")
            self.estat_parking="finish"
        elif self.return_callback_sonar=="impact":
            self.pub_park.publish("stop")
            if self.estat_parking ==  "right_align":
                self.estat_parking = "back_left_maneuver"
            elif self.estat_parking ==  "left_align":
                self.estat_parking = "back_right_maneuver"
    """
    """

    def detect_impact_sides(self, msg):
        counter_near=0
        for i in range(len(msg.ranges)):
            if (0.21 < msg.ranges[0] < 0.23) and (0.21 < msg.ranges[len(msg.ranges)-1] < 0.23) :
                return "aligned"
            #menos de 0.25 el laser no detecta bn
            if msg.ranges[i] < 0.20:
                counter_near+=1
            if counter_near > 5 :
                if (i < 200):
                    return "left"
                else:
                    return "right"
        return None

    """

    def detect_parking(self, msg):

        self.counter_refresh+=1
        if (self.counter_refresh % 5) ==0 :
            self.high_corner=None
            self.low_corner=None

        for i in range (10, len(msg.ranges)-10):
            #detect top corner
            if  (msg.ranges[i-1] > msg.ranges[i] < msg.ranges[i+1]):
                if  (1.5 > msg.ranges[i] > 0.5)  and (350 > i > 50):
                    past_high_corner=self.high_corner
                    self.high_corner=i
                    for j in range (i-10 , i+10):
                        if msg.ranges[j]<msg.ranges[i]:
                            self.high_corner=past_high_corner
                            break
            #detect low corner
            if (i > len(msg.ranges)/2):

                if (msg.ranges[i-1]>1.2  and 0.8 > msg.ranges[i]>0.4):
                    past_low_corner=self.low_corner
                    self.low_corner=i
                    for j in range (i-5, i+5):
                        if j < i and msg.ranges[j] < 1.2 :
                            self.low_corner=past_low_corner
                            break
                        elif j >i and msg.ranges[j] > 0.7:
                            self.low_corner=past_low_corner
                            break


            else:
                if (msg.ranges[i+1]>1.2  and 0.8 > msg.ranges[i]>0.4):
                    past_low_corner=self.low_corner
                    self.low_corner=i
                    for j in range (i-5, i+5):
                        if j > i and msg.ranges[j] < 1.2 :
                            self.low_corner=past_low_corner
                            break
                        elif j < i and msg.ranges[j] > 0.7:
                            self.low_corner=past_low_corner
                            break



            #detect parking at right or left depending on angle
            """
            if self.high_corner !=None and self.low_corner !=None:
                print "high" , self.high_corner
                print "low", self.low_corner


            if (360>self.high_corner>290) and (340<self.low_corner<395):
                self.estat_parking="parking_right_detected"
            if (120>self.high_corner>70) and (5<self.low_corner<60):
                self.estat_parking = "parking_left_detected"
            """
            """
            if (360>self.high_corner>290) and (360<self.low_corner<395):
                self.estat_parking="parking_right_detected"

            if (120>self.high_corner>70) and (5<self.low_corner<60):
                self.estat_parking = "parking_left_detected"
            """
            """
            if (self.first_parking_right_enable==1) :
                if (360>self.high_corner>330):
                    print ("first")
                    self.first_parking_right=1
                    self.estat_parking="parking_right_detected"
            """
            if (320>self.high_corner>300) and (360<self.low_corner<400):
                #print "high" , self.high_corner
                #print "low", self.low_corner
                """if self.first_parking_right==1:
                    self.first_parking_right=0"""
                self.estat_parking="parking_right_detected"


            if (120>self.high_corner>70) and (5<self.low_corner<60):
                self.estat_parking = "parking_left_detected"


    def callback_frontal_obstacle(self, msg):
        if (self.front_obstacle_enable==1):
            obstacle=False
            counter_obstacle=0
            front_max_distance=1 # NO CANVIAR
            front_max_points=25
            for i in range (len(msg.ranges)):
                if msg.ranges[i]<front_max_distance:
                    counter_obstacle+=1
                if counter_obstacle>front_max_points:
                    obstacle = True
                    break
            if obstacle ==True:
                if self.right_line_type=="cont" and self.left_line_type=="disc":
                    self.front_obstacle_turn="left"
                    """
                    if self.first_parking_right_enable==1:
                        self.first_parking_right=1
                    """
                elif self.left_line_type=="cont" and self.right_line_type=="disc":
                    self.front_obstacle_turn="right"
                else:
                    self.front_obstacle_turn=""
            else:
                self.front_obstacle_turn=""

            self.obstacle_avoid()





    def obstacle_avoid(self):
        if self.front_obstacle_turn != "":
            self.pub_front_obstacle.publish(self.front_obstacle_turn)

if __name__ == '__main__':


    rospy.init_node('scan')
    scaner= scaner()

    try:
        rospy.spin()
    except rospy.ROSInterruptException or KeyboardInterrupt:
        print("Shutting down")
