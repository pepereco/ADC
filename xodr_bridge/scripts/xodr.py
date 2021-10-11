#!/usr/bin/python
import rospy
import time
import xml.etree.ElementTree as ET
from graph import *
from math import sin, cos
from iri_adc_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import tf
from iri_model_car_msgs.msg import encoders

XODR_DIM = 10
TURN_LEFT_ENCODERS = 70
TURN_RIGHT_ENCODERS = 15
TURN_RIGHT_ENCODERS
TICKS_PER_METER = 9.52
CAR_TICKS_DIM = 20
MAX_ANGLE_TURN = 1.2

def get_graph_from_xodr_file(file_path):
    """
    Returns the graph of the xodr file processed.
    """
    mygraph = Graph()

    ## Getting the file
    root = ET.parse(file_path).getroot()

    ## Loop for each junction
    for junction in root.findall('junction'):

        ## SOBREENTENC QUE PUC ANAR D'UN LLOC A UN ALTRE EN UNA JUNCTION
        ## SEMPRE, VULL DIR NO MIRO LES CONNEXIONS INTERNES DE LA JUNCTION
        mygraph.nodes.append(Node(mygraph, junction.get('id')))


    ## Loop for each road
    mydumper = RoadDumper(mygraph)

    for road in root.findall('road'):

        # Getting if it's an actual road
        if road.get('junction') == '-1':

            # Getting road data
            id = road.get('id')
            pred = road.find('link').find('predecessor')
            succ = road.find('link').find('successor')
            pred_type = pred.get('elementType')
            succ_type = succ.get('elementType')
            pred_id = pred.get('elementId')
            succ_id = succ.get('elementId')
            length = float(road.get('length'))

            #TODO: Check if one side or both
            two_sides = True

            geos = road.find('planView').findall('geometry')

            if len(geos) == 1: #Straight line, no curves.
                startcoords = Coords( float(geos[0].get('x')), float(geos[0].get('y')), float(geos[0].get('hdg')) ) # x, y, yaw(radians)
                curvature = 0 # 0 radians per meter
                endcoords = Coords( startcoords.x + length*cos(startcoords.yaw), startcoords.y + length*sin(startcoords.yaw), startcoords.yaw )

            else: #Curve
                startcoords = Coords( float(geos[0].get('x')), float(geos[0].get('y')), float(geos[0].get('hdg')) ) # x, y, yaw(radians)
                curvature = float(geos[1].find('arc').get('curvature'))
                endcoords = Coords( float(geos[2].get('x')), float(geos[2].get('y')), float(geos[2].get('hdg')) ) # x, y, yaw(radians)

            # Adding the subroad.
            tmp = Road(id, pred_type, succ_type, pred_id, succ_id, two_sides, length, startcoords, curvature, endcoords)
            mydumper.add(tmp)

    mydumper.connect()
    return mygraph

#This plots a curved road, to show that split_line works.
if __name__ == "__main__OFF":
    import matplotlib.pyplot as plt

    coordinates = split_line(Coords(10, 0, 1.59), 0.1, 15.9)

    for point in coordinates:
        plt.plot(point.x, point.y, marker='*')
        #print(point)

    plt.show()

class goal_resolver(object):
    def __init__(self,mygraph):
        self.pub_control= rospy.Publisher('/xodr/turn', String, queue_size=10)
        self.pub_fb= rospy.Publisher('/adc_car/fb', String, queue_size=10)
        self.pub_parking_enable= rospy.Publisher('/xodr/parking_enable', String, queue_size=10)
        self.graph=mygraph
        self.position = None
        self.x=0
        self.y = 0
        self.yaw=0
        self.goal=None
        self.type=None
        self.path_angles=[]
        self.last_meters=0
        self.intersection_pos=False
        self.encoder_start=0
        self.encoder=0
        self.first_entry=True
        self.counter=0
        self.change_angle=False
        self.enable_turn=True
        self.encoder_start_end=0
        self.finish_goal=False
        self.goal_list=[]
        self.enable_parking=False
        self.last_ticks=None
        self.counter_change_angle=0
        self.first_yaw=0
        self.change_angle_counter=0
        self.park_sub = rospy.Subscriber("/parking/state", String, self.cb_park_state)
        self.goal_sub = rospy.Subscriber("/adc_car/goal", adc_goal, self.goal_cb)
        self.position_sub =  rospy.Subscriber("/adc_car/amcl_pose", PoseWithCovarianceStamped, self.pos_cb)
        self.no_lineSub = rospy.Subscriber("/vision/no_line", String, self.no_line_cb)
        self.sub_encoders=rospy.Subscriber('/adc_car/encoders', encoders, self.cb_encoders)
    def cb_park_state(self, msg):
        self.pub_fb.publish(msg.data)

    def cb_encoders(self, msg):
        self.encoder=msg.right_ticks
    def no_line_cb(self, side):
        self.change_angle=False
        #if self.first_entry==True and (10 <self.x < 60):
        if self.first_entry==True:
            print ("first entry")
            self.first_yaw=self.yaw
            self.first_entry=False
            self.encoder_start=self.encoder
        if self.intersection_pos==True and self.enable_turn==True:
            self.intersection_action()
    def intersection_action(self):
        self.counter+=1
        #print ("angles: ", self.path_angles)
        if self.path_angles!=[]:
            #if 1.58>self.path_angles[0]>1.56 and ((self.x > 27 and self.y < 50) or (self.x < 43 and self.y >50)):
            print ("encoders: ", self.encoder- self.encoder_start)
            print ("angle: ", self.yaw)
            if 1.58>self.path_angles[0]>1.56 and (self.encoder- self.encoder_start >TURN_LEFT_ENCODERS):
                #print ("turn left counter", self.counter)
                if self.yaw - self.first_yaw > MAX_ANGLE_TURN :
                    #self.change_angle=True
                    self.pub_control.publish("end")
                else:
                    self.pub_control.publish("left_front")
            elif -1.58 < self.path_angles[0]<-1.56 and (self.encoder- self.encoder_start >TURN_RIGHT_ENCODERS):
                if self.yaw - self.first_yaw< - MAX_ANGLE_TURN :
                    #self.change_angle=True
                    self.pub_control.publish("end")
                else:
                    print ("turn right")
                    self.pub_control.publish("right_follow")

    def next_goal(self):
        self.first_entry=True
        self.change_angle=False
        self.enable_turn=True
        self.finish_goal=False
        del self.goal_list[0]
        self.goal = self.goal_list[0]
        self.path = (mygraph.from_point_to_point(Coords(self.x,self.y,self.yaw), Coords(self.goal.x*XODR_DIM, self.goal.y*XODR_DIM, self.goal.yaw)))
        self.path_angles=self.path.angles
        self.last_meters=self.path.lastmeters
        self.type = self.goal.type
        print ("yaws:", self.path.angles)
        print ("meters", self.path.lastmeters)



    def goal_cb (self,goal):
        self.goal_list.append(goal)
        self.goal = self.goal_list[0]
        self.path = (mygraph.from_point_to_point(Coords(self.x,self.y,self.yaw), Coords(self.goal.x*XODR_DIM, self.goal.y*XODR_DIM, self.goal.yaw)))
        self.path_angles=self.path.angles
        self.last_meters=self.path.lastmeters
        self.type = self.goal.type
        if len(self.path_angles)==0:
            self.finish_goal=True
            self.calculate_meters()
            self.encoder_start_end=self.encoder
            print ("last ticks",self.last_ticks)
        print ("yaws:", self.path.angles)
        print ("meters", self.path.lastmeters)
    def pos_cb(self,pos):
        self.position=pos.pose.pose
        self.x=self.position.position.x*XODR_DIM
        self.y=self.position.position.y*XODR_DIM
        orientation_list_quaternion = [self.position.orientation.x, self.position.orientation.y, self.position.orientation.z, self.position.orientation.w]
        (roll, peach, self.yaw) = tf.transformations.euler_from_quaternion(orientation_list_quaternion)
        #print (type(self.yaw))
        self.manager()

    def manager(self):
        state = "navigating"
        if ((20 <self.x < 50) and ((65<self.y < 90) or (-10 <self.y <15))):
            print ("in")
            self.intersection_pos=True
            #self.pub_control.publish("end")
            if self.change_angle==True:
                print ("change angle +1 ")
                self.change_angle_counter+=1
                if self.change_angle_counter >= 2:
                    self.change_angle_counter=0
                    print ("change angle")
                    self.pub_control.publish("end")
                    self.enable_turn=False
                    if 0 <=len(self.path_angles) <=1 and self.finish_goal==False:
                        self.finish_goal=True
                        self.calculate_meters()
                        self.encoder_start_end=self.encoder
                        print ("last ticks in",self.last_ticks)
            else:
                self.counter_change_angle=0
                self.enable_turn=True
            self.change_angle=True
        else:
            if self.change_angle_counter>=1:
                self.change_angle_counter=0
            """if len(self.path_angles)==0 and len(self.goal_list)>0 and self.finish_goal==False:
                self.finish_goal=True
                self.calculate_meters()
                self.encoder_start_end=self.encoder
                print ("last ticks",self.last_ticks)"""
            self.intersection_pos=False
            if self.change_angle==True:
                print ("neteja")
                self.change_angle=False
                self.first_entry=True
                del self.path_angles[0]
        if self.finish_goal==True:
            print ("finish: ", abs(self.encoder - self.encoder_start_end))
            if abs(self.encoder - self.encoder_start_end) > self.last_ticks:
                self.finish_goal=False
                if self.type == 0:
                    print ("in type 0")
                    self.next_goal()
                elif self.type==1:
                    self.pub_control.publish("stop")
                    fb = "stoped"
                    self.pub_fb.publish(state)
                    time.sleep(5)
                    self.pub_control.publish("end")
                    self.next_goal()
                elif self.type ==2:
                    self.pub_parking_enable.publish("right")
                    print ("parking enable")

        self.pub_fb.publish(state)
        #print (self.yaw)
        #print (self.position, self.x, self.y, self.yaw)
    def calculate_meters(self):
        self.last_ticks = (self.last_meters * TICKS_PER_METER)- TICKS_PER_METER



# This is made to test some paths
if __name__=="__main__":
    mygraph = get_graph_from_xodr_file('/home/pepe/catkin_ws/src/xodr_bridge/scripts/sample_parking.xodr')
    rospy.init_node('xodr', anonymous=True)
    goal_resolver = goal_resolver(mygraph)
    #time.sleep(1)
    rospy.spin()
    # print(mygraph.find_xodr_connection(Coords(36, 20, -1.57)))
    # print(mygraph.find_xodr_connection(Coords(36, 20, 1.57)))

    # print(mygraph.find_xodr_connection(Coords(-10, 1, 0)))
    # print(mygraph.find_xodr_connection(Coords(45, 30, -1.5)))

    # print(mygraph.from_point_to_point(Coords(-10,1,0), Coords(-10, 1, pi)))
    # print(mygraph.from_point_to_point(Coords(-10,1,0), Coords(45, 30, -1.5)))

    #print(mygraph.from_point_to_point(Coords(10,1,0), Coords(-10, 1, 0)))
    #print("Result of removing a connection: {0}".format(mygraph.can_not_turn(Coords(20, 1, 0), 1.57)))
    #print(mygraph.from_point_to_point(Coords(10,1,0), Coords(-10, 1, 0)))
