#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>

using namespace std;

#define MIN_VALUE_DESV 110
#define MAX_VALUE_DESV 125
#define FRONT_OBSTACLE_TURN 10

geometry_msgs::Twist tx_msg;
std_msgs::String alignment;
struct stop_turn{
  int right;
  int left;
};

struct parking_orders{
  int right_front;
  int left_front;
  int left_back;
  int right_back;
  int front;
  int back;
  int stop;
  int align_info;
  int right_follow;
};

stop_turn stop_obstacle = {0,0};
parking_orders park = {0,0,0,0,0,0,0,0,0};
string right_line_type;
string left_line_type;


void direction( std::string from, float value){
  static std::string turning_from;
  static float angular_z;
  static int turn_only_wheel;

  //Inicializamos velocidad lineal a 5
  //tx_msg.linear.x = 1;
  tx_msg.linear.x = 5;

  //Parking at right or left
  if (park.stop == 1){
      turn_only_wheel=0;
      tx_msg.linear.x = 0;
      park = {0,0,0,0,0,0,0,0};
      park.stop=1;
  }
  else if (park.front == 1){
    angular_z=0;
    }
  else if (park.back == 1){
    angular_z=0;
    tx_msg.linear.x = -5;
    }
  else if (park.right_front == 1){
    if (turn_only_wheel<200){
      tx_msg.linear.x=0.1;
      turn_only_wheel+=1;
      }
    angular_z=-7;
    }
  else if (park.left_front == 1){
    if (turn_only_wheel<200){
      tx_msg.linear.x=0.1;
      turn_only_wheel+=1;
      }
    angular_z=7;
    }
  else if (park.left_back == 1){
    if (turn_only_wheel<200){
      tx_msg.linear.x=-0.2;
      turn_only_wheel+=1;
      }
      else{
        tx_msg.linear.x = -1;
      }
     angular_z=-7;
  }
  else if (park.right_back == 1){
    if (turn_only_wheel<200){
      tx_msg.linear.x=-0.2;
      turn_only_wheel+=1;
      }
      else{
        tx_msg.linear.x = -1;
      }
    angular_z=7;
  }
  else if (park.right_follow == 1){
    if (abs(value)>MAX_VALUE_DESV && value > 0){
      if (from=="left_line"){
	       angular_z= (value- MAX_VALUE_DESV)*0.2;
      }
      //girem a la dreta = valor negatiu
      else if (from=="right_line"){
	       angular_z= (value - MAX_VALUE_DESV)*-0.2;
      }
    }
  }
  //OBSTACLE DETECTION CHANGE VIA
  else if (stop_obstacle.right == 1){
    if (right_line_type=="disc"){
      angular_z=-FRONT_OBSTACLE_TURN;
    }
    else if (right_line_type == "cont"){
      stop_obstacle.right=0;
    }
  }
  else if (stop_obstacle.left == 1){
    if (left_line_type=="disc"){
      angular_z=FRONT_OBSTACLE_TURN;
    }
    else if (left_line_type == "cont"){
      stop_obstacle.left=0;
    }
  }

  //Path following using camera sensors
  else{
    //girem a l'esquerra = valor positiu
    if (abs(value)<MIN_VALUE_DESV && value > 0){
      if (from=="left_line"){
	       angular_z= (MIN_VALUE_DESV-value)*-0.2;
	       turning_from=from;
         alignment.data="left";
      }
      //girem a la dreta = valor negatiu
      else if (from=="right_line"){
	       angular_z= (MIN_VALUE_DESV-value)*0.2;
	       turning_from=from;
         alignment.data="right";
      }
    }
    else if (value < 0 ){
      if (from=="left_line"){
	       angular_z= 50*0.2;
	       turning_from=from;
         alignment.data="left";
      }
      else if (from=="right_line"){
	       angular_z= 50*- 0.2;
	       turning_from=from;
         alignment.data="right";
      }
    }
    else if (from == turning_from){
      alignment.data="center";
      angular_z=0;
    }

  }
  if (park.align_info==1){
    if (abs(value)<MIN_VALUE_DESV && value > 0){
      if (from=="left_line"){
         alignment.data="left";
      }
      //girem a la dreta = valor negatiu
      else if (from=="right_line"){
         alignment.data="right";
      }
    }
    else if (value < 0 ){
      if (from=="left_line"){
         alignment.data="left";
      }
      else if (from=="right_line"){
         alignment.data="right";
      }
    }
    else if (from == turning_from){
      alignment.data="center";
    }
  }

  tx_msg.angular.z = angular_z;

}



void front_left_desv_Callback( const std_msgs::Float64 rx_msg){
  direction("left_line",rx_msg.data);
}

void front_right_desv_Callback( const std_msgs::Float64 rx_msg){
  direction("right_line", rx_msg.data);
}

void right_line_type_Callback( std_msgs::String rx_msg){
  right_line_type=rx_msg.data;
}

void left_line_type_Callback( std_msgs::String rx_msg){
  left_line_type=rx_msg.data;
}


void front_obstacle_turn_Callback( const std_msgs::String turn){
  if (turn.data=="right"){
    stop_obstacle.right=1;
  }
  else if (turn.data=="left"){
    stop_obstacle.left=1;
  }
}

void parking_Callback( const std_msgs::String parking_turn){
  if (parking_turn.data=="right_front"){
    park.stop=0;
    park.right_front=1;
  }
  else if (parking_turn.data=="left_front"){
    park.stop=0;
    park.left_front=1;
  }
  else if (parking_turn.data=="left_back"){
    park.stop=0;
    park.left_back=1;
  }

  else if (parking_turn.data=="right_back"){
    park.stop=0;
    park.right_back=1;
  }
  else if (parking_turn.data=="front"){
    park.stop=0;
    park.front=1;
  }
  else if (parking_turn.data=="back"){
    park.stop=0;
    park.back=1;
  }
  else if (parking_turn.data=="alignment_info"){
    park.stop=0;
    park.align_info=1;
  }
  else if (parking_turn.data=="right_follow"){
    park.stop=0;
    park.right_follow=1;
  }
  else if (parking_turn.data=="stop"){
    park.stop=1;
  }
  else if (parking_turn.data=="end"){
    park = {0,0,0,0,0,0,0,0};
  }
  direction("",0);
}



int main(int argc, char **argv)
{

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "control");

  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/adc_car/cmd_vel", 1000);
  ros::Publisher align_pub = n.advertise<std_msgs::String>("/control/alignment", 1000);

  tx_msg.linear.x = 0;
  tx_msg.angular.z = 0;

  ros::Subscriber sub_left_desv = n.subscribe("/control/front_left_desviation",10,front_left_desv_Callback);
  ros::Subscriber sub_right_desv = n.subscribe("/control/front_right_desviation",10,front_right_desv_Callback);
  ros::Subscriber sub_left_line = n.subscribe("/vision/left_line_type",10,left_line_type_Callback);
  ros::Subscriber sub_right_line = n.subscribe("/vision/right_line_type",10,right_line_type_Callback);
  ros::Subscriber sub_turn_obstacle = n.subscribe("/laser/front_obstacle_turn",10,front_obstacle_turn_Callback);
  ros::Subscriber sub_parking = n.subscribe("/laser/park",10,parking_Callback);


  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    /*
    if (tx_msg.linear.x!=0){
      ROS_INFO("%s", "hel");

      chatter_pub.publish(tx_msg);

    }
    */
    chatter_pub.publish(tx_msg);
    align_pub.publish(alignment);
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
