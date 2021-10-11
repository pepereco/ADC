#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <jury_server/adc_juryAction.h>
#include <iri_adc_msgs/adc_goal.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/String.h"
//using namespace std;
//using std::string;
//#include <string>

//protoptips
void fb_cb(std::string state);

// Este archivo es el usado para interactuar con el IRI_ADC_JURY.
// De momento obtiene todas las variables necesarias para poder procesarlas posteriormente.
// Tambien publica un feedback (de momento por defecto), y un resultado una vez acabada la mision (de momento inexistente).

class juryServerAction
{
private:
  uint32_t seq_id = 0;

  jury_server::adc_juryFeedback getDefaultFeedback(){
      // Esta funcion nos da un feedback constante para poder hacer pruebas.
      jury_server::adc_juryFeedback fb;
      fb.status = "idle";

      fb.base_position.header.seq = seq_id;
      seq_id++;
      fb.base_position.header.stamp = ros::Time::now();
      fb.base_position.header.frame_id = "CarPose";

      fb.base_position.pose.position.x = 0.0;
      fb.base_position.pose.position.y = 0.0;
      fb.base_position.pose.position.z = 0.0;

      fb.base_position.pose.orientation.x = 0.0;
      fb.base_position.pose.orientation.y = 0.0;
      fb.base_position.pose.orientation.z = 0.0;
      fb.base_position.pose.orientation.w = 0.0;

      return fb;
  }
protected:
    ros::NodeHandle nh_;
    //ros::Subscriber sub_pose = nh_.subscribe("/adc_car/amcl_pose",10,pose_cb);
    actionlib::SimpleActionServer<jury_server::adc_juryAction> as_;
    // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // creamos mensages para publicar feedback/result
    geometry_msgs::PoseStamped pose_;
    jury_server::adc_juryFeedback feedback_= juryServerAction::getDefaultFeedback();
    jury_server::adc_juryResult result_;
    ros::Subscriber sub_fb;
    ros::Subscriber sub_pos;
    ros::Publisher pub_goal;
    bool goals_seq_done=false;

public:
    juryServerAction(std::string name) :
        as_(nh_, name, boost::bind(&juryServerAction::executeCB, this, _1), false),
        action_name_(name)
    {
      pub_goal = nh_.advertise<iri_adc_msgs::adc_goal>("/adc_car/goal", 1000);
      sub_fb = nh_.subscribe("/adc_car/fb",10,&juryServerAction::fb_cb, this);
      sub_pos = nh_.subscribe("/adc_car/amcl_pose",10,&juryServerAction::pose_cb, this);
        as_.start();
        // mostrar informacion para debug.
        ROS_INFO("%s: Server started.", action_name_.c_str());
    }

    ~juryServerAction(void)
    {
    }

    void executeCB(const jury_server::adc_juryGoalConstPtr &goal)
    {
        // obtener las variables que nos han pasado.
        jury_server::adc_goal_array allGoals = goal->goals;
        int numGoals = end(allGoals.goals) - begin(allGoals.goals);
        bool success = true;

        // publish info to the console for the user
        ROS_INFO("%s: Executing, received %i goal(s).", action_name_.c_str(), numGoals);

        // publish info for each single goal.
        for ( int i = 0; i<numGoals; i++) {
          if (as_.isPreemptRequested() || !ros::ok())
          {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            break;
          }

          jury_server::adc_goal thisGoal = allGoals.goals[i];
          ROS_INFO("Goal %i: ID: %s, type: %i, x: %f, y: %f, yaw: %f.", i, thisGoal.id.c_str(), thisGoal.type, thisGoal.x, thisGoal.y, thisGoal.yaw);

          pub_goal.publish(thisGoal);
          // publish default feedback.
          //as_.publishFeedback(juryServerAction::getDefaultFeedback());

        }
        while (goals_seq_done==false){
        }
          // set the action state to succeeded
        as_.setSucceeded(result_);
    }
    void pose_cb(geometry_msgs::PoseWithCovarianceStamped pose)
    {
      feedback_.base_position.pose = pose.pose.pose;
      //geometry_msgs::PoseStamped pose_ = pose;
    }
    //void fb_cb(std::string state)
    void fb_cb(std_msgs::String state)
    {
      if (state.data == "finished"){
         goals_seq_done= true;
      }
      else{
        goals_seq_done=false;
      }
      //jury_server::adc_juryFeedback fb = juryServerAction::getDefaultFeedback();
      feedback_.status = state.data;
      as_.publishFeedback(feedback_);
    }
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "adc_jury_server");

    juryServerAction juryServer("iri_adc_jury/adc_jury");
    ros::spin();

    return 0;
}
