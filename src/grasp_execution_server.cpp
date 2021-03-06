#include <ros/ros.h>
#include<actionlib/server/simple_action_server.h>
#include<grasp_execution/ExecuteGraspAction.h>


#include <grasp_execution/grasp_execution.h>
#include<visualization_msgs/Marker.h>




class ExecuteGraspAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<grasp_execution::ExecuteGraspAction> gs_;
  std::string action_name_;
  grasp_execution::ExecuteGraspFeedback feedback_;
  grasp_execution::ExecuteGraspResult result_;

public:
  ExecuteGraspAction(std::string name):
    gs_(nh_,name, boost::bind(&ExecuteGraspAction::executeCB, this, _1), false),
    action_name_(name)
{
  gs_.start ();
}

~ExecuteGraspAction(void)
{

}

void executeCB(const grasp_execution::ExecuteGraspGoalConstPtr &goal)
{
  ros::Rate r(1);
  bool success = true;
  ROS_INFO("%s: Executing grasp ", action_name_.c_str ());
  ROS_INFO("Grasp pose: position.x = %f, position.y = %f, position.z = %f", goal->grasps.grasps[0].pose.position.x, goal->grasps.grasps[0].pose.position.y, goal->grasps.grasps[0].pose.position.z );
  ROS_INFO("Orientation.x = %f, Orientation.y = %f, Orientation.z = %f, Orientation.w = %f", goal->grasps.grasps[0].pose.orientation.x, goal->grasps.grasps[0].pose.orientation.y, goal->grasps.grasps[0].pose.orientation.z,
           goal->grasps.grasps[0].pose.orientation.w);


  grasp_execution::GraspExecution execute(nh_, goal->grasps);
  execute.goToGrasp();
  execute.pickUp();
  execute.place();



  feedback_.pose = execute.getCurrentPose();
  gs_.publishFeedback (feedback_);

  if(success)
  {
    result_.success = true;
    ROS_INFO("%s: succeded", action_name_.c_str ());
    gs_.setSucceeded ();
  }

}



};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "grasp_execution_server");
  ExecuteGraspAction ex("grasp_execution_server");

  ros::spin ();
  return 0;
}
