#include <ros/ros.h>
#include<actionlib/client/simple_action_client.h>
#include<actionlib/client/terminal_state.h>
#include<grasp_execution/ExecuteGraspAction.h>
#include<sq_grasping/getGrasps.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  
  ros::init(argc, argv, "grasp_execution_client");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<grasp_execution::ExecuteGraspAction> gs("grasp_execution_server", true);
  ROS_INFO("Waiting for server to start");
  gs.waitForServer ();

  ROS_INFO("Grasp Execution Action server started");

  ros::ServiceClient client = nh.serviceClient<sq_grasping::getGrasps>("/sq_grasp/grasps");
  sq_grasping::getGrasps srv;
  srv.request.num_of_fingers = 2;

  int num_of_objects = 10;
  while(num_of_objects>0)
  {
    client.call(srv);
    std::cout<<"Got "<<srv.response.grasps.grasps.size()<<" grasps back"<<std::endl;
    num_of_objects = srv.response.grasps.grasps.size();
    grasp_execution::ExecuteGraspGoal goal;
    goal.grasps = srv.response.grasps;
    gs.sendGoal (goal);
    bool finished_before_timeout = gs.waitForResult (ros::Duration(40.0));
    if(finished_before_timeout )
    {
      actionlib::SimpleClientGoalState state = gs.getState ();
      ROS_INFO("Action finished: %s", state.toString ().c_str ());
    }
    else
      ROS_INFO("Action did not finish before time out");
   }

  ROS_INFO("I am done");
  return 0;
}
