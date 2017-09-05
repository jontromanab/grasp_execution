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

  ROS_INFO("Grasp Execution Action server started, sending grasp");

  //Creating a grasp
  /*grasp_execution::grasp gr;
  gr.pose.position.x = 0.557;
  gr.pose.position.y = -0.543;
  gr.pose.position.z = 1.083;
  gr.pose.orientation.x = -0.009;
  gr.pose.orientation.y = 0.024;
  gr.pose.orientation.z = 0.027;
  gr.pose.orientation.w = 0.999;

  gr.angle = 0.79; //opening angle of the gripper*/
  ros::ServiceClient client = nh.serviceClient<sq_grasping::getGrasps>("/sq_grasp/grasps");
  sq_grasping::getGrasps srv;
  srv.request.num_of_fingers = 2;
  client.call(srv);
  std::cout<<"Got "<<srv.response.grasps.grasps.size()<<" grasps back"<<std::endl;

  if(srv.response.grasps.grasps.size()==0)
    return 0;

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

  return 0;
}
