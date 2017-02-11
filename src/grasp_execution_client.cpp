#include <ros/ros.h>
#include<actionlib/client/simple_action_client.h>
#include<actionlib/client/terminal_state.h>
#include<grasp_execution/ExecuteGraspAction.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "grasp_execution_client");
  actionlib::SimpleActionClient<grasp_execution::ExecuteGraspAction> gs("grasp_execution_server", true);
  ROS_INFO("Waiting for server to start");
  gs.waitForServer ();

  ROS_INFO("Grasp Execution Action server started, sending grasp");

  //Creating a grasp
  grasp_execution::grasp gr;
  gr.pose.position.x = 0.557;
  gr.pose.position.y = -0.543;
  gr.pose.position.z = 1.083;
  gr.pose.orientation.x = -0.009;
  gr.pose.orientation.y = 0.024;
  gr.pose.orientation.z = 0.027;
  gr.pose.orientation.w = 0.999;

  gr.angle = 0.79; //opening angle of the gripper

  //Best one
  /*grasp_execution::grasp gr;
  gr.pose.position.x = 0.671;
  gr.pose.position.y = -0.441;
  gr.pose.position.z = 1.088;
  gr.pose.orientation.x = 0.667;
  gr.pose.orientation.y = -0.330;
  gr.pose.orientation.z = -0.609;
  gr.pose.orientation.w = -0.275;*/

  //This grasp pose directly on the table
  /*grasp_execution::grasp gr;
  gr.pose.position.x = 0.779;
  gr.pose.position.y = -0.208;
  gr.pose.position.z = 1.058;
  gr.pose.orientation.x = 0.639;
  gr.pose.orientation.y = -0.326;
  gr.pose.orientation.z = -0.616;
  gr.pose.orientation.w = -0.327;*/




  gr.approach.x = 1.0;
  gr.approach.y = 0.0;
  gr.approach.z = -1.0;



  grasp_execution::ExecuteGraspGoal goal;
  goal.grasp = gr;
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
