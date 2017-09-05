#ifndef GRASP_EXECUTION_H
#define GRASP_EXECUTION_H
#include <iostream>
#include<ros/ros.h>

#include <grasp_execution/grasp.h>
#include<grasp_execution/graspArr.h>
#include <visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/InteractiveMarker.h>
#include<interactive_markers/interactive_marker_server.h>

#include<moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include<grasp_execution/create_gripper_marker.h>
#include<pr2_controllers_msgs/Pr2GripperCommand.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>



namespace grasp_execution {

class GraspExecution{

public:
  GraspExecution(ros::NodeHandle& node, grasp_execution::graspArr grasps);
  //~GraspExecution();


  geometry_msgs::Pose getCurrentPose(); //Returns current pose of the end-effector, needed in action feedback


  //main grasp execution pipeline
  bool goToGrasp();  //Going to grasp position and close gripper
  bool pickUp(); //Pick up the object
  bool place(); //Place it and come back to joints prenominal




private:
  grasp_execution::grasp grasp_;
  visualization_msgs::Marker approachMarker_;
  ros::Publisher approach_pub_;
  ros::Publisher grasp_waypoints_pub_;
  ros::Publisher waypoints_pub_;
  ros::NodeHandle nh_;
  std::vector<geometry_msgs::Pose> grasp_wayPoints_;
  std::vector<geometry_msgs::Pose> pick_wayPoints_;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer;

  //ros::Publisher gripper_pub_;
  std::string frame_id_;


  ros::AsyncSpinner spinner;
  moveit::planning_interface::MoveGroup group;
  moveit::planning_interface::MoveGroup group2;

  void publishApproachMarker(); //Publishing the approach marker ->Arrow
  void publishGripperMarker(); //Publishing gripper marker
  void publishPickWayPointsMarker(); //Publishing pick up point markers ->blue cube
  void publishGraspWayPointsMarker();//Publishing grasp way point markers ->yellow sphere


  visualization_msgs::Marker createApproachMarker(grasp_execution::grasp grasp); //Creating approach markers
  visualization_msgs::MarkerArray createWayPointMarkers(std::vector<geometry_msgs::Pose> waypoints, int type); //Creating picking up way points markerArray
  geometry_msgs::Pose createPickPose(); //Creating pick up pose.needed when cartesian fails
  bool generateWayPointsGrasp(grasp_execution::grasp grasp);//Creating grasp waypoints
  bool createExtraWayPoints(); //Creating middle waypoint between last waypoint and current gripper pose
  bool generatePickWayPoints(); //Creating pick way points


  bool publishGripperMarkerMoveit(); //Visualizing gripper with joint value



  bool executePickPose(); //Executing pick pose
  bool executePoseGrasp(grasp_execution::grasp grasp); //Executing grasp pose.Needed if cartesian fails
  double executeWayPoints(std::vector<geometry_msgs::Pose> waypoints); //Executing provided waypoints(cartesian execution)
  bool executeJointTarget(float joint_values[]); //Executing joint targets
  bool executeJointTargetNominal(); //Executing joint target nominal
  bool executeJointTargetPreNominal();//Executing joint target prenominal
  bool executeJointTargetPlace();//Executing joint target place
  bool openGripper(); //Opening the gripper
  bool closeGripper(); //Closing the gripper


  //New Functions
  boost::scoped_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  moveit_msgs::CollisionObject collision_object_;
  ros::Publisher gripper_pub_;
  geometry_msgs::Vector3 table_center_;
  void createTableCollisionObject();
  void removeTableCollisionObject();
  void openPR2Gripper();
  void closePR2Gripper();
  void closePR2GripperByValue(double angle);
  void moveToHome();
  void moveToApproach();
  void generateGraspWayPoints(std::vector<geometry_msgs::Pose>& way);
  void moveToRetreat();
  void moveToDrop();




};



}

#endif // GRASP_EXECUTION_H
