#include<grasp_execution/create_gripper_marker.h>

CreateGripperMarker::CreateGripperMarker()
{}

visualization_msgs::Marker CreateGripperMarker::createGripperMeshMarker (double x, double y, double z, double rx, double ry, double rz, double rw, std::string meshLocation, int color)
{

  visualization_msgs::Marker marker;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = rx;
  marker.pose.orientation.y = ry;
  marker.pose.orientation.z = rz;
  marker.pose.orientation.w = rw;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = meshLocation;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  if(color == 0)
  {
    marker.color.r = 0.1;
    marker.color.g = 0.1;
    marker.color.b = 0.1;
  }
  else
  {
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
  }
  marker.color.a = 1.0;
  marker.mesh_use_embedded_materials;
  return marker;
}

geometry_msgs::Pose CreateGripperMarker::rotatePose(const geometry_msgs::Pose pose)
{
  tf2::Transform trns;
  tf2::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Vector3 vec(pose.position.x, pose.position.y, pose.position.z);
  trns.setOrigin (vec);
  trns.setRotation (quat);

  tf2::Quaternion quat2;
  quat2.setRPY( M_PI/2, 0,  0);
  tf2::Vector3 vec2(0, 0 , 0);
  tf2::Transform multiplier;
  //multiplier.setIdentity ();
  multiplier.setOrigin (vec2);
  multiplier.setRotation (quat2);

  trns = trns * multiplier;
  tf2::Vector3 new_pose_vec;
  tf2::Quaternion new_pose_quat;
  new_pose_vec = trns.getOrigin ();
  new_pose_quat = trns.getRotation ();
  new_pose_quat.normalize ();

  geometry_msgs::Pose new_pose;
  new_pose.position.x = new_pose_vec[0];
  new_pose.position.y = new_pose_vec[1];
  new_pose.position.z = new_pose_vec[2];
  new_pose.orientation.x = new_pose_quat[0];
  new_pose.orientation.y = new_pose_quat[1];
  new_pose.orientation.z = new_pose_quat[2];
  new_pose.orientation.w = new_pose_quat[3];
  return new_pose;
}



visualization_msgs::InteractiveMarker CreateGripperMarker::makeGripperMaker (geometry_msgs::Pose pose)
{
  geometry_msgs::Pose new_pose = rotatePose(pose);
  visualization_msgs::InteractiveMarker iMarker;
  iMarker.header.frame_id = "/base_link";
  iMarker.pose = new_pose;
  iMarker.scale = 0.2;
  iMarker.name = "gripper_robotiq";
  iMarker.description = "gripper pose";

  visualization_msgs::Marker gripperBase = createGripperMeshMarker (-0.055, 0, 0, -0.707, 0, 0, 0.707, "package://grasp_execution/meshes/visual/robotiq_85_base_link.dae", 0);
  visualization_msgs::Marker gripperLeftKnuckle = createGripperMeshMarker(-0.001, 0, -0.031, 0.707, 0, 0, 0.707, "package://grasp_execution/meshes/visual/robotiq_85_knuckle_link.dae",1);
  visualization_msgs::Marker gripperRightKnuckle = createGripperMeshMarker(-0.001, 0, 0.031,-0.707, 0, 0, 0.707,"package://grasp_execution/meshes/visual/robotiq_85_knuckle_link.dae",1);
  visualization_msgs::Marker gripperLeftFinger = createGripperMeshMarker(-0.005, 0, -0.062, 0.707, 0, 0, 0.707,"package://grasp_execution/meshes/visual/robotiq_85_finger_link.dae",0);
  visualization_msgs::Marker gripperRightFinger = createGripperMeshMarker(-0.005, 0, 0.062, -0.707, 0, 0, 0.707,"package://grasp_execution/meshes/visual/robotiq_85_finger_link.dae",0);
  visualization_msgs::Marker gripperLeftInnerKnuckle = createGripperMeshMarker(0.006, 0, -0.013, 0.707, 0, 0, 0.707,"package://grasp_execution/meshes/visual/robotiq_85_inner_knuckle_link.dae",0);
  visualization_msgs::Marker gripperRightInnerKnuckle = createGripperMeshMarker(0.006, 0, 0.013,-0.707, 0, 0, 0.707, "package://grasp_execution/meshes/visual/robotiq_85_inner_knuckle_link.dae",0);
  visualization_msgs::Marker gripperLeftFingerTip = createGripperMeshMarker(0.049, 0, -0.05,.707, 0, 0, 0.707,"package://grasp_execution/meshes/visual/robotiq_85_finger_tip_link.dae",1);
  visualization_msgs::Marker gripperRightFingerTip = createGripperMeshMarker(0.049, 0, 0.05,-0.707, 0, 0, 0.707,"package://grasp_execution/meshes/visual/robotiq_85_finger_tip_link.dae",1);

  visualization_msgs::InteractiveMarkerControl gripperControl;
  gripperControl.markers.push_back (gripperBase);
  gripperControl.markers.push_back (gripperLeftKnuckle);
  gripperControl.markers.push_back (gripperRightKnuckle);
  gripperControl.markers.push_back (gripperLeftFinger);
  gripperControl.markers.push_back (gripperRightFinger);
  gripperControl.markers.push_back (gripperLeftInnerKnuckle);
  gripperControl.markers.push_back (gripperRightInnerKnuckle);
  gripperControl.markers.push_back (gripperLeftFingerTip);
  gripperControl.markers.push_back (gripperRightFingerTip);
  gripperControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  gripperControl.always_visible = true;

  iMarker.controls.push_back (gripperControl);
  return iMarker;
}
