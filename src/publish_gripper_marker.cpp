#include <ros/ros.h>
#include<geometry_msgs/PoseArray.h>
#include<geometry_msgs/PoseStamped.h>
#include<visualization_msgs/InteractiveMarker.h>
#include<visualization_msgs/MarkerArray.h>
#include<interactive_markers/interactive_marker_server.h>
#include<tf2/LinearMath/Transform.h>
#include<tf2/LinearMath/Quaternion.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include<moveit/move_group_interface/move_group.h>
#include<sq_fitting/sq.h>
#include<sq_fitting/sqArray.h>
#include<tf/transform_listener.h>

void sq_create_transform(const geometry_msgs::Pose& pose, Eigen::Affine3f& transform)
{
  transform = Eigen::Affine3f::Identity();
  Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  q.normalize();
  transform.translation()<<pose.position.x, pose.position.y, pose.position.z;
  transform.rotate(q);
}

void transformFrame(const geometry_msgs::Pose &pose_in, geometry_msgs::Pose& pose_out)
{
  //std::cout<<"Output_frame: "<<output_frame_<<std::endl;
  //std::cout<<"Input frame: "<<this->input_msg_.header.frame_id<<std::endl;
  

  tf::TransformListener listener;
  tf::StampedTransform transform;
  try{

    listener.waitForTransform( "/base_link","/odom_combined",ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform("/base_link","/odom_combined",ros::Time(0), transform);
    //std::cout<<"Transform: "<<transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<" "<<transform.getOrigin().z()<<std::endl;

    geometry_msgs::Pose inter_pose;
    inter_pose.position.x = transform.getOrigin().x();
    inter_pose.position.y = transform.getOrigin().y();
    inter_pose.position.z = transform.getOrigin().z();
    inter_pose.orientation.x = transform.getRotation().getX();
    inter_pose.orientation.y = transform.getRotation().getY();
    inter_pose.orientation.z = transform.getRotation().getZ();
    inter_pose.orientation.w = transform.getRotation().getW();
    Eigen::Affine3d transform_in_eigen;
    tf::poseMsgToEigen(inter_pose, transform_in_eigen);

    Eigen::Affine3f pose_in_eigen;
    sq_create_transform(pose_in, pose_in_eigen);

    tf::poseEigenToMsg( transform_in_eigen * pose_in_eigen.cast<double>(), pose_out );


  }
  catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

  }


/*visualization_msgs::InteractiveMarker makeGripperMarker( const char *name, const geometry_msgs::PoseStamped &stamped,
                                                         float scale, float angle, bool view_facing, std_msgs::ColorRGBA color, bool use_color )
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = 1.0; //scale;
  int_marker.pose = stamped.pose;

//  add6DofControl(int_marker, false);

  visualization_msgs::InteractiveMarkerControl control;

  visualization_msgs::Marker mesh;
  mesh.mesh_use_embedded_materials = !use_color;
  mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
  mesh.scale.x = scale;
  mesh.scale.y = scale;
  mesh.scale.z = scale;
  mesh.color = color;

  tf::Transform T1, T2;
  tf::Transform T_proximal, T_distal;

  T1.setOrigin(tf::Vector3(0.07691, 0.01, 0));
  T1.setRotation(tf::Quaternion(tf::Vector3(0,0,1),  angle));
  T2.setOrigin(tf::Vector3(0.09137, 0.00495, 0));
  T2.setRotation(tf::Quaternion(tf::Vector3(0,0,1), -angle));
  T_proximal = T1;
  T_distal = T1 * T2;

  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/gripper_palm.dae";
  mesh.pose.orientation.w = 1;
  control.markers.push_back( mesh );
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger.dae";
  mesh.pose = object_manipulator::msg::createPoseMsg(T_proximal);
  control.markers.push_back( mesh );
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae";
  mesh.pose = object_manipulator::msg::createPoseMsg(T_distal);
  control.markers.push_back( mesh );

  T1.setOrigin(tf::Vector3(0.07691, -0.01, 0));
  T1.setRotation(tf::Quaternion(tf::Vector3(1,0,0), M_PI)*tf::Quaternion(tf::Vector3(0,0,1),  angle));
  T2.setOrigin(tf::Vector3(0.09137, 0.00495, 0));
  T2.setRotation(tf::Quaternion(tf::Vector3(0,0,1), -angle));
  T_proximal = T1;
  T_distal = T1 * T2;

  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger.dae";
  mesh.pose = object_manipulator::msg::createPoseMsg(T_proximal);
  control.markers.push_back( mesh );
  mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae";
  mesh.pose = object_manipulator::msg::createPoseMsg(T_distal);
  control.markers.push_back( mesh );

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  int_marker.controls.push_back( control );

  return int_marker;
}*/

visualization_msgs::Marker createGripperMeshMarker (double x, double y, double z, double rx, double ry, double rz, double rw, std::string meshLocation, int color)
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

Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * ry * rx;
}


geometry_msgs::Pose rotatePoseNew(const geometry_msgs::Pose pose, double value, int dir, bool negative)
{
  Eigen::Affine3d pose_in_eigen;
  tf::poseMsgToEigen(pose, pose_in_eigen);
  Eigen::Affine3d pose_inv = pose_in_eigen.inverse();
  Eigen::Affine3d pose_in_center = pose_in_eigen * pose_inv;

  Eigen::Affine3d transformation_mat;
  int ve_value = -1;
  if(negative)
    ve_value = 1;
  if(dir==1)
    transformation_mat = create_rotation_matrix(value, 0, 0);
  if(dir==2)
    transformation_mat = create_rotation_matrix(0, value, 0);
  if(dir==3)
    transformation_mat = create_rotation_matrix(0, 0, value);

  Eigen::Affine3d new_trans = pose_in_center* transformation_mat;
  Eigen::Affine3d back_to_place = new_trans* pose_in_eigen;
  geometry_msgs::Pose trans_pose;
  tf::poseEigenToMsg(back_to_place, trans_pose);
  return trans_pose;
}



geometry_msgs::Pose rotatePose(const geometry_msgs::Pose pose, double value, int dir, bool negative)
{
  tf2::Transform trns;
  tf2::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Vector3 vec(pose.position.x, pose.position.y, pose.position.z);
  trns.setOrigin (vec);
  trns.setRotation (quat);

  tf2::Quaternion quat2;

  int ve_value = -1;
  if(negative)
    ve_value = 1;

  if(dir==1)
    quat2.setRPY(ve_value*value, 0,  0);
  if(dir==2)
    quat2.setRPY(0,ve_value*value, 0);
  if(dir==3)
    quat2.setRPY(0,0,ve_value*value);
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


visualization_msgs::InteractiveMarker makeGripperMaker (geometry_msgs::Pose pose)
{
  geometry_msgs::Pose new_pose = rotatePose(pose,M_PI/2,1, true);
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

class PublishMarker
{
public:
  PublishMarker(ros::NodeHandle& nh, const std::string& sq_topic);
  ~PublishMarker();
  void runNode();
private:
  void sqCallback(const sq_fitting::sqArray& msg);
  visualization_msgs::MarkerArray createGripperMarkerMoveit(geometry_msgs::Pose pose, const std::vector<double> colors, int id);
  std::vector<double> createColor(const double r, const double g, const double b, const double a);
  void createXPoses(const sq_fitting::sq& sq, std::vector<geometry_msgs::Pose>& poses);
  void createYPoses(const sq_fitting::sq& sq, std::vector<geometry_msgs::Pose>& poses);
  void createBottomZPose(const sq_fitting::sq& sq, geometry_msgs::Pose& pose);
  void createTopZPose(const sq_fitting::sq& sq, geometry_msgs::Pose& pose);
  void createTopYPose(const sq_fitting::sq &sq, geometry_msgs::Pose &pose);
  void createZTiltedPoses(const sq_fitting::sq& sq, std::vector<geometry_msgs::Pose>& poses);
  void createYTiltedPosesPos(const sq_fitting::sq& sq, std::vector<geometry_msgs::Pose>& poses);
  void createYTiltedPosesNeg(const sq_fitting::sq& sq, std::vector<geometry_msgs::Pose>& poses);

  visualization_msgs::Marker createApproachMarker(geometry_msgs::Pose pose);
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer;


  ros::Subscriber sq_sub_;
  ros::Publisher pub_arrow_;
  ros::Publisher gripper_pub_ ;
  ros::Publisher approach_arrow_pub_;
  ros::NodeHandle nh_;
  geometry_msgs::Pose pose_;
  sq_fitting::sq sq_;
  geometry_msgs::PoseStamped pose_st_;
  ros::AsyncSpinner spinner;
  moveit::planning_interface::MoveGroup group;

};


PublishMarker::PublishMarker(ros::NodeHandle &nh, const std::string &sq_topic):spinner(1), group("left_gripper")
{
  std::cout<<"WE ARE HERE"<<std::endl;
  sq_sub_ = nh_.subscribe(sq_topic, 10, &PublishMarker::sqCallback, this);
  //pub_arrow_ = nh_.advertise<geometry_msgs::PoseStamped>("arrow",10);
  gripper_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("final_grasp",10);
  approach_arrow_pub_ = nh_.advertise<visualization_msgs::Marker>("final_arrow",10);
  std::cout<<"Started getting poses"<<std::endl;
}

PublishMarker::~PublishMarker(){}

std::vector<double> PublishMarker::createColor(const double r, const double g, const double b, const double a)
{
  std::vector<double> colors;
  colors.push_back(r);
  colors.push_back(g);
  colors.push_back(b);
  colors.push_back(a);
  return colors;
}

void PublishMarker::createXPoses(const sq_fitting::sq &sq, std::vector<geometry_msgs::Pose> &poses)
{
  geometry_msgs::Pose first_pose = rotatePose(sq.pose, M_PI,3, true);
  //first_pose.position.x = sq.pose.position.x - sq.a1*2.6;
  poses.push_back(first_pose);


  geometry_msgs::Pose second_pose;
  second_pose.orientation = sq.pose.orientation;
  second_pose.position = sq.pose.position;
  second_pose.position.x = sq.pose.position.x +  sq.a1*2.6;
  poses.push_back(second_pose);


}

void PublishMarker::createYPoses(const sq_fitting::sq &sq, std::vector<geometry_msgs::Pose> &poses)
{
  geometry_msgs::Pose first_pose = rotatePose(sq.pose, M_PI/2,3, true);
  geometry_msgs::Pose second_pose = rotatePose(sq.pose, M_PI/2,3,false);
  first_pose.position.y = sq.pose.position.y + sq.a2*2.6;
  second_pose.position.y = sq.pose.position.y - sq.a2*2.6;
  poses.push_back(first_pose);
  poses.push_back(second_pose);
}

void PublishMarker::createYTiltedPosesPos(const sq_fitting::sq &sq, std::vector<geometry_msgs::Pose> &poses)
{
  geometry_msgs::Pose main_pose = rotatePose(sq.pose, M_PI/2,3, true);
  main_pose.position.y = main_pose.position.y + sq.a2*2.6;
  for(int i=-2;i<2;++i)
  {
    geometry_msgs::Pose new_pose = main_pose;
    new_pose.position.z = new_pose.position.z+ (i*0.03);
    geometry_msgs::Pose new_pose2 = rotatePose(new_pose, i*0.02*6,2, true);
    poses.push_back(new_pose2);
  }
}

void PublishMarker::createYTiltedPosesNeg(const sq_fitting::sq &sq, std::vector<geometry_msgs::Pose> &poses)
{
  geometry_msgs::Pose main_pose = rotatePose(sq.pose, M_PI/2,3, false);
  main_pose.position.y = main_pose.position.y - sq.a2*2.6;
  for(int i=-2;i<2;++i)
  {
    geometry_msgs::Pose new_pose = main_pose;
    new_pose.position.z = new_pose.position.z+ (i*0.03);
    geometry_msgs::Pose new_pose2 = rotatePose(new_pose, i*0.02*6,2, true);
    poses.push_back(new_pose2);
  }
}

void PublishMarker::createBottomZPose(const sq_fitting::sq &sq, geometry_msgs::Pose &pose)
{
  geometry_msgs::Pose int_pose = rotatePose(sq.pose, M_PI/2,2, false);
  pose = rotatePose(int_pose, M_PI/2,1, true);
  pose.position.z = pose.position.z - sq.a3 * 2;
  
}

void PublishMarker::createTopZPose(const sq_fitting::sq &sq, geometry_msgs::Pose &pose)
{
   pose =rotatePose(sq.pose, M_PI/2,2, true);
   //pose = rotatePose(int_pose, M_PI/2,1, false);
   pose.position.z = pose.position.z + sq.a3 * 7 ;

}

void PublishMarker::createTopYPose(const sq_fitting::sq &sq, geometry_msgs::Pose &pose)
{
   pose =rotatePose(sq.pose, M_PI/2,3, false);
   //pose = rotatePose(int_pose, M_PI/2,1, false);
   pose.position.y = pose.position.y + sq.a2*4.7 ;
   //pose.position.x = pose.position.x + sq.a1/2  ;
}

void PublishMarker::createZTiltedPoses(const sq_fitting::sq &sq, std::vector<geometry_msgs::Pose> &poses)
{
  geometry_msgs::Pose main_pose = rotatePose(sq.pose, M_PI/2,2, true);
  geometry_msgs::Pose pose = rotatePose(main_pose, M_PI/2,1, true);
  pose.position.z = pose.position.z + sq.a3 * 2.7;
  poses.push_back(pose);
  for(int i=-5;i<5;++i)
  {
    geometry_msgs::Pose new_pose = pose;
    new_pose.position.y = new_pose.position.y+ (i*0.03);
    geometry_msgs::Pose new_pose2 = rotatePose(new_pose, i*0.02*6,2, true);
    poses.push_back(new_pose2);
  }
}

visualization_msgs::Marker PublishMarker::createApproachMarker(geometry_msgs::Pose pose)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = "/odom_combined";
  marker.header.stamp = ros::Time::now ();
  marker.lifetime = ros::Duration(30.0);
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = "basic_shape";
  marker.id = 0;
  marker.scale.x = 0.015;
  marker.scale.y = 0.015;
  marker.scale.z = 0.015;
  marker.color.r =  0.0f;
  marker.color.g =  1.0f;
  marker.color.b =  1.0f;
  marker.color.a =  1.0f;
  //marker.pose = pose;
  //marker.pose.position.z = marker.pose.position.z+0.02;
  geometry_msgs::Point p,q;
  p.x = pose.position.x;
  p.y = pose.position.y;
  p.z = pose.position.z;
  q.x = pose.position.x;
  q.y = pose.position.y + 0.2;
  q.z = pose.position.z ;
  marker.points.push_back (q);
  marker.points.push_back (p);
  return marker;
}


void PublishMarker::sqCallback(const sq_fitting::sqArray& msg)
{
  std::cout<<"I received "<<msg.sqs.size()<<" supers"<<std::endl;
  sq_ = msg.sqs[0];

  

  ROS_INFO("Publishing gripper marker");
  visualization_msgs::MarkerArray markARR;
  std::vector<double> red_color = createColor(1.0, 0.0, 0.0, 0.7);
  std::vector<double> yellow_color = createColor(1.0, 0.5, 0.0, 1);
  std::vector<double> yellow_light_color = createColor(1.0, 0.8, 0.0, 0.6);
  std::vector<double> green_color = createColor(0,1,0, 1);

  std::vector<geometry_msgs::Pose> x_poses;
  createXPoses(sq_, x_poses);

  geometry_msgs::Pose final_pose;
  sq_fitting::sq sq_new = sq_;
 // std::cout<<"Value of z"<<sq_new.pose.position.z<<std::endl;
  //sq_new.pose.position.z =  sq_.pose.position.z + sq_.a3*2.7;
  //std::cout<<"Value of z NOW:"<<sq_new.pose.position.z<<std::endl;
  createTopZPose(sq_, final_pose);
  //createTopYPose(sq_, final_pose);

  std::cout<<"Value of pose: "<<final_pose.position.x<<" "<<final_pose.position.y<<" "<<final_pose.position.z;
  std::cout<<" "<<final_pose.orientation.x<<" "<<final_pose.orientation.y<<" "<<final_pose.orientation.z<<" "<<final_pose.orientation.w<<std::endl;

  //geometry_msgs::Pose z_pos_pose;
  //z_pos_pose.position.x =  1;//sq_.pose.position.x;
 // z_pos_pose.position.y = 0;
  //z_pos_pose.position.z = 1;
 // z_pos_pose.orientation.w = 0;

  imServer.reset(new interactive_markers::InteractiveMarkerServer("gripper", "gripper", false));
  ros::Duration(0.1).sleep();
  imServer->applyChanges();
  visualization_msgs::InteractiveMarker gripperMarker;
  gripperMarker = makeGripperMaker (final_pose);
  imServer->insert(gripperMarker);
  imServer->applyChanges();




  /*geometry_msgs::Pose z_neg_pose;
  createBottomZPose(sq_, z_neg_pose);
  x_poses.push_back(z_pos_pose);

  std::vector<geometry_msgs::Pose> y_poses;
  createYPoses(sq_, y_poses);
  y_poses.push_back(z_pos_pose);*/

  std::vector<geometry_msgs::Pose> z_tilted_poses;
  createZTiltedPoses(sq_,z_tilted_poses);

  std::vector<geometry_msgs::Pose> y_tilted_poses;
  createYTiltedPosesPos(sq_,y_tilted_poses);

/*
  geometry_msgs::Pose final_pose = y_tilted_poses[2];

  std::vector<geometry_msgs::Pose> y_tilted_poses_neg;
  createYTiltedPosesNeg(sq_,y_tilted_poses_neg);

  for(int i=0;i<y_tilted_poses_neg.size();++i)
    y_tilted_poses.push_back(y_tilted_poses_neg[i]);


 /* for(int i=0;i<x_poses.size();++i)
  {
    visualization_msgs::MarkerArray markArr = createGripperMarkerMoveit(x_poses[i], yellow_color, i+1);
    for(int j=0;j<markArr.markers.size();++j)
      markARR.markers.push_back(markArr.markers[j]);
  }*/




  /*for(int i=0;i<y_poses.size();++i)
  {
    visualization_msgs::MarkerArray markArr = createGripperMarkerMoveit(y_poses[i], yellow_light_color, i+4);
    for(int j=0;j<markArr.markers.size();++j)
      markARR.markers.push_back(markArr.markers[j]);
  }*/

  /*for(int i=0;i<z_tilted_poses.size();++i)
  {
    visualization_msgs::MarkerArray markArr = createGripperMarkerMoveit(z_tilted_poses[i], yellow_light_color, i+7);
    for(int j=0;j<markArr.markers.size();++j)
      markARR.markers.push_back(markArr.markers[j]);
  }

  /*for(int i=0;i<y_tilted_poses.size();++i)
   {
     visualization_msgs::MarkerArray markArr = createGripperMarkerMoveit(y_tilted_poses[i], yellow_light_color, i+11);
     for(int j=0;j<markArr.markers.size();++j)
       markARR.markers.push_back(markArr.markers[j]);
   }





  /*visualization_msgs::MarkerArray markArr2 = createGripperMarkerMoveit(z_neg_pose, red_color, 25);
  for(int j=0;j<markArr2.markers.size();++j)
    markARR.markers.push_back(markArr2.markers[j]);*/

  visualization_msgs::MarkerArray markArr = createGripperMarkerMoveit(final_pose, yellow_color, 1);
  for(int j=0;j<markArr.markers.size();++j)
    markARR.markers.push_back(markArr.markers[j]);


  //visualization_msgs::Marker approach_arrow_marker = createApproachMarker(final_pose);

  gripper_pub_.publish(markARR);
  //approach_arrow_pub_.publish(approach_arrow_marker);
}

visualization_msgs::MarkerArray PublishMarker::createGripperMarkerMoveit(geometry_msgs::Pose pose,  const std::vector<double> colors, int id)
{
  //std::string ee_name = group.getEndEffector();
  std::string ee_name = group.getName();
  std::cout<<"Name of ee: "<<ee_name<<std::endl;
  moveit::core::RobotModelConstPtr robot_model = group.getRobotModel();
  moveit::core::RobotStatePtr robot_state_(new moveit::core::RobotState(robot_model));

  //Setting the gripper joint to the openning value of the gripper
  const std::string ee_joint = "l_gripper_l_finger_joint";
  double value = double(0.12);
  double *valuePtr = &value;
  robot_state_->setJointPositions(ee_joint, valuePtr);
  robot_state_->update();

  //New jointmodel group of the end effector with the openning value
  const moveit::core::JointModelGroup* ee_jmp = robot_model->getJointModelGroup(ee_name);
  std::string ee_group = ee_jmp->getName();
  std::cout<<"EE group: "<<ee_group<<std::endl;

  if(ee_jmp == NULL)
  {
    ROS_ERROR_STREAM("Unable to find joint model group with address"<<ee_jmp);
    //return false;
  }

  //maps
  std::map<const robot_model::JointModelGroup *, visualization_msgs::MarkerArray> ee_markers_map_;

  ee_markers_map_[ee_jmp].markers.clear();

  const std::vector<std::string>& ee_link_names = ee_jmp->getLinkModelNames();
  robot_state_->getRobotMarkers(ee_markers_map_[ee_jmp], ee_link_names);
  const std::string& ee_parent_link_name = ee_jmp->getEndEffectorParentGroup().second;
  //std::cout<<"ee_parent_link_name: "<<ee_parent_link_name<<std::endl;

  Eigen::Affine3d tf_root_to_ee = robot_state_->getGlobalLinkTransform(ee_parent_link_name);
  Eigen::Affine3d tf_ee_to_root = tf_root_to_ee.inverse();
  Eigen::Affine3d trans_bw_poses;
  trans_bw_poses = tf_root_to_ee * tf_ee_to_root;


  Eigen::Affine3d grasp_tf;
  tf::poseMsgToEigen(pose, grasp_tf);

  for(std::size_t i=0; i<ee_markers_map_[ee_jmp].markers.size();++i)
  {
    ee_markers_map_[ee_jmp].markers[i].header.frame_id = "/base_link";
    ee_markers_map_[ee_jmp].markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
    ee_markers_map_[ee_jmp].markers[i].mesh_use_embedded_materials = true;
    ee_markers_map_[ee_jmp].markers[i].id = ((i+5)*100*id)-id;

    ee_markers_map_[ee_jmp].markers[i].header.stamp = ros::Time::now();
    ee_markers_map_[ee_jmp].markers[i].ns = "gripper_links";
    ee_markers_map_[ee_jmp].markers[i].lifetime = ros::Duration(40.0);

    ee_markers_map_[ee_jmp].markers[i].color.r = colors[0];
    ee_markers_map_[ee_jmp].markers[i].color.g = colors[1];
    ee_markers_map_[ee_jmp].markers[i].color.b = colors[2];
    ee_markers_map_[ee_jmp].markers[i].color.a = colors[3];

    Eigen::Affine3d link_marker;
    tf::poseMsgToEigen(ee_markers_map_[ee_jmp].markers[i].pose, link_marker);

    Eigen::Affine3d tf_link_in_root =  tf_ee_to_root * link_marker;

    geometry_msgs::Pose new_marker_pose;
    tf::poseEigenToMsg( grasp_tf * tf_link_in_root  , new_marker_pose );
    ee_markers_map_[ee_jmp].markers[i].pose = new_marker_pose;
  }


  //gripper_pub_.publish(ee_markers_map_[ee_jmp]);
  return ee_markers_map_[ee_jmp];

}

void PublishMarker::runNode()
{
  ros::Rate rate(1);
  std::cout<<"Waiting for posestopic....\n";
  while(ros::ok())
  {

    ros::spinOnce();
    rate.sleep();
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_gripper_marker");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  PublishMarker pm(nh, "/super/sqs");
  pm.runNode();
  return 0;

}
