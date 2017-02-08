#ifndef CREATE_GRIPPER_MARKER_H
#define CREATE_GRIPPER_MARKER_H

#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<visualization_msgs/InteractiveMarker.h>
#include<tf2/LinearMath/Transform.h>
#include<tf2/LinearMath/Quaternion.h>


class CreateGripperMarker
{
public:
  CreateGripperMarker();
  static visualization_msgs::InteractiveMarker makeGripperMaker(geometry_msgs::Pose pose);


private:
  static visualization_msgs::Marker createGripperMeshMarker(double x, double y, double z, double rx, double ry, double rz, double rw, std::string meshLocation, int color);
  static geometry_msgs::Pose rotatePose(const geometry_msgs::Pose pose);

};

#endif // CREATE_GRIPPER_MARKER_H
