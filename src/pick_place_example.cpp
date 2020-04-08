#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>

#include "hirop_msgs/Pick.h"
#include "hirop_msgs/Place.h"
#include "hirop_msgs/ShowObject.h"
#include "hirop_msgs/listActuator.h"
#include "hirop_msgs/listGenerator.h"
#include "hirop_msgs/SetGenActuator.h"

ros::Publisher pick_pose_pub;
ros::Publisher place_pose_pub;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "left_finger_1_joint";
  posture.joint_names[1] = "right_finger_1_joint";
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.6;
  posture.points[0].positions[1] = -0.6;  
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "left_finger_1_joint";
  posture.joint_names[1] = "right_finger_1_joint";
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0;
  posture.points[0].positions[1] = 0;  
  posture.points[0].time_from_start = ros::Duration(0.5);

}

void pick(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::Pose pose, int pre_vec[], int back_vec[])
{
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
  // 抓取姿态
  grasps[0].grasp_pose.header.frame_id = "base_link";
  // tf2::Quaternion orientation;
  // orientation.setRPY(0, 0, -M_PI / 2);
  // grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.orientation.x = pose.orientation.x;
  grasps[0].grasp_pose.pose.orientation.y = pose.orientation.y;
  grasps[0].grasp_pose.pose.orientation.z = pose.orientation.z;
  grasps[0].grasp_pose.pose.orientation.w = pose.orientation.w;
  grasps[0].grasp_pose.pose.position.x = pose.position.x;
  grasps[0].grasp_pose.pose.position.y = pose.position.y;
  grasps[0].grasp_pose.pose.position.z = pose.position.z;
  // 发送pick的位置信息
  pick_pose_pub.publish(grasps[0].grasp_pose.pose);
  // 抓取方向
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
  grasps[0].pre_grasp_approach.direction.vector.x = pre_vec[0];
  grasps[0].pre_grasp_approach.direction.vector.y = pre_vec[1];
  grasps[0].pre_grasp_approach.direction.vector.z = pre_vec[2];
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;
  // 撤退方向
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
  grasps[0].post_grasp_retreat.direction.vector.x = back_vec[0];
  grasps[0].post_grasp_retreat.direction.vector.y = back_vec[1];
  grasps[0].post_grasp_retreat.direction.vector.z = back_vec[2];
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;
  // 模拟关闭夹爪
  closedGripper(grasps[0].grasp_posture);
  // 动作
  move_group.pick("object", grasps);

}

void place(moveit::planning_interface::MoveGroupInterface& group, geometry_msgs::Pose pose, int pre_vec[], int back_vec[])
{

  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);
  // 抓取位置
  place_location[0].place_pose.header.frame_id = "base_link";
  // tf2::Quaternion orientation;
  // orientation.setRPY(0, 0, 0);
  // place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
  place_location[0].place_pose.pose.orientation.x = pose.orientation.x;
  place_location[0].place_pose.pose.orientation.y = pose.orientation.y;
  place_location[0].place_pose.pose.orientation.z = pose.orientation.z;
  place_location[0].place_pose.pose.orientation.w = pose.orientation.w;
  place_location[0].place_pose.pose.position.x = pose.position.x;
  place_location[0].place_pose.pose.position.y = pose.position.y;
  place_location[0].place_pose.pose.position.z = pose.position.z;
  // 发送放置的位置信息
  place_pose_pub.publish(place_location[0].place_pose.pose);
  // 抓取方向
  place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
  place_location[0].pre_place_approach.direction.vector.x = pre_vec[0];
  place_location[0].pre_place_approach.direction.vector.y = pre_vec[1];
  place_location[0].pre_place_approach.direction.vector.z = pre_vec[2];
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;
  // 撤退方向
  place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
  place_location[0].post_place_retreat.direction.vector.x = back_vec[0];
  place_location[0].post_place_retreat.direction.vector.y = back_vec[1];
  place_location[0].post_place_retreat.direction.vector.z = back_vec[2];
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;
  // 模拟打开夹爪
  openGripper(place_location[0].post_place_posture);
  // 抓取动作
  group.place("object", place_location);
}

ros::ServiceClient list_generator_client;
ros::ServiceClient set_gen_actuator_client;
ros::ServiceClient list_actuator_client;

bool setGenActuator()
{
    hirop_msgs::listGenerator list_generator_srv;
    hirop_msgs::listActuator list_actuator_srv;
    hirop_msgs::SetGenActuator set_gen_actuator_srv;
    if(list_generator_client.call(list_generator_srv) && list_actuator_client.call(list_actuator_srv))
    {
        set_gen_actuator_srv.request.generatorName = list_generator_srv.response.generatorList[0];
        set_gen_actuator_srv.request.actuatorName = list_actuator_srv.response.actuatorList[0];
        if(set_gen_actuator_client.call(set_gen_actuator_srv))
            return true;
    }
    return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("arm");
  group.setPlanningTime(45.0);

  // addCollisionObjects(planning_scene_interface);
  // 
  ros::ServiceClient show_object_client = nh.serviceClient<hirop_msgs::ShowObject>("showObject");
  list_generator_client = nh.serviceClient<hirop_msgs::listGenerator>("listGenerator");
  set_gen_actuator_client = nh.serviceClient<hirop_msgs::SetGenActuator>("setGenActuator");
  list_actuator_client = nh.serviceClient<hirop_msgs::listActuator>("listActuator");
  // gripper
  pick_pose_pub = nh.advertise<geometry_msgs::Pose>("pick_pose", 1);
  place_pose_pub = nh.advertise<geometry_msgs::Pose>("place_pose", 1);

  setGenActuator();
  
  hirop_msgs::ShowObject srv;
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, -M_PI / 2);
  ROS_INFO_STREAM(tf2::toMsg(orientation));
  srv.request.objPose.header.frame_id = "base_link";
  srv.request.objPose.pose.position.x = 0.418;
  srv.request.objPose.pose.position.y = -0.618;
  srv.request.objPose.pose.position.z = 0.68;
  srv.request.objPose.pose.orientation = tf2::toMsg(orientation);
  if(show_object_client.call(srv))
  {
      ROS_INFO_STREAM("show object "<< (srv.response.isSetFinsh ? "Succeed" : "Faild"));
  }
  else
  {
      ROS_INFO("check \\showObject service ");
  }
  geometry_msgs::Pose pick_pose;
  geometry_msgs::Pose place_pose;
  ros::WallDuration(1.0).sleep();
  int pre_vec[3] = {1, 0, 0};
  int back_vec[3] = {-1, 0, 0};
  pick(group, pick_pose, pre_vec, back_vec);

  ros::WallDuration(1.0).sleep();
  int place_pre_vec[3] = {0, -1, 0};
  int place_back_vec[3] = {0, 1, 0};
  place(group, place_pose, place_pre_vec, place_back_vec);

  ros::waitForShutdown();
  return 0;
}


