#pragma once
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>

#include <pluginlib/class_loader.h>

// MoveIt!
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/planning_interface/planning_interface.h>
// #include <moveit/planning_scene/planning_scene.h>
// #include <moveit/kinematic_constraints/utils.h>
// // #include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_msgs/PlanningScene.h>
// // #include <moveit_visual_tools/moveit_visual_tools.h>
// #include <boost/scoped_ptr.hpp>

#include <vector>

#include "hirop_msgs/Pick.h"
#include "hirop_msgs/Place.h"
#include "hirop_msgs/RemoveObject.h"
#include "hirop_msgs/ShowObject.h"
#include "hirop_msgs/listActuator.h"
#include "hirop_msgs/listGenerator.h"
#include "hirop_msgs/SetGenActuator.h"
#include "hirop_msgs/ObjectArray.h"
#include "hirop_msgs/detection.h"

class PickPlaceBridge
{
public:
    PickPlaceBridge(ros::NodeHandle _n, moveit::planning_interface::MoveGroupInterface& group);



private:
    void openGripper(trajectory_msgs::JointTrajectory& posture);
    void closedGripper(trajectory_msgs::JointTrajectory& posture);
    moveit_msgs::MoveItErrorCodes pick(geometry_msgs::Pose pose, float pre_vec[], float back_vec[]);
    moveit_msgs::MoveItErrorCodes place(geometry_msgs::Pose pose, float pre_vec[], float back_vec[]);
    bool setGenActuator();
    void rmObject();
    void showObject(geometry_msgs::Pose pose);
    void objectCallback(const hirop_msgs::ObjectArray::ConstPtr& msg);
    void actionDataCallback(const std_msgs::Int32MultiArray::ConstPtr &msg);
    void CartesianPath(geometry_msgs::Pose pose);


    ros::NodeHandle& nh;
    moveit::planning_interface::MoveGroupInterface& move_group;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ros::ServiceClient list_generator_client;
    ros::ServiceClient set_gen_actuator_client;
    ros::ServiceClient list_actuator_client;
    ros::ServiceClient show_object_client;
    ros::ServiceClient remove_object_client;
    ros::ServiceClient detection_client;
    ros::Publisher pick_pose_pub;
    ros::Publisher place_pose_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber action_sub;
    int intent;
    int object;
    int target;
    geometry_msgs::Pose place_pose1;
    geometry_msgs::Pose place_pose2;
    geometry_msgs::Pose place_pose3;
    std::vector<geometry_msgs::Pose> place_poses;


};
