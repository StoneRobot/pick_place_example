// #pragma once

// #include <moveit_msgs/PickupAction.h>
// #include <moveit_msgs/ExecuteTrajectoryAction.h>
// #include <moveit_msgs/PlaceAction.h>
// #include <moveit_msgs/ExecuteKnownTrajectory.h>
// #include <moveit_msgs/QueryPlannerInterfaces.h>
// #include <moveit_msgs/GetCartesianPath.h>
// #include <moveit_msgs/GraspPlanning.h>
// #include <moveit_msgs/GetPlannerParams.h>
// #include <moveit_msgs/SetPlannerParams.h>

// class ResetPickPlace
// {
// public:
//     ResetPickPlace(actionlib::SimpleActionClient<moveit_msgs::PickupAction> client, moveit::planning_interface::MoveGroupInterface& group);
//     void constructGoal(moveit_msgs::PlaceGoal& goal_out, const std::string& object);
//     void constructGoal(moveit_msgs::PickupGoal& goal_out, const std::string& object);
// private:
//     ros::NodeHandle nh;
//     moveit::planning_interface::MoveGroupInterface& move_group;
//     actionlib::SimpleActionClient<moveit_msgs::PickupAction>& pick_action_client_;

// };
