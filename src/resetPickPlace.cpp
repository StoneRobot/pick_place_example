// #include <moveit_msgs/PickupAction.h>
// #include <moveit_msgs/ExecuteTrajectoryAction.h>
// #include <moveit_msgs/PlaceAction.h>
// #include <moveit_msgs/ExecuteKnownTrajectory.h>
// #include <moveit_msgs/QueryPlannerInterfaces.h>
// #include <moveit_msgs/GetCartesianPath.h>
// #include <moveit_msgs/GraspPlanning.h>
// #include <moveit_msgs/GetPlannerParams.h>
// #include <moveit_msgs/SetPlannerParams.h>
// #include <ros/ros.h>
// #include <actionlib/client/simple_action_client.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_msgs/MoveItErrorCodes.h>


// class ResetPickPlace : public moveit::planning_interface::MoveItErrorCode
// {
// public:
//     ResetPickPlace(actionlib::SimpleActionClient<moveit_msgs::PickupAction> client, moveit::planning_interface::MoveGroupInterface& group);
//     void constructGoal(moveit_msgs::PlaceGoal& goal_out, const std::string& object);
//     void constructGoal(moveit_msgs::PickupGoal& goal_out, const std::string& object);
//     MoveItErrorCode pick(const std::string& object, const std::vector<moveit_msgs::Grasp>& grasps, bool plan_only = false);
// private:
//     ros::NodeHandle nh;
//     moveit::planning_interface::MoveGroupInterface& move_group;
//     actionlib::SimpleActionClient<moveit_msgs::PickupAction>& pick_action_client_;
//     std::string support_surface_;

// };

// ResetPickPlace::ResetPickPlace(actionlib::SimpleActionClient<moveit_msgs::PickupAction> client, moveit::planning_interface::MoveGroupInterface& group)
// :pick_action_client_{client},
// move_group{group}
// {
//     pick_action_client_.waitForServer();
// }



// void ResetPickPlace::constructGoal(moveit_msgs::PickupGoal& goal_out, const std::string& object)
// {
//     moveit_msgs::PickupGoal goal;
//     goal.target_name = object;
//     goal.group_name = move_group.getName();
//     goal.end_effector = move_group.getEndEffector();
//     goal.allowed_planning_time = 10;
//     goal.planner_id = move_group.getPlannerId();
//     goal.support_surface_name = support_surface_;
//     if (!support_surface_.empty())
//         goal.allow_gripper_support_collision = true;
//     goal_out = goal;
// }

// void ResetPickPlace::constructGoal(moveit_msgs::PlaceGoal& goal_out, const std::string& object)
// {
//     moveit_msgs::PlaceGoal goal;
//     goal.attached_object_name = object;
//     goal.group_name = move_group.getName();
//     goal.allowed_planning_time = 10;
//     goal.planner_id = move_group.getPlannerId();
//     goal.support_surface_name = support_surface_;
//     if (!support_surface_.empty())
//         goal.allow_gripper_support_collision = true;
//     goal_out = goal;
// }

// MoveItErrorCode ResetPickPlace::pick(const std::string& object, const std::vector<moveit_msgs::Grasp>& grasps, bool plan_only = false)
//   {
//     // 检查action是否存在
//     if (!pick_action_client_)
//     {
//       ROS_ERROR_STREAM_NAMED("move_group_interface", "Pick action client not found");
//       return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
//     }
//     if (!pick_action_client_->isServerConnected())
//     {
//       ROS_ERROR_STREAM_NAMED("move_group_interface", "Pick action server not connected");
//       return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
//     }
//     moveit_msgs::PickupGoal goal;
//     // 填充Pick 的planner，endEffie。。。
//     constructGoal(goal, object);
//     goal.possible_grasps = grasps;
//     goal.planning_options.plan_only = plan_only;
//     goal.planning_options.look_around = can_look_;
//     goal.planning_options.replan = can_replan_;
//     goal.planning_options.replan_delay = replan_delay_;
//     goal.planning_options.planning_scene_diff.is_diff = true;
//     goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

//     pick_action_client_->sendGoal(goal);
//     if (!pick_action_client_->waitForResult())
//     {
//       ROS_INFO_STREAM_NAMED("move_group_interface", "Pickup action returned early");
//     }
//     if (pick_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//     {
//       return MoveItErrorCode(pick_action_client_->getResult()->error_code);
//     }
//     else
//     {
//       ROS_WARN_STREAM_NAMED("move_group_interface", "Fail: " << pick_action_client_->getState().toString() << ": "
//                                                              << pick_action_client_->getState().getText());
//       return MoveItErrorCode(pick_action_client_->getResult()->error_code);
//     }
//   }