#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>

#include <math.h>

#include "hirop_msgs/Pick.h"
#include "hirop_msgs/Place.h"
#include "hirop_msgs/ShowObject.h"
#include "hirop_msgs/listActuator.h"
#include "hirop_msgs/listGenerator.h"
#include "hirop_msgs/SetGenActuator.h"
#include "hirop_msgs/openGripper.h"
#include "hirop_msgs/closeGripper.h"


geometry_msgs::Pose pick_pose;
geometry_msgs::Pose place_pose;
int intent;

void pickCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    intent = 1;
    pick_pose.position = msg->position;
    ROS_INFO_STREAM("record pick msg");
}

void placeCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    intent = 2;
    place_pose.position = msg->position;
    ROS_INFO_STREAM("record place msg");
}

bool checkPickPose(geometry_msgs::Pose pose)
{
    if(fabs(pose.position.x - pick_pose.position.x) < 0.01 && fabs(pose.position.y - pick_pose.position.y) < 0.01 && fabs(pose.position.z - pick_pose.position.z) < 0.01)
    {
        return true;
    }
    return false;
}

bool checkPlacePose(geometry_msgs::Pose pose)
{
    if(fabs(pose.position.x - place_pose.position.x) < 0.01 && fabs(pose.position.y - place_pose.position.y) < 0.01 && fabs(pose.position.z - place_pose.position.z) < 0.05)
    {
        return true;
    }
    return false;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gripper_controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    std::string PLANNING_GROUP;
    nh.getParam("/gripper_controller/group", PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    ros::Subscriber pick_sub = nh.subscribe("pick_pose", 1, pickCallback);
    ros::Subscriber place_sub = nh.subscribe("place_pose", 1, placeCallback);

    ros::ServiceClient open_gripper_client = nh.serviceClient<hirop_msgs::openGripper>("openGripper");
    ros::ServiceClient close_gripper_client = nh.serviceClient<hirop_msgs::closeGripper>("closeGripper");

    ros::Rate loop(5);

    while (ros::ok())
    {
        if(intent != 0)
        {
            geometry_msgs::Pose pose = move_group.getCurrentPose().pose;
            pose.position.z -= 1;
            if(intent == 1)
                if(checkPickPose(pose))
                {
                    hirop_msgs::closeGripper srv;
                    if(close_gripper_client.call(srv))
                    {
                        ROS_INFO_STREAM("close gripper " << srv.response.isClose ? "" : "faild");
                    }
                    else
                    {
                        ROS_INFO_STREAM("check close gripper service!!!");

                    }
                    intent = 0;
                }

            if(intent == 2)
                if(checkPlacePose(pose))
                {
                    hirop_msgs::openGripper srv;
                    if(open_gripper_client.call(srv))
                    {
                        ROS_INFO_STREAM("open gripper " <<(srv.response.isOpen ? "" : "faild"));
                    }
                    else
                    {
                        ROS_INFO_STREAM("check open gripper service!!!");
                    }
                    intent = 0;
                }
        }
        loop.sleep();
    }
    return 0;
}