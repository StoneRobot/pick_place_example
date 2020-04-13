#include "pick_place_example/pick_place_bridge.h"

PickPlaceBridge::PickPlaceBridge(ros::NodeHandle _n, moveit::planning_interface::MoveGroupInterface& group)
:nh{_n},
move_group{group}
{
    ROS_INFO("init");
    remove_object_client = nh.serviceClient<hirop_msgs::RemoveObject>("removeObject");
    show_object_client = nh.serviceClient<hirop_msgs::ShowObject>("showObject");
    list_generator_client = nh.serviceClient<hirop_msgs::listGenerator>("listGenerator");
    set_gen_actuator_client = nh.serviceClient<hirop_msgs::SetGenActuator>("setGenActuator");
    list_actuator_client = nh.serviceClient<hirop_msgs::listActuator>("listActuator");
    // gripper
    pick_pose_pub = nh.advertise<geometry_msgs::Pose>("pick_pose", 1);
    place_pose_pub = nh.advertise<geometry_msgs::Pose>("place_pose", 1);
    // 发布姿态
    pose_sub = nh.subscribe("/object_array", 1, &PickPlaceBridge::objectCallback, this);
    action_sub = nh.subscribe("/action_data", 1, &PickPlaceBridge::actionDataCallback, this);

    detection_client = nh.serviceClient<hirop_msgs::detection>("detection");

    setGenActuator();

    tf2::Quaternion orien;
    orien.setRPY(0, 0, -1.57);

    place_pose1.position.x = 0.418;
    place_pose1.position.y = -0.68;
    place_pose1.position.z = 0.63;
    place_pose1.orientation = tf2::toMsg(orien);


    place_pose2.position.x = 0.418;
    place_pose2.position.y = -0.68;
    place_pose2.position.z = 0.32;
    place_pose2.orientation = tf2::toMsg(orien);

    place_pose3.position.x = 0.78;
    place_pose3.position.y = 0;
    place_pose3.position.z = 0.25;
    orien.setRPY(0, 0, 6.28);
    place_pose3.orientation = tf2::toMsg(orien);
    // place_pose3.orientation.x = 0;
    // place_pose3.orientation.y = 0;
    // place_pose3.orientation.z = 0;
    // place_pose3.orientation.w = 1;

    place_poses.push_back(place_pose1);
    place_poses.push_back(place_pose2);
    place_poses.push_back(place_pose3);
    ROS_INFO_STREAM(place_poses[0] << place_poses[1] << place_poses[2]);

    ROS_INFO("init_over");
}

void PickPlaceBridge::openGripper(trajectory_msgs::JointTrajectory& posture)
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

void PickPlaceBridge::closedGripper(trajectory_msgs::JointTrajectory& posture)
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

moveit_msgs::MoveItErrorCodes PickPlaceBridge::pick(geometry_msgs::Pose pose, float pre_vec[], float back_vec[])
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);
    grasp_pose[0].pre_grasp_posture.header.frame
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
    grasps[0].pre_grasp_approach.min_distance = 0.015;
    grasps[0].pre_grasp_approach.desired_distance = 0.05;
    // 撤退方向
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    grasps[0].post_grasp_retreat.direction.vector.x = back_vec[0];
    grasps[0].post_grasp_retreat.direction.vector.y = back_vec[1];
    grasps[0].post_grasp_retreat.direction.vector.z = back_vec[2];
    grasps[0].post_grasp_retreat.min_distance = 0.015;
    grasps[0].post_grasp_retreat.desired_distance = 0.05;
    // 模拟关闭夹爪
    closedGripper(grasps[0].grasp_posture);
    // 动作

    return move_group.pick("object", grasps);
}

moveit_msgs::MoveItErrorCodes PickPlaceBridge::place(geometry_msgs::Pose pose, float pre_vec[], float back_vec[])
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
    // 放置方向
    place_location[0].pre_place_approach.direction.header.frame_id = "base_link";

    // place_location[0].pre_place_approach.direction.vector.x = 1;
    //   place_location[0].pre_place_approach.direction.vector.x = 1;
    //   place_location[0].pre_place_approach.direction.vector.y = pre_vec[1];
    // 不同的
    if(intent == 0)
    {
        place_location[0].pre_place_approach.direction.vector.y = -1; 
        place_location[0].post_place_retreat.direction.vector.y = 1;
    }
    else if(intent == 1)
    {
        place_location[0].pre_place_approach.direction.vector.z = -1; 
        place_location[0].post_place_retreat.direction.vector.z = 1;
    }  
    place_location[0].pre_place_approach.min_distance = 0.05;
    place_location[0].pre_place_approach.desired_distance = 0.115;
    // 撤退方向
    place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
    //   place_location[0].post_place_retreat.direction.vector.x = back_vec[0];
    //   place_location[0].post_place_retreat.direction.vector.y = back_vec[1];

        
    place_location[0].post_place_retreat.min_distance = 0.05;
    place_location[0].post_place_retreat.desired_distance = 0.115;
    // 模拟打开夹爪
    openGripper(place_location[0].post_place_posture);
    // 抓取动作

    return move_group.place("object", place_location);
}

bool PickPlaceBridge::setGenActuator()
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

void PickPlaceBridge::showObject(geometry_msgs::Pose pose)
{
    hirop_msgs::ShowObject srv;
    // tf2::Quaternion orientation;
    // orientation.setRPY(0, 0, -M_PI / 2);
    // ROS_INFO_STREAM(tf2::toMsg(orientation));
    srv.request.objPose.header.frame_id = "base_link";
    srv.request.objPose.pose.position.x = pose.position.x;
    srv.request.objPose.pose.position.y = pose.position.y;
    srv.request.objPose.pose.position.z = pose.position.z;
    srv.request.objPose.pose.orientation.x = pose.orientation.x;
    srv.request.objPose.pose.orientation.y = pose.orientation.y;
    srv.request.objPose.pose.orientation.z = pose.orientation.z;
    srv.request.objPose.pose.orientation.w = pose.orientation.w;
    // srv.request.objPose.pose.orientation = tf2::toMsg(orientation);
    if(show_object_client.call(srv))
    {
        ROS_INFO_STREAM("show object "<< (srv.response.isSetFinsh ? "Succeed" : "Faild"));
    }
    else
    {
        ROS_INFO("check \\showObject service ");
    }
}

void PickPlaceBridge::rmObject()
{
    hirop_msgs::RemoveObject srv;
    remove_object_client.call(srv);
}

void PickPlaceBridge::actionDataCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    intent = msg->data[0];
    object = msg->data[1];
    target = msg->data[2];

    hirop_msgs::detection det;
    det.request.objectName = "object";
    det.request.detectorName = "detector";
    det.request.detectorType = 1;
    det.request.detectorConfig = "config";
    if(detection_client.call(det))
    {
        ROS_INFO("Identify the successful");
    }
}

void PickPlaceBridge::CartesianPath(geometry_msgs::Pose pose)
{
    // 直线去抓取object
    // 从现在位置
    move_group.setPoseReferenceFrame("base_link");
    geometry_msgs::Pose target_pose = move_group.getCurrentPose(move_group.getEndEffectorLink()).pose;
    std::vector<geometry_msgs::Pose> waypoints;
    target_pose.position.z -= 1.0;
    ROS_INFO_STREAM("target_pose: " << target_pose);
    waypoints.push_back(target_pose);
    // 到预抓取位置
    geometry_msgs::Pose target_finish = pose;


    target_finish.position.x *= 0.85;
    target_finish.position.y *= 0.85;
    target_finish.position.z *= 1;
    target_finish.orientation.x = 0;
    target_finish.orientation.y = 0.254;
    target_finish.orientation.z = 0;
    target_finish.orientation.w = 0.967;



    waypoints.push_back(target_finish);
    ROS_INFO_STREAM("target_finish: " << target_finish);

    // 警告 - 在操作实际硬件时禁用跳转阈值可能会
    // 导致冗余接头的大量不可预知运动，并且可能是一个安全问题
    bool velocity = false;
    nh.getParam("/velocity", velocity);
    if(velocity == true)
    {
        move_group.setMaxVelocityScalingFactor(0.01);
        ROS_INFO("set velocityScalingFactor 0.01");
    }
    else
        move_group.setMaxVelocityScalingFactor(1);
    moveit_msgs::RobotTrajectory trajectory;
    double jump_threshold = 0.0;
    double eef_step = 0.02;

    double fraction = 0;
    int cnt = 0;

    while (fraction < 1.0 && cnt < 100)
    {
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        cnt ++;
    }
    ROS_INFO_STREAM( "waypoints "<<waypoints.size()<<" "<<fraction);
    my_plan.trajectory_ = trajectory;
    // 运动
    move_group.execute(my_plan);
}





void PickPlaceBridge::objectCallback(const hirop_msgs::ObjectArray::ConstPtr& msg)
{
    geometry_msgs::Pose pose;
    nh.getParam("/intent", intent);
    nh.getParam("/target", target);
    int i = msg->objects.size();
    static int cnt = 0;
    static int errorCnt = 0;
    bool velocity = false;
    nh.getParam("/velocity", velocity);
    if(velocity == true)
    {
        move_group.setMaxVelocityScalingFactor(0.01);
        ROS_INFO("set velocityScalingFactor 0.01");
    }
    else
        move_group.setMaxVelocityScalingFactor(1);
    cnt++;
    nh.setParam("/cnt", cnt);
    for(int j = 0; j < i; ++j)
    {   
        pose = msg->objects[0].pose.pose;
        rmObject();
        showObject(pose);
        float pick_pre_vec[3] = {0};
        float pick_back_vec[3] = {0};
        float place_pre_vec[3] = {0};
        float place_back_vec[3] = {0};

        if(intent == 0)
        {
            pick_pre_vec[0] = 1;   
            pick_back_vec[2] = 1;
            place_pre_vec[1] = -1;
            place_back_vec[1] = 1;
            this->CartesianPath(pose);
        }
        else if(intent == 1)
        {
            tf2::Quaternion orientation;
            orientation.setRPY(0, 0, -1.57);
            pose.orientation = tf2::toMsg(orientation);
            pick_pre_vec[1] = -1;   
            pick_back_vec[1] = 1;
            place_pre_vec[0] = 1;
            place_back_vec[2] = 1;
        }
            
        pick(pose, pick_pre_vec, pick_back_vec);
        ros::WallDuration(1.0).sleep();
        // 测试
        moveit_msgs::MoveItErrorCodes code;
        code = place(place_poses[target], place_pre_vec, place_back_vec);
        if(code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            errorCnt ++;
        nh.setParam("error_cnt", errorCnt);
        move_group.setNamedTarget("home");
        move_group.move();
        nh.setParam("/over", true);
    }
}

