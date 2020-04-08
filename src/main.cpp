#include "pick_place_example/pick_place_bridge.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    std::string PLANNING_GROUP;
    nh.getParam("/pick_place/group", PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    PickPlaceBridge p(nh, move_group);
    ros::waitForShutdown();
    return 0;
}