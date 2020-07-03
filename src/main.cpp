#include "pick_place_bridge/PickPlacePose.h"
#include "reception_robot/ReceptionRobot.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "reception_robot");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    ReceptionRobot r(nh);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
