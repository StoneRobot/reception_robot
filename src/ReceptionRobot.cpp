#include "reception_robot/ReceptionRobot.h"

ReceptionRobot::ReceptionRobot(ros::NodeHandle& n)
:nh{n}
{
    pickClient = nh.serviceClient<pick_place_bridge::PickPlacePose>("/pick");
    placeClient = nh.serviceClient<pick_place_bridge::PickPlacePose>("/place");
    moveClient = nh.serviceClient<pick_place_bridge::PickPlacePose>("/move");

    detectionClient = nh.serviceClient<hirop_msgs::detection>("/detection");

    objectArraySub = nh.subscribe<hirop_msgs::ObjectArray>("object_array", 1, &ReceptionRobot::objectCallBack, this);
    
    handClawGrabDollServer = nh.advertiseService("handClaw_grabDoll", &ReceptionRobot::handClawGrabDollCallback, this);
}

bool ReceptionRobot::transformFrame(geometry_msgs::PoseStamped& poseStamped, std::string frame_id="world")
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::PoseStamped* worldFramePose = new geometry_msgs::PoseStamped[1];
    geometry_msgs::PoseStamped* otherFramePose = new geometry_msgs::PoseStamped[1];
    tf::TransformListener listener;



    otherFramePose[0] = poseStamped;
    for(int i=0; i < 5; ++i)
    {
        try
        {
            listener.transformPose(frame_id, otherFramePose[0], worldFramePose[0]);
            break;
        }
        catch(tf::TransformException& ex)
        {
            ROS_INFO_STREAM(ex.what());
            ros::WallDuration(1).sleep();
            continue;
        }
    }
    poseStamped = worldFramePose[0];
    delete[] worldFramePose;
    delete[] otherFramePose;
    double add[3] = {0};
    nh.getParam("/grasp_place/position_x_add", add[0]);
    nh.getParam("/grasp_place/position_y_add", add[1]);
    nh.getParam("/grasp_place/position_z_add", add[2]);

    poseStamped.pose.position.x += add[0];
    poseStamped.pose.position.y += add[1];
    poseStamped.pose.position.z += add[2];
    if(poseStamped.header.frame_id == "world")
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool ReceptionRobot::handClawGrabDollCallback(rb_msgAndSrv::rb_DoubleBool::Request& req, rb_msgAndSrv::rb_DoubleBool::Response& rep)
{
    test();
    rep.respond = true;
}

void ReceptionRobot::test()
{
    pick_place_bridge::PickPlacePose pose;
    pose.request.Pose.header.frame_id = "world";
    pose.request.Pose.pose.position.x = 0.465235;
    pose.request.Pose.pose.position.y = -0.299244;
    pose.request.Pose.pose.position.z = 1.49598;
    pose.request.Pose.pose.orientation.x = 0.139581;
    pose.request.Pose.pose.orientation.y = -0.104279;
    pose.request.Pose.pose.orientation.z = -0.631793;
    pose.request.Pose.pose.orientation.w = 0.755302;
    moveClient.call(pose);
    hirop_msgs::detection d;
    d.request.detectorName = "Yolo6d";
    d.request.detectorType = 1;
    d.request.objectName = "doll";
    detectionClient.call(d);
}

void ReceptionRobot::objectCallBack(const hirop_msgs::ObjectArray::ConstPtr& msg)
{
    geometry_msgs::PoseStamped pose = msg->objects[0].pose;
    transformFrame(pose);
    pick_place_bridge::PickPlacePose pickPose;
    pickPose.request.Pose = pose;
    pickClient.call(pickPose);
    pick_place_bridge::PickPlacePose placePose;
    placePose.request.Pose.header.frame_id = "world";
    placePose.request.Pose.pose.position.x = 0.5;
    placePose.request.Pose.pose.position.y = 0.0;
    placePose.request.Pose.pose.position.z = 1.3;
    placePose.request.Pose.pose.orientation.w = 1;
    moveClient.call(placePose);
}
