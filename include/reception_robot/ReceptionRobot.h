#pragma once
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "pick_place_bridge/PickPlacePose.h"
#include "hirop_msgs/ObjectArray.h"
#include "hirop_msgs/detection.h"
#include "rb_msgAndSrv/rb_DoubleBool.h"

class ReceptionRobot
{
public:
    ReceptionRobot(ros::NodeHandle& n);
    bool transformFrame(geometry_msgs::PoseStamped& poseStamped, std::string frame_id);
    void test();
private:

    bool handClawGrabDollCallback(rb_msgAndSrv::rb_DoubleBool::Request& req, rb_msgAndSrv::rb_DoubleBool::Response& rep);

    void objectCallBack(const hirop_msgs::ObjectArray::ConstPtr& msg);
    ros::NodeHandle& nh;

    ros::ServiceClient pickClient;
    ros::ServiceClient fixedPickClient;
    ros::ServiceClient placeClient;
    ros::ServiceClient moveClient;

    ros::ServiceClient detectionClient;

    ros::ServiceServer handClawGrabDollServer; 

    ros::Subscriber objectArraySub;
};


