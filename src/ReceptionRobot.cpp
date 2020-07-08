#include "reception_robot/ReceptionRobot.h"

ReceptionRobot::ReceptionRobot(ros::NodeHandle& n)
:nh{n}
{
    // 客户端
    pickClient = nh.serviceClient<pick_place_bridge::PickPlacePose>("/pick");
    fixedPickClient = nh.serviceClient<pick_place_bridge::PickPlacePose>("/fixed_pick");
    placeClient = nh.serviceClient<pick_place_bridge::PickPlacePose>("/place");
    moveClient = nh.serviceClient<pick_place_bridge::PickPlacePose>("/move");
    detectionClient = nh.serviceClient<hirop_msgs::detection>("/detection");
    getForceClient = nh.serviceClient<hirop_msgs::getForce>("getForce");
    moveSeqClient = nh.serviceClient<hirop_msgs::moveSeqIndex>("moveSeq");
    // 服务
    handClawGrabDollServer = nh.advertiseService("handClaw_grabDoll", &ReceptionRobot::handClawGrabDollCallback, this);
    handgestureServer = nh.advertiseService("handClaw_shakeHand", &ReceptionRobot::handgestureSerCallback, this);
    // 订阅
    objectArraySub = nh.subscribe<hirop_msgs::ObjectArray>("object_array", 1, &ReceptionRobot::objectCallBack, this);
    pedestrainSub = nh.subscribe("/pedestrian_detection", 10, &ReceptionRobot::pedestrainCallback, this);
    handgestureSub = nh.subscribe("/handgesture_detection", 10, &ReceptionRobot::handgestureCallback, this);
    shakeSub = nh.subscribe("isShake", 10, &ReceptionRobot::shakeCallback, this);
    // 发布
    speedScalePub = nh.advertise<std_msgs::Bool>("speedScale", 10);
    detachObjectPub = nh.advertise<std_msgs::Empty>("detach_object", 10);
}

bool ReceptionRobot::handClawGrabDollCallback(rb_msgAndSrv::rb_DoubleBool::Request& req, rb_msgAndSrv::rb_DoubleBool::Response& rep)
{
    test();
    rep.respond = true;
    return true;
}

void ReceptionRobot::objectCallBack(const hirop_msgs::ObjectArray::ConstPtr& msg)
{
    geometry_msgs::PoseStamped pose = msg->objects[0].pose;
    transformFrame(pose, "world");
    pick_place_bridge::PickPlacePose pickPose;
    pickPose.request.Pose = pose;
    pickClient.call(pickPose);
    // fixedPickClient.call(pickPose);
    pick_place_bridge::PickPlacePose placePose;
    placePose.request.Pose.header.frame_id = "world";
    placePose.request.Pose.pose.position.x = 0.80;
    placePose.request.Pose.pose.position.y = 0.0;
    placePose.request.Pose.pose.position.z = 1.4;
    placePose.request.Pose.pose.orientation.w = 1;
    moveClient.call(placePose);
    std_msgs::Empty emptryMsg;
    detachObjectPub.publish(emptryMsg);
    int cnt = 0;
    while (cnt < 40)
    {
        if(!checkForce() || cnt == 39)
        {
            system("rostopic pub -1 /back_home std_msgs/Int8 \"data: 0\"");
            setFiveFightPose(HOME);
            break;
        }
        cnt++;
        ros::WallDuration(0.25).sleep();
    }
}

///////////////回调//////////////////////
void ReceptionRobot::pedestrainCallback(const std_msgs::Bool::ConstPtr& msg)
{
    std_msgs::Bool sMsg;
    sMsg.data = true;
    speedScalePub.publish(sMsg);
}


void ReceptionRobot::shakeCallback(const std_msgs::Bool::ConstPtr& msg)
{
    isShake = msg->data;
}

void ReceptionRobot::handgestureCallback(const std_msgs::Bool::ConstPtr& msg)
{
    handgesture();
}

bool ReceptionRobot::handgestureSerCallback(rb_msgAndSrv::rb_DoubleBool::Request& req, rb_msgAndSrv::rb_DoubleBool::Response& rep)
{
    handgesture();
	rep.respond = true;
	return true;
}
/////////////////// 实现//////////////

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

void ReceptionRobot::test()
{
    pick_place_bridge::PickPlacePose pose;
    pose.request.Pose.header.frame_id = "world";
    // pose.request.Pose.pose.position.x = 0.70;
    pose.request.Pose.pose.position.x = 0.0453143;
    pose.request.Pose.pose.position.y = -0.838108;
    pose.request.Pose.pose.position.z = 1.50026;

    pose.request.Pose.pose.orientation.x = 0.248138;
    pose.request.Pose.pose.orientation.y = 0.312854;
    pose.request.Pose.pose.orientation.z = -0.590081;
    pose.request.Pose.pose.orientation.w = 0.70168;
    moveClient.call(pose);
    hirop_msgs::detection d;
    d.request.detectorName = "Yolo6d";
    d.request.detectorType = 1;
    d.request.objectName = "doll";
    detectionClient.call(d);
}

bool ReceptionRobot::checkForce()
{
    hirop_msgs::getForce srv;
    getForceClient.call(srv);
    for(int i=0; i < srv.response.finger_force.size(); ++i)
    {
        if(i < 4)
        {
            srv.response.finger_force[i] > 50;
            return true;
        }
        else
        {
            srv.response.finger_force[i] > 90;
            return true;
        }
    }
    return false;
}

bool ReceptionRobot::setFiveFightPose(int index)
{
    ROS_INFO_STREAM("Pose index: " << index);
    hirop_msgs::moveSeqIndex srv;
    srv.request.index = index;
    if(moveSeqClient.call(srv))
    {
        if(srv.response.sucesss)
            ROS_INFO_STREAM("alreay move to pose");
        else
            ROS_INFO_STREAM("move to pose faild");
        return srv.response.sucesss;
    }
    ROS_INFO_STREAM("check gripper server");
    return false;
}

bool ReceptionRobot::handgesture()
{
    ROS_INFO_STREAM("----handgesture begin ----");
    geometry_msgs::PoseStamped targetPose;
    targetPose.header.frame_id = "world";
    targetPose.pose.position.x = 0.80;
    targetPose.pose.position.y = 0.0;
    targetPose.pose.position.z = 1.4;
    targetPose.pose.orientation.w = 1;
    pick_place_bridge::PickPlacePose pose;
    pose.request.Pose = targetPose;
    moveClient.call(pose);
    int cnt = 0;
    ros::WallDuration(0.5).sleep();
    while (cnt < 12)
    {
        if(checkForce() || isShake || cnt == 11)
        {
            ROS_INFO_STREAM("set five fight pose: " << SHAKE);
            setFiveFightPose(SHAKE);
            break;
        }
        ros::WallDuration(0.25).sleep();
        cnt ++;
    }
    cnt = 0;
    while (cnt < 16)
    {
        if(!checkForce() || !isShake || cnt == 15)
        {
            ROS_INFO_STREAM("set five fight pose: " << HOME);
            setFiveFightPose(HOME);
            break;
        }
        ros::WallDuration(0.25).sleep();
        cnt ++;
    }
    ros::WallDuration(1).sleep();
    system("rostopic pub -1 /back_home std_msgs/Int8 \"data: 0\"");
    ROS_INFO_STREAM("----handgesture over ----");
    return true;
}
