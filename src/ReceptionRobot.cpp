#include "reception_robot/ReceptionRobot.h"

ReceptionRobot::ReceptionRobot(ros::NodeHandle& n)
:nh{n}
{
    recordLoadPosePtr = new recordLoadPose(nh);
    nh.getParam("/reception_robot/detectionPose", detectionPosePath);
    nh.getParam("/reception_robot/handgesturePose", handgesturePosePath);
    try
    {
        /* code */
        recordLoadPosePtr->loadRobotPose(detectionPose, detectionPosePath);
        recordLoadPosePtr->loadRobotPose(handgesturePose, handgesturePosePath);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        ROS_INFO_STREAM("check pose file");
    }
    
    // 客户端
    pickClient = nh.serviceClient<pick_place_bridge::PickPlacePose>("/pick");
    fixedPickClient = nh.serviceClient<pick_place_bridge::PickPlacePose>("/fixed_pick");
    placeClient = nh.serviceClient<pick_place_bridge::PickPlacePose>("/place");
    moveClient = nh.serviceClient<pick_place_bridge::PickPlacePose>("/move");
    detectionClient = nh.serviceClient<hirop_msgs::detection>("/detection");
    getForceClient = nh.serviceClient<hirop_msgs::getForce>("getForce");
    moveSeqClient = nh.serviceClient<hirop_msgs::moveSeqIndex>("moveSeq");
    getPoseClient = nh.serviceClient<pick_place_bridge::recordPose>("recordPose");
    backHomeClient = nh.serviceClient<std_srvs::Empty>("back_home");
    // 服务
    handClawGrabDollServer = nh.advertiseService("handClaw_grabDoll", &ReceptionRobot::handClawGrabDollCallback, this);
    handgestureServer = nh.advertiseService("handClaw_shakeHand", &ReceptionRobot::handgestureSerCallback, this);
    PointTipServer = nh.advertiseService("list_point",  &ReceptionRobot::PointTipServerCallback, this);
    // 订阅
    objectArraySub = nh.subscribe<hirop_msgs::ObjectArray>("object_array", 1, &ReceptionRobot::objectCallBack, this);
    pedestrainSub = nh.subscribe("/pedestrian_detection", 10, &ReceptionRobot::pedestrainCallback, this);
    // handgestureSub = nh.subscribe("/handgesture_detection", 10, &ReceptionRobot::handgestureCallback, this);
    shakeSub = nh.subscribe("isShake", 10, &ReceptionRobot::shakeCallback, this);
    HandgestureModeSub = nh.subscribe("uipub_impedenceLive", 10, &ReceptionRobot::HandgestureModeCallback, this);
    updataPoseSub = nh.subscribe("updata_pose", 10, &ReceptionRobot::updataPoseCallback, this);
    backHomeSub = nh.subscribe("homePoint", 10, &ReceptionRobot::backHomeCallback, this);
    // 发布
    speedScalePub = nh.advertise<std_msgs::Bool>("speedScale", 10);
    detachObjectPub = nh.advertise<std_msgs::Empty>("detach_object", 10);
    isOpenFollowPub = nh.advertise<std_msgs::Bool>("is_follow", 10);
    freeStatusPub = nh.advertise<std_msgs::Bool>("rbCtlBusy_status", 10);
    isShake = false;
    HandgestureMode = false;
}

ReceptionRobot::~ReceptionRobot()
{
    delete recordLoadPosePtr;
    recordLoadPosePtr = nullptr;
}

bool ReceptionRobot::handClawGrabDollCallback(rb_msgAndSrv::rb_DoubleBool::Request& req, rb_msgAndSrv::rb_DoubleBool::Response& rep)
{
    test();
    rep.respond = true;
    return true;
}



///////////////回调//////////////////////
void ReceptionRobot::pedestrainCallback(const std_msgs::Bool::ConstPtr& msg)
{
    std_msgs::Bool speedMsg;
    speedMsg.data = true;
    speedScalePub.publish(speedMsg);
}


void ReceptionRobot::shakeCallback(const std_msgs::Bool::ConstPtr& msg)
{
    isShake = msg->data;
}

void ReceptionRobot::HandgestureModeCallback(const std_msgs::Bool::ConstPtr& msg)
{
    HandgestureMode = msg->data;
}

void ReceptionRobot::backHomeCallback(const std_msgs::Int8::ConstPtr& msg)
{
    backHome();
}

// 语音触发
// void ReceptionRobot::handgestureCallback(const std_msgs::Bool::ConstPtr& msg)
// {
//     // handgesture();
//     moveHandgesturePose();
//     checkHandgestureLoop();
// }

// UI触发
bool ReceptionRobot::handgestureSerCallback(rb_msgAndSrv::rb_DoubleBool::Request& req, rb_msgAndSrv::rb_DoubleBool::Response& rep)
{
    // handgesture();
    // moveHandgesturePose();
    pubStatus(BUSY);
    moveHandgesturePose(handgesturePose);
    pubStatus(!BUSY);
    checkHandgestureLoop();
	rep.respond = true;
	return true;
}

void ReceptionRobot::updataPoseCallback(const std_msgs::Int8::ConstPtr& msg)
{
    geometry_msgs::PoseStamped pose;
    pick_place_bridge::recordPose srv;
    getPoseClient.call(srv);
    pose = srv.response.pose;
    switch (msg->data)
    {
        case 0:
            recordLoadPosePtr->recordRobotPose(pose, detectionPosePath);
            detectionPose = pose;
            break;
        case 1:
            recordLoadPosePtr->recordRobotPose(pose, handgesturePosePath);
            handgesturePose = pose;
            break;
    }
}

bool ReceptionRobot::PointTipServerCallback(reception_robot::listPose::Request& req, reception_robot::listPose::Response& rep)
{
    rep.pose.resize(2);
    rep.pose[0] = "detection pose index: 0";
    rep.pose[1] = "handgesture pose index: 1";
    return true;
}

void ReceptionRobot::objectCallBack(const hirop_msgs::ObjectArray::ConstPtr& msg)
{
    ros::WallDuration(0.5).sleep();
    pubStatus(BUSY);
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
    int timeCnt = 40;
    while (cnt < timeCnt)
    {
        if(!checkForce() || cnt == (timeCnt - 1))
        {
            backHome();
            setFiveFightPose(HOME);
            break;
        }
        cnt++;
        ros::WallDuration(0.25).sleep();
    }
    pubStatus(!BUSY);
}

/////////////////// 实现//////////////

void ReceptionRobot::test()
{
    if(HandgestureMode)
        return;
    pubStatus(BUSY);
    std_msgs::Bool msg;
    msg.data = false;
    isOpenFollowPub.publish(msg);
    pick_place_bridge::PickPlacePose pose;
    // pose.request.Pose.header.frame_id = "world";
    // pose.request.Pose.pose.position.x = 0.0453143;
    // pose.request.Pose.pose.position.y = -0.838108;
    // pose.request.Pose.pose.position.z = 1.50026;

    // pose.request.Pose.pose.orientation.x = 0.248138;
    // pose.request.Pose.pose.orientation.y = 0.312854;
    // pose.request.Pose.pose.orientation.z = -0.590081;
    // pose.request.Pose.pose.orientation.w = 0.70168;
    pose.request.Pose = detectionPose;
    moveClient.call(pose);
    hirop_msgs::detection d;
    d.request.detectorName = "Yolo6d";
    d.request.detectorType = 1;
    d.request.objectName = "doll";
    detectionClient.call(d);
    pubStatus(!BUSY);
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

bool ReceptionRobot::moveHandgesturePose(geometry_msgs::PoseStamped pose)
{
    pick_place_bridge::PickPlacePose targetPose;
    targetPose.request.Pose = pose;
    moveClient.call(targetPose);
    return targetPose.response.result;
}

bool ReceptionRobot::moveHandgesturePose()
{
    geometry_msgs::PoseStamped targetPose;
    pick_place_bridge::PickPlacePose Pose;
    targetPose.header.frame_id = "world";
    targetPose.pose.position.x = 0.80;
    targetPose.pose.position.y = 0.0;
    targetPose.pose.position.z = 1.4;
    targetPose.pose.orientation.w = 1;
    Pose.request.Pose = targetPose;
    moveClient.call(Pose);
    std_msgs::Bool msg;
    msg.data = true;
    isOpenFollowPub.publish(msg);
    return Pose.response.result;
}

bool ReceptionRobot::checkHandgestureLoop()
{
    int flag = 0;
    // 等待阻抗开启(10s),开启退出
    for(int i=0; i<40; ++i)
    {
        if(HandgestureMode)
            break;
        ros::WallDuration(0.25).sleep();
    }
    while (ros::ok() && HandgestureMode)
    {
        if((checkForce() || isShake) && flag == 0)
        {
            flag = 1;
            ROS_INFO_STREAM("set five fight pose: " << SHAKE);
            setFiveFightPose(SHAKE);
        }
        if((!checkForce() || !isShake) && flag == 1)
        {
            flag = 0;
            ROS_INFO_STREAM("set five fight pose: " << HOME);
            setFiveFightPose(HOME);
        }
    }
    if(!HandgestureMode)
    {
        backHome();
    }
    setFiveFightPose(HOME);
    flag = 0;
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
    setFiveFightPose(SHAKE_PREPARE);
    ros::WallDuration(0.5).sleep();
    int timeCnt = 12;
    while (cnt < timeCnt)
    {
        // 力量大于阈值, 或者六轴传感器有数据波动, 或者超过3S, 闭合
        if(checkForce() || isShake || cnt == (timeCnt - 1))
        {
            ROS_INFO_STREAM("set five fight pose: " << SHAKE);
            setFiveFightPose(SHAKE);
            break;
        }
        ros::WallDuration(0.25).sleep();
        cnt ++;
    }
    cnt = 0;
    timeCnt = 16;
    while (cnt < 16)
    {
        // 力量小于于阈值, 或者六轴传感器有数据无波动, 或者超过4S, 打开
        if(!checkForce() || !isShake || cnt == (timeCnt - 1))
        {
            ROS_INFO_STREAM("set five fight pose: " << HOME);
            setFiveFightPose(HOME);
            break;
        }
        ros::WallDuration(0.25).sleep();
        cnt ++;
    }
    ros::WallDuration(1).sleep();
    // system("rostopic pub -1 /back_home std_msgs/Int8 \"data: 0\""); 
    ROS_INFO_STREAM("----handgesture over ----");
    return true;
}


void ReceptionRobot::pubStatus(bool isFree)
{
    std_msgs::Bool msg;
    msg.data = isFree;
    freeStatusPub.publish(msg);
}

void ReceptionRobot::backHome()
{
    pubStatus(BUSY);
    std_srvs::Empty srv;
    backHomeClient.call(srv);
    pubStatus(!BUSY);
}