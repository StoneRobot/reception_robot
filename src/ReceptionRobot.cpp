#include "reception_robot/ReceptionRobot.h"

ReceptionRobot::ReceptionRobot(ros::NodeHandle& n)
:nh{n}
{
    recordLoadPosePtr = new recordLoadPose(nh);
    nh.getParam("/reception_robot/detectionPose", detectionPosePath);
    nh.getParam("/reception_robot/handgesturePose", handgesturePosePath);
    nh.getParam("/reception_robot/ok", okPosePath);
    nh.getParam("/reception_robot/wave", WavePosePath);
    try
    {
        /* code */
        recordLoadPosePtr->loadRobotPose(detectionPose, detectionPosePath);
        recordLoadPosePtr->loadRobotPose(handgesturePose, handgesturePosePath);
        recordLoadPosePtr->loadRobotPose(OKPose, okPosePath);
        recordLoadPosePtr->loadRobotPose(wavePose, WavePosePath);
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
    backHomeClient = nh.serviceClient<std_srvs::SetBool>("/back_home");
    // 服务
    // handClawGrabDollServer = nh.advertiseService("handClaw_grabDoll", &ReceptionRobot::handClawGrabDollCallback, this);
    handgestureServer = nh.advertiseService("handClaw_shakeHand", &ReceptionRobot::handgestureSerCallback, this);
    PointTipServer = nh.advertiseService("list_point",  &ReceptionRobot::PointTipServerCallback, this);
    handDetectionDollServer = nh.advertiseService("handClaw_detectDoll", &ReceptionRobot::handDetectionDollCallback, this);
    waveServer = nh.advertiseService("wave_pose", &ReceptionRobot::waveCallback, this);
    okServer = nh.advertiseService("ok_pose", &ReceptionRobot::okCallback, this);
    // 订阅
    objectArraySub = nh.subscribe<hirop_msgs::ObjectArray>("object_array", 1, &ReceptionRobot::objectCallBack, this);
    pedestrainSub = nh.subscribe("/pedestrian_detection", 10, &ReceptionRobot::pedestrainCallback, this);
    // handgestureSub = nh.subscribe("/handgesture_detection", 10, &ReceptionRobot::handgestureCallback, this);
    shakeSub = nh.subscribe("isShake", 10, &ReceptionRobot::shakeCallback, this);
    HandgestureModeSub = nh.subscribe("uipub_impedenceLive", 10, &ReceptionRobot::HandgestureModeCallback, this);
    updataPoseSub = nh.subscribe("updata_pose", 10, &ReceptionRobot::updataPoseCallback, this);
    backHomeSub = nh.subscribe("homePoint", 10, &ReceptionRobot::backHomeCallback, this);
    robotStatusSub = nh.subscribe("robot_status", 10, &ReceptionRobot::robotStatusCallback, this);
    shakeOverSub = nh.subscribe("shake_over", 10, &ReceptionRobot::shakeOverCallback, this);
    // 发布
    speedScalePub = nh.advertise<std_msgs::Float32>("speedScale", 10);
    detachObjectPub = nh.advertise<std_msgs::Empty>("detach_object", 10);
    isOpenFollowPub = nh.advertise<std_msgs::Bool>("is_follow", 10);
    freeStatusPub = nh.advertise<std_msgs::Bool>("rbCtlBusy_status", 10);
    isSetConstrainPub = nh.advertise<std_msgs::Bool>("is_setConstrain", 10);
    isShake = false;
    HandgestureMode = false;
    isShakeOver = false;
    robotStatus = true;
}

ReceptionRobot::~ReceptionRobot()
{
    delete recordLoadPosePtr;
    recordLoadPosePtr = nullptr;
}

///////////////回调//////////////////////
void ReceptionRobot::pedestrainCallback(const std_msgs::Bool::ConstPtr& msg)
{
    std_msgs::Float32 speedMsg;
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
    if(backHome())
    {
        ROS_INFO_STREAM("back home success");
    }
    else
    {
        ROS_INFO_STREAM("back home faild");
    }
}

void ReceptionRobot::robotStatusCallback(const std_msgs::Bool::ConstPtr& msg)
{
    robotStatus = msg->data;
}

// UI触发
bool ReceptionRobot::handgestureSerCallback(rb_msgAndSrv::rb_DoubleBool::Request& req, rb_msgAndSrv::rb_DoubleBool::Response& rep)
{
    // handgesture();
    // moveHandgesturePose();
    pubConstrainState(true);
    movePose(handgesturePose);
    rep.respond = checkHandgestureLoop();
    pubConstrainState(false);
	return rep.respond;
}

bool ReceptionRobot::handDetectionDollCallback(rb_msgAndSrv::rb_DoubleBool::Request& req, rb_msgAndSrv::rb_DoubleBool::Response& rep)
{
    pubConstrainState(false);
    rep.respond = toDetect();
    ROS_INFO_STREAM("handDetectionDollCallback:" << rep.respond << " ");
    return rep.respond;
}

bool ReceptionRobot::waveCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& rep)
{
    if(req.data)
    {
        rep.success = wave();
        return rep.success;
    }
    return false;
}

bool ReceptionRobot::okCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& rep)
{
    bool flag=false;
    if(req.data)
    {
        flag = toOkPose();
        rep.success = flag;
    }
    return flag;
}

/**不用*/
// bool ReceptionRobot::handClawGrabDollCallback(rb_msgAndSrv::rb_DoubleBool::Request& req, rb_msgAndSrv::rb_DoubleBool::Response& rep)
// {
    // actionGrasp();
    // rep.respond = true;
    // return true;
// }

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
        case 2:
            recordLoadPosePtr->recordRobotPose(pose, okPosePath);
            OKPose = pose;
            break;
        case 3:
            recordLoadPosePtr->recordRobotPose(pose, WavePosePath);
            wavePose = pose;
            break;
    }
}

bool ReceptionRobot::PointTipServerCallback(reception_robot::listPose::Request& req, reception_robot::listPose::Response& rep)
{
    rep.pose.resize(4);
    rep.pose[0] = "detection pose index: 0";
    rep.pose[1] = "handgesture pose index: 1";
    rep.pose[2] = "OK pose index: 2";
    rep.pose[3] = "wave pose index: 3";
    return true;
}

void ReceptionRobot::objectCallBack(const hirop_msgs::ObjectArray::ConstPtr& msg)
{

    // std::vector<hirop_msgs::ObjectInfo>().swap(objectPose);
    // objectPose.resize(msg->objects.size());
    // for(int i=0; i < msg->objects.size(); ++i)
    // {
    //     objectPose[i] = msg->objects[i];
        
    // }
}

void ReceptionRobot::actionGrasp()
{
    ros::WallDuration(0.5).sleep();
    // pubStatus(BUSY);
    for(int i=0; i<objectPose.size(); ++i)
    {
        // 坐标转换
        geometry_msgs::PoseStamped pose = objectPose[0].pose;
        transformFrame(pose, "world");
        pick_place_bridge::PickPlacePose pickPose;
        // 调用Pick
        pickPose.request.Pose = pose;


        pickPose.request.Pose.header.frame_id = "world";
        pickPose.request.Pose.pose.position.x = 0.576525;
        pickPose.request.Pose.pose.position.y = -0.357523;
        pickPose.request.Pose.pose.position.z = 1.24766;
        pickPose.request.Pose.pose.orientation.x = 0;
        pickPose.request.Pose.pose.orientation.y = 0;
        pickPose.request.Pose.pose.orientation.z = 0;
        pickPose.request.Pose.pose.orientation.w = 1;
        
        /**************************/
        // pickPose.request.Pose.header.frame_id = "world";
        // pickPose.request.Pose.pose.position.x = 0.236844;
        // pickPose.request.Pose.pose.position.y = -0.403213;
        // pickPose.request.Pose.pose.position.z = 1.36805;
        // pickPose.request.Pose.pose.orientation.x = -0.0144055;
        // pickPose.request.Pose.pose.orientation.y = -0.00911873;
        // pickPose.request.Pose.pose.orientation.z = 0.0931309;
        // pickPose.request.Pose.pose.orientation.w = 0.995507;
        /**************************/
        // pickClient.call(pickPose);
        // fixedPickClient.call(pickPose);
        pick_place_bridge::PickPlacePose placePose;

        /**************************/
        placePose.request.Pose.header.frame_id = "world";
        placePose.request.Pose.pose.position.x = 0.379353;
        placePose.request.Pose.pose.position.y = 0.687491;
        placePose.request.Pose.pose.position.z = 1.45898;
        // placePose.request.Pose.pose.orientation.x = 0.0282123;
        // placePose.request.Pose.pose.orientation.y = 0.0537069;
        // placePose.request.Pose.pose.orientation.z = 0.704343;
        // placePose.request.Pose.pose.orientation.w = 0.707263;
        placePose.request.Pose.pose.orientation.x = 0.0426387;
        placePose.request.Pose.pose.orientation.y = 0.0432133;
        placePose.request.Pose.pose.orientation.z = 0.88249;
        placePose.request.Pose.pose.orientation.w = 0.466398;
        /**************************/

        /**************************/
        // placePose.request.Pose.pose.position.x = 0.95;
        // placePose.request.Pose.pose.position.y = -0.20;
        // placePose.request.Pose.pose.position.z = 1.51;
        // placePose.request.Pose.pose.orientation.x = -0.038076;
        // placePose.request.Pose.pose.orientation.y = 0.0136285;
        // placePose.request.Pose.pose.orientation.z = -0.0250326;
        // placePose.request.Pose.pose.orientation.w = 0.998868;
        /**************************/


        // moveClient.call(placePose);
        // pick_place_bridge::PickPlacePose srv;
        // std_msgs::Empty emptryMsg;
        // detachObjectPub.publish(emptryMsg);
        
        // placeClient.call(placePose);

        backHome();
        setFiveFightPose(HOME);

        // int cnt = 0;
        // int timeCnt = 40;
        // while (ros::ok())
        // {
        //     // !checkForce() || 
        //     if(cnt == (timeCnt - 1))
        //     {
        //         break;
        //     }
        //     cnt++;
        //     ros::WallDuration(0.25).sleep();
        // }

    }
    // pubStatus(!BUSY);
}

void ReceptionRobot::shakeOverCallback(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO_STREAM("--------------------------->> recevice shake over");
    isShakeOver = msg->data;
    ROS_INFO_STREAM(isShakeOver);
}

/////////////////// 实现//////////////



bool ReceptionRobot::toDetect()
{
    if(HandgestureMode)
        return false;
    // pubStatus(BUSY);
    setFiveFightPose(TAKE_PHOTO);
    followSwitch(false);
    pick_place_bridge::PickPlacePose pose;

    pose.request.Pose = detectionPose;
    if(!moveClient.call(pose))
    {
        return false;
    }
    else
    {
        return pose.response.result;
    }
}

bool ReceptionRobot::transformFrame(geometry_msgs::PoseStamped& poseStamped, std::string frame_id="world")
{
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
    if(poseStamped.header.frame_id == frame_id)
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
    try
    {
        std::vector<int> force = srv.response.finger_force;
        for(int i=0; i < srv.response.finger_force.size(); ++i)
        {
            if(i < 4)
            {
                if(srv.response.finger_force[i] > 50)
                    return true;
            }
            else
            {
                if(srv.response.finger_force[i] > 90)
                    return true;
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        ROS_INFO_STREAM("check five finger node");
        return false;
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

bool ReceptionRobot::movePose(geometry_msgs::PoseStamped pose)
{
    // pubStatus(BUSY);
    bool flag;
    pick_place_bridge::PickPlacePose targetPose;
    targetPose.request.Pose = pose;
    if(!moveClient.call(targetPose))
    {
        ROS_INFO_STREAM("check move pose server");
        flag = false;
    }
    else
    {
        if(targetPose.response.result)
        {
            ROS_INFO_STREAM("move pose SUCCESS");
        }
        else
        {
            ROS_INFO_STREAM("move pose FAILED");
        }
        flag  = targetPose.response.result;
    }
    // pubStatus(!BUSY);
    return flag;
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
    followSwitch(true);
    return Pose.response.result;
}

bool ReceptionRobot::checkHandgestureLoop()
{
    ROS_INFO_STREAM("----------------------------");
    followSwitch(true);
    while(ros::ok() && !HandgestureMode);
    setFiveFightPose(SHAKE_PREPARE);
    int cnt = 0;
    // 等待结束
    while (ros::ok())
    {
        // 五指全部没感受到力矩 !checkForce() ||  || 超时10s || 六轴没有力矩 || 退出握手模式 cnt == 40 || isShakeOver || 
        if(!HandgestureMode)
        {
            setFiveFightPose(HOME);
            ROS_INFO_STREAM("exit setFiveFightPose ...");
            break;
        }
        ros::WallDuration(0.25).sleep();
        ++cnt;
    }
    followSwitch(false);
    // 等待退出握手模式, 且机器人状态正常
    for(int j=0; j<80; ++j)
    {
        ROS_INFO_STREAM("---------isShakeOver:" << isShakeOver);
        ROS_INFO_STREAM("------------wait for isShakeOver --------------");
        if(isShakeOver)
        {
            bool flag[2];
            ROS_INFO_STREAM("------------  ShakeOver --------------");
            setFiveFightPose(OK);
            flag[0] = movePose(OKPose);
            ros::WallDuration(1.5).sleep();
            flag[1] = backHome();
            setFiveFightPose(HOME);
            return (flag[0] && flag[1]);
        }
        ros::WallDuration(0.25).sleep();
    }
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
    setFiveFightPose(SHAKE_PREPARE);
    ros::WallDuration(0.5).sleep();
    int timeCnt = 12;
    while (cnt < timeCnt)
    {
        // 力量大于阈值 || 六轴传感器有数据波动 || 超过3S
        if(checkForce() || isShake || cnt == (timeCnt - 1))
        {
            // ROS_INFO_STREAM("set five fight pose: " << SHAKE);
            // setFiveFightPose(SHAKE);
            break;
        }
        ros::WallDuration(0.25).sleep();
        cnt ++;
    }
    cnt = 0;
    timeCnt = 16;
    while (cnt < 16)
    {
        // 五指没有感受到力 || 六轴传感器有数据无波动 || 超过4S
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

bool ReceptionRobot::backHome()
{
    // pubStatus(BUSY);
    std_srvs::SetBool srv;
    srv.request.data = true;
    if(!backHomeClient.call(srv))
    {
        ROS_INFO_STREAM("check back home server");
        return false;
    }
    else
    {
        if(srv.response.success)
        {
            ROS_INFO_STREAM("back home success");
        }
        else
        {
            ROS_INFO_STREAM("back home faild");
        }
        return srv.response.success;
    }
    // pubStatus(!BUSY);
}

void ReceptionRobot::followSwitch(bool onOff)
{
    std_msgs::Bool msg;
    msg.data = onOff;
    isOpenFollowPub.publish(msg);
}

void ReceptionRobot::pubConstrainState(bool isSet)
{
    std_msgs::Bool msg;
    msg.data = isSet;
    isSetConstrainPub.publish(msg);
}

bool ReceptionRobot::wave()
{
    pick_place_bridge::PickPlacePose srv;
    srv.request.Pose = wavePose;
    if(moveClient.call(srv))
    {
        setFiveFightPose(HOME);
        if(srv.response.result)
        {
            ROS_INFO_STREAM("Reached the wave position successfully");
        }
        else
        {
            ROS_INFO_STREAM("Reached the wave position failed");
        }
        return srv.response.result;
    }
    return false;
}

bool ReceptionRobot::toOkPose()
{
    bool flag=false;
    pick_place_bridge::PickPlacePose srv;
    srv.request.Pose = OKPose;
    setFiveFightPose(OK);
    if(moveClient.call(srv))
    {
        if(srv.response.result)
        {
            ROS_INFO_STREAM("Reached the OK position successfully");
        }
        else
        {
            ROS_INFO_STREAM("Reached the OK position failed");
        }
        flag = srv.response.result;
    }
    setFiveFightPose(HOME);
    return flag;
}
