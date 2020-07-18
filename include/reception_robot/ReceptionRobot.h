#pragma once
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include <stdlib.h>
#include <stdio.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>


#include "pick_place_bridge/PickPlacePose.h"
#include "pick_place_bridge/recordPose.h"
#include "reception_robot/recordLoadPose.h"

#include "reception_robot/listPose.h"

#include "hirop_msgs/ObjectArray.h"
#include "hirop_msgs/detection.h"
#include "hirop_msgs/getForce.h"
#include "hirop_msgs/moveSeqIndex.h"

#include "rb_msgAndSrv/rb_DoubleBool.h"

class ReceptionRobot
{
public:
    ReceptionRobot(ros::NodeHandle& n);
    ~ReceptionRobot();

    /**
     * @brief 位置变换,默认转到世界坐标系
     * @param poseStamped 需要转换的坐标及完成后返回的坐标
     * @param frame_id 转换后的参考坐标系
     * @return 是否转换成功
    */
    bool transformFrame(geometry_msgs::PoseStamped& poseStamped, std::string frame_id);

    /**
     * @brief 去到机器人握手的姿势
     * @param pose 握手的姿态
    */
    bool movePose(geometry_msgs::PoseStamped pose);

    /**
     * @brief 去到机器人握手的姿势
    */
    bool moveHandgesturePose();

    /**
     * @brief 检查是否符合握手条件
    */
    bool checkHandgestureLoop();

    /**
     * @brief 实现握手
    */
    bool handgesture();


    /**
     * @brief 用于抓娃娃测试
    */
    bool toDetect();

    /**
     * @brief 检查五指力矩是否超过阈值
     * @return 如果超过返回true, 没力或者没开五指夹爪Node为false
    */
    bool checkForce();


    /**
    * @brief 设置夹爪动作
    * @param index 动作索引
    * const int GRASP = 2;
    * const int OK = 3;
    * const int HOME = 4;
    * const int SHAKE_PREPARE = 5;
    * const int TAKE_PHOTO = 6;
    * @return 是否设置成功
    */
    bool setFiveFightPose(int index);

    /**
     * @brief 发布机器人是否空闲
     * @param isFree 处于空闲时发True
    */
    void pubStatus(bool isFree);

    /**
     * @brief 回Home点
    */
    bool backHome();

    /**
     *  @brief 随动的开关
     *  @param true为开
    */
    void followSwitch(bool onOff);

    bool wave();

    bool toOkPose();
    
private:

    void actionGrasp();

    void pubConstrainState(bool isSet);

    recordLoadPose* recordLoadPosePtr;
    ros::NodeHandle& nh;
    // 服务回调
    // bool handClawGrabDollCallback(rb_msgAndSrv::rb_DoubleBool::Request& req, rb_msgAndSrv::rb_DoubleBool::Response& rep);
    bool handgestureSerCallback(rb_msgAndSrv::rb_DoubleBool::Request& req, rb_msgAndSrv::rb_DoubleBool::Response& rep);
    bool PointTipServerCallback(reception_robot::listPose::Request& req, reception_robot::listPose::Response& rep);
    bool handDetectionDollCallback(rb_msgAndSrv::rb_DoubleBool::Request& req, rb_msgAndSrv::rb_DoubleBool::Response& rep);
    bool waveCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& rep);
    bool okCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& rep);
    // 话题回调
    void pedestrainCallback(const std_msgs::Bool::ConstPtr& msg);
    void objectCallBack(const hirop_msgs::ObjectArray::ConstPtr& msg);
    // void handgestureCallback(const std_msgs::Bool::ConstPtr& msg);
    void shakeCallback(const std_msgs::Bool::ConstPtr& msg);
    void HandgestureModeCallback(const std_msgs::Bool::ConstPtr& msg);
    void updataPoseCallback(const std_msgs::Int8::ConstPtr& msg);
    void backHomeCallback(const std_msgs::Int8::ConstPtr& msg);
    void robotStatusCallback(const std_msgs::Bool::ConstPtr& msg);
    void shakeOverCallback(const std_msgs::Bool::ConstPtr& msg);

    /****** 客户端 ******/
    ros::ServiceClient pickClient;
    ros::ServiceClient fixedPickClient;
    ros::ServiceClient placeClient;
    ros::ServiceClient moveClient;
    ros::ServiceClient detectionClient;
    ros::ServiceClient getForceClient;
    ros::ServiceClient moveSeqClient;
    ros::ServiceClient getPoseClient;
    ros::ServiceClient backHomeClient;
    /****** 服务器 ******/
    // 检测
    ros::ServiceServer handDetectionDollServer;
    // 抓娃娃
    // ros::ServiceServer handClawGrabDollServer; 
    // 握手
    ros::ServiceServer handgestureServer;
    // 记录点位的提示
    ros::ServiceServer PointTipServer;
    ros::ServiceServer waveServer;
    ros::ServiceServer okServer;

    /****** 订阅 ******/
    // 抓取物体抓取姿态
    ros::Subscriber objectArraySub;
    // 行人检测
    ros::Subscriber pedestrainSub;
    // 握手信号(嘉辉)
    // ros::Subscriber handgestureSub;
    // 是否有人在握手
    ros::Subscriber shakeSub;
    // 是否要处于握手模式(处于阻抗)
    ros::Subscriber HandgestureModeSub;
    // 更新检测和握手的点位
    ros::Subscriber updataPoseSub;
    // 中转,获取繁忙信号
    ros::Subscriber backHomeSub;
    // 订阅机器人状态
    ros::Subscriber robotStatusSub;
    ros::Subscriber shakeOverSub;


    /****** 发布 ******/
    // 调解速度
    ros::Publisher speedScalePub;
    // 从机器人上移除物体
    ros::Publisher detachObjectPub;
    // 空闲状态发布
    ros::Publisher freeStatusPub;
    // 开启阻抗状态(不用)
    ros::Publisher isOpenFollowPub;
    ros::Publisher isSetConstrainPub;


    // 判断六轴传感器是否有摇动信号
    bool isShake;

    // 是否处于握手模式
    bool HandgestureMode;

    // 结束信号
    bool isShakeOver;

    // 机器人故障状态
    bool robotStatus;

    /**** 五指夹爪的Pose的索引,从1开始****/
    // const int SHAKE = 1;
    const int GRASP = 2;
    const int OK = 3;
    const int HOME = 4;
    const int SHAKE_PREPARE = 5;
    const int TAKE_PHOTO = 6;
    /**** 点位路径 ****/
    // 检测点位存储路径
    std::string detectionPosePath;
    // 握手点位存储路径
    std::string handgesturePosePath;
    // OK 点位路径
    std::string okPosePath;
    std::string WavePosePath;

    // 检测点位
    geometry_msgs::PoseStamped detectionPose;
    //
    // 握手点位
    geometry_msgs::PoseStamped handgesturePose;
    // OK点位
    geometry_msgs::PoseStamped OKPose;
    geometry_msgs::PoseStamped wavePose;
    // 工作信号
    const bool BUSY = true;
    std::vector<hirop_msgs::ObjectInfo> objectPose;
};


