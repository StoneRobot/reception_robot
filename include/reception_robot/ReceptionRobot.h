#pragma once
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <stdlib.h>
#include <stdio.h>


#include "pick_place_bridge/PickPlacePose.h"

#include "hirop_msgs/ObjectArray.h"
#include "hirop_msgs/detection.h"
#include "hirop_msgs/getForce.h"
#include "hirop_msgs/moveSeqIndex.h"

#include "rb_msgAndSrv/rb_DoubleBool.h"

class ReceptionRobot
{
public:
    ReceptionRobot(ros::NodeHandle& n);

    /**
     * @brief 位置变换,默认转到世界坐标系
     * @param poseStamped 需要转换的坐标及完成后返回的坐标
     * @param frame_id 转换后的参考坐标系
     * @return 是否转换成功
    */
    bool transformFrame(geometry_msgs::PoseStamped& poseStamped, std::string frame_id);

    /**
     * @brief 实现握手
    */
    bool handgesture();


    /**
     * @brief 用于抓娃娃测试
    */
    void test();

    /**
     * @brief 检查五指力矩是否超过阈值
     * @return 如果超过返回true
    */
    bool checkForce();


    /**
    * @brief 设置夹爪动作
    * @param index 动作索引
    * @return 是否设置成功
    */
    bool setFiveFightPose(int index);
private:
    ros::NodeHandle& nh;
    // 服务回调
    bool handClawGrabDollCallback(rb_msgAndSrv::rb_DoubleBool::Request& req, rb_msgAndSrv::rb_DoubleBool::Response& rep);
    bool handgestureSerCallback(rb_msgAndSrv::rb_DoubleBool::Request& req, rb_msgAndSrv::rb_DoubleBool::Response& rep);
    // 话题回调
    void pedestrainCallback(const std_msgs::Bool::ConstPtr& msg);
    void objectCallBack(const hirop_msgs::ObjectArray::ConstPtr& msg);
    void handgestureCallback(const std_msgs::Bool::ConstPtr& msg);
    void shakeCallback(const std_msgs::Bool::ConstPtr& msg);

    // 客户端 
    ros::ServiceClient pickClient;
    ros::ServiceClient fixedPickClient;
    ros::ServiceClient placeClient;
    ros::ServiceClient moveClient;
    ros::ServiceClient detectionClient;
    ros::ServiceClient getForceClient;
    ros::ServiceClient moveSeqClient;
    // 服务器
    ros::ServiceServer handClawGrabDollServer; 
    ros::ServiceServer handgestureServer;
    // 订阅
    ros::Subscriber objectArraySub;
    ros::Subscriber pedestrainSub;
    ros::Subscriber handgestureSub;
    ros::Subscriber shakeSub;
    // 发布
    ros::Publisher speedScalePub;
    ros::Publisher detachObjectPub;
    // 判断六轴传感器是否有摇动信号
    bool isShake;
    const int SHAKE = 0;
    const int GRASP = 1;
    const int OK = 2;
    const int HOME = 3;
};


