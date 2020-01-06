//
// Created by LHospital
//

// RealSenseT265_node.h
// 1. 从RealSense T265中读取数据
// 2. 将摄像头图像转为OpenCV的Mat
// 3. 以ROS的格式发布图像
// 4. 以ROS的格式发布IMU的数据
// 5. 以ROS的格式发布相机的pose
// 6. 以ROS格式发布IMU、相机、两个摄像头之间的坐标转换

#ifndef _REALSENSET265_NODE_H
#define _REALSENSET265_NODE_H 

#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>

// Update: 2020.1.5
/**
 * version: 1.0
 * 功能：
 * 1. 开启Intel RealSense T265 Tracking Camera设备，
 * 2. 从设备中获取左右鱼眼视频流并以rostopic的格式发布
 * 3. 从设备中获取陀螺仪与加速度计数据，同步为IMU数据流并以rostopic的格式同步发布
 * TODO: 
 * 1. 发布设备自身的里程计
 * 2. 增加多种陀螺仪与加速度计的同步策略
 * 3. 能够根据launch文件的参数调整发布频率
 * 4. 为Image打上时间戳
 */

#define MAX_IMAGE_LENGTH 2000
#define MAX_GYRO_LENGTH 15000
#define MAX_ACCEL_LENGTH 5000
#define MAX_IMU_LENGTH 5000

typedef std::pair<double, cv::Mat> ImgMeta;
typedef std::pair<double, rs2_vector> GyroMeta;
typedef std::pair<rs2_vector, rs2_vector> ImuDataMeta;
typedef std::pair<double, ImuDataMeta> ImuMeta;
typedef struct float3 {
    float x;
    float y;
    float z;
    float3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
}float3;
typedef struct RS265DataQueue {
    std::vector<GyroMeta> gyros_que_;
    std::vector<rs2_vector> accel_que_;
    std::queue<ImuMeta> syncImu_que_;

    std::queue<ImgMeta> leftFishEye_que_;
    std::queue<ImgMeta> rightFishEye_que_;

    std::map<int, int> counters;
    std::map<int, std::string> streamNames;
}RS265DataQueue;

class SyncImuPublisher {
public:
    SyncImuPublisher(ros::NodeHandle *nh, RS265DataQueue *data_que, std::mutex *mtx) {
        nh_ = nh;
        data_que_ = data_que;
        mtx_ = mtx;
    }

    void run();
private:
    ros::NodeHandle *nh_;
    RS265DataQueue *data_que_;
    std::mutex *mtx_;
    // ros::Publisher imu_pub_;
};

class FishEyePublisher {
public:
    FishEyePublisher(ros::NodeHandle *nh, RS265DataQueue *data_que, std::mutex *mtx) {
        nh_ = nh;
        data_que_ = data_que;
        mtx_ = mtx;
    }

    void run();

private:
    ros::NodeHandle *nh_;
    RS265DataQueue *data_que_;
    std::mutex *mtx_;
};

class Streamer {
public:
    Streamer(rs2::pipeline *pp, rs2::config *cfg, RS265DataQueue *data_queue, std::mutex *mutex) {
        pp_ = pp;
        cfg_ = cfg;
        data_queue_ = data_queue;
        mutex_ = mutex;
    }
    ~Streamer() {
        stop();
    };

    void run();
    void stop();
private:
    void FillDataQueue_callback(const rs2::frame &frame);
private:
    rs2::pipeline *pp_;
    rs2::config *cfg_;
    RS265DataQueue *data_queue_;
    std::mutex *mutex_;
};

class RealSenseT265 {
public:
    RealSenseT265();
    ~RealSenseT265() = default;

    void init(int argc, char *argv[]);
    void run();

private:
    ros::NodeHandle nh_;
    
    ros::Publisher imu_pub_;
    ros::Publisher fishEye_pub_;

    // 数据队列
    RS265DataQueue data_queue_;

    std::mutex mutex;

    rs2::config cfg_;
    rs2::pipeline pipes_;

};

#endif	// _REALSENSET265_NODE_H