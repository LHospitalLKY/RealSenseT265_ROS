//
// Created by LHospital
//

#include "../include/RealSense/RealSenseT265_node.h"

#include <iostream>

// #define DEBUG

void Streamer::run() {
    // 将类成员函数变为回调函数
    auto call_back_inner = [this](const rs2::frame &frame) {
        FillDataQueue_callback(frame);
    };
    pp_ -> start(*cfg_, call_back_inner);
}

void Streamer::stop() {
    // pp_ -> stop();
}

void Streamer::FillDataQueue_callback(const rs2::frame &frame) {
    std::lock_guard<std::mutex> lock(*mutex_);
    std::cout.setf(std::ios::fixed, std::ios::floatfield);

    // 图像
    if(frame.get_profile().stream_type() == RS2_STREAM_FISHEYE) {
#ifdef DEBUG
        LOG(INFO) << "Video Frame is captured!";
#endif
        rs2::frameset fs = frame.as<rs2::frameset>();
        rs2::video_frame img_tmp1 = fs.get_fisheye_frame(1);
        rs2::video_frame img_tmp2 = fs.get_fisheye_frame(2);
        double img_time = fs.get_timestamp();
        cv::Mat img_1(cv::Size(848, 800), CV_8UC1, (void*)(img_tmp1.get_data()), cv::Mat::AUTO_STEP);
        cv::Mat img_2(cv::Size(848, 800), CV_8UC1, (void*)(img_tmp2.get_data()), cv::Mat::AUTO_STEP);
        // TODO: 根据实际情况更改img_1和img_2
        ImgMeta leftImgMeta(img_time, img_1);
        ImgMeta rightImgMeta(img_time, img_2);
        data_queue_ -> leftFishEye_que_.push(leftImgMeta);
        data_queue_ -> rightFishEye_que_.push(rightImgMeta);
        // 若长度大于队列最大长度
        if(data_queue_ -> leftFishEye_que_.size() > MAX_IMAGE_LENGTH) {
            data_queue_ -> leftFishEye_que_.pop();
            data_queue_ -> rightFishEye_que_.pop();
            data_queue_ -> counters[0]--;
            data_queue_ -> counters[1]--;
        }

        data_queue_ -> counters[0]++;
        data_queue_ -> counters[1]++;
    }
    // IMU数据
    if(frame.get_profile().stream_type() == RS2_STREAM_GYRO) {
#ifdef DEBUG
        LOG(INFO) << "Gyroscope Frame is captured!";
#endif
        double gyro_time = frame.get_timestamp();
        auto gyro_tmp = reinterpret_cast<const float3*>(frame.get_data());
        rs2_vector gyro_data;
        gyro_data.x = gyro_tmp -> x;
        gyro_data.y = gyro_tmp -> y;
        gyro_data.z = gyro_tmp -> z;
        GyroMeta gyro_meta_tmp;
        gyro_meta_tmp.first = gyro_time;
        gyro_meta_tmp.second = gyro_data;
        data_queue_ -> gyros_que_.push_back(gyro_meta_tmp);
        
        if(data_queue_ -> gyros_que_.size() > MAX_GYRO_LENGTH) {
            std::vector<GyroMeta>::iterator gyros_begin = data_queue_ -> gyros_que_.begin();
            data_queue_ -> gyros_que_.erase(gyros_begin);
        }
    }
    if(frame.get_profile().stream_type() == RS2_STREAM_ACCEL) {
#ifdef DEBUG
        LOG(INFO) << "Accelerometer Frame is captured!";
#endif
        double accel_time = frame.get_timestamp();
        auto accel_tmp = reinterpret_cast<const float3*>(frame.get_data());
        rs2_vector accel_data;
        accel_data.x = accel_tmp -> x;
        accel_data.y = accel_tmp -> y;
        accel_data.z = accel_tmp -> z;
        
        // 加速度计与陀螺仪时间配对
        rs2_vector gyro_data;
        double gyro_time = -1;
        double min_time_diff = 9999;
        // 遍历gyro
        std::vector<GyroMeta> &gyros_que = data_queue_ -> gyros_que_;
        for(int i = 0; i < gyros_que.size(); i++) {
            double time_diff = std::abs(accel_time - gyros_que[i].first);
            if(time_diff < min_time_diff) {
                min_time_diff = time_diff;
                gyro_time = gyros_que[i].first;
                gyro_data = gyros_que[i].second;
            }
        }
#ifdef DEBUG
        std::cout << "Gyro Time: " << gyro_time << "\n";
        std::cout << "Accel Time: " << accel_time << "\n";
#endif
        ImuDataMeta imuData_meta(gyro_data, accel_data);
        ImuMeta imu_meta(accel_time, imuData_meta);

        data_queue_ -> syncImu_que_.push(imu_meta);
        gyros_que.clear();

        if(data_queue_ -> syncImu_que_.size() > MAX_IMU_LENGTH) {
            data_queue_ -> syncImu_que_.pop();
            data_queue_ -> counters[2]--;
            data_queue_ -> counters[5]--;
        }
        data_queue_ -> counters[2]++;
        data_queue_ -> counters[5]++;
    }
}

// SyncImuPublisher
void SyncImuPublisher::run() {
    ros::Publisher imu_pub_;
    imu_pub_ = nh_ -> advertise<sensor_msgs::Imu>("realsense/imu", 1000);
    sensor_msgs::Imu imu;
    imu.header.frame_id = "rst265";
    while(ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::lock_guard<std::mutex> lock(*mtx_);
        if(data_que_ -> syncImu_que_.size() > 0) {
            ImuMeta imu_meta = data_que_ -> syncImu_que_.front();
            imu.header.stamp = ros::Time(imu_meta.first/1000);
            imu.angular_velocity.x = imu_meta.second.first.x;
            imu.angular_velocity.y = imu_meta.second.first.y;
            imu.angular_velocity.z = imu_meta.second.first.z;
            imu.linear_acceleration.x = imu_meta.second.second.x;
            imu.linear_acceleration.y = imu_meta.second.second.y;
            imu.linear_acceleration.z = imu_meta.second.second.z;

            imu_pub_.publish(imu);
            ROS_INFO_STREAM("Imu Data has been published! time: " << imu.header.stamp);

            data_que_ -> syncImu_que_.pop();
            data_que_ -> counters[2]--;
            data_que_ -> counters[5]--;
        }
    }
}

void FishEyePublisher::run() {
    ros::Publisher leftImg_pub_ = nh_ -> advertise<sensor_msgs::Image>("realsense/fisheye_left", 1000);;
    ros::Publisher rightImg_pub_ = nh_ -> advertise<sensor_msgs::Image>("realsense/fisheye_right", 1000);;

    sensor_msgs::ImagePtr leftImg_ros;
    sensor_msgs::ImagePtr rightImg_ros;

    cv::Mat leftImg_cv;
    cv::Mat rightImg_cv;
    
    while(ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        std::lock_guard<std::mutex> lock(*mtx_);
        if(data_que_ -> leftFishEye_que_.size() > 0 && data_que_ -> rightFishEye_que_.size() > 0) {
            double img_time = data_que_ -> leftFishEye_que_.front().first;
            leftImg_cv = data_que_ -> leftFishEye_que_.front().second;
            rightImg_cv = data_que_ -> rightFishEye_que_.front().second;
            cv_bridge::CvImage leftCv(std_msgs::Header(),  "mono8", leftImg_cv);
            cv_bridge::CvImage rightCv(std_msgs::Header(), "mono8", rightImg_cv);

#ifdef DEBUG
            std::cout << "Left Image Size: " << leftCv.image.size() << '\n';
            std::cout << "right Image Size: " << rightCv.image.size() << '\n';
            cv::imshow("left sight", leftImg_cv);
            cv::imshow("right sight: ", rightCv.image);
            cv::waitKey(10);
#endif

            leftImg_ros = leftCv.toImageMsg();
            leftImg_ros -> header.stamp = ros::Time(img_time/1000);
            rightImg_ros = rightCv.toImageMsg();
            rightImg_ros -> header.stamp = ros::Time(img_time/1000);

            leftImg_pub_.publish(leftImg_ros);
            rightImg_pub_.publish(rightImg_ros);

            ROS_INFO_STREAM("Fish Eye Images has been published! time: " << leftImg_ros -> header.stamp);

            data_que_ -> leftFishEye_que_.pop();
            data_que_ -> rightFishEye_que_.pop();
            data_que_ -> counters[0]--;
            data_que_ -> counters[1]--;
        }
    }
}

RealSenseT265::RealSenseT265() {}

void RealSenseT265::init(int argc, char *argv[]) {
    std::cout << "-------------RealSenseT265 initialization-------------" << std::endl;

    cfg_.enable_all_streams();

    // TODO: 增加参数选择的部分

    std::cout << "-------------RealSenseT265 initialize finish-----------" << std::endl;

}

void RealSenseT265::run() {
    Streamer st(&pipes_, &cfg_, &data_queue_, &mutex);
    SyncImuPublisher imuPub(&nh_, &data_queue_, &mutex);
    FishEyePublisher fishPub(&nh_, &data_queue_, &mutex);

    std::thread thread_dataStream(&Streamer::run, st);
    std::thread thread_syncImuPublisher(&SyncImuPublisher::run, imuPub);
    std::thread thread_fishEyePublisher(&FishEyePublisher::run, fishPub);

    thread_dataStream.join();
    thread_syncImuPublisher.join();
    thread_fishEyePublisher.join();
}
