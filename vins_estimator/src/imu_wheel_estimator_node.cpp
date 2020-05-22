#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"



Estimator estimator;

std::condition_variable con;
double current_time = -1;
double current_time_wh = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<nav_msgs::Odometry> wh_odom_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::PointCloudConstPtr> relo_buf;
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;

//variables for adding wheel odom
double latest_wh_time;
bool init_wh  = 1;
Eigen::Vector3d vel_0;
Eigen::Vector3d wh_gyr_0;
nav_msgs::Odometry wh_odom_g;


void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void predict_with_wheel(const nav_msgs::Odometry &wh_msg)
{
    double t = wh_msg.header.stamp.toSec();
    if (init_wh)
    {
        latest_wh_time = t;
        init_wh = 0;
        return;
    }
    double dt = t - latest_wh_time;
    latest_wh_time = t;

    double vx = wh_msg.twist.twist.linear.x;
    Eigen::Vector3d linear_vel{vx, 0, 0};

    double rz = wh_msg.twist.twist.angular.z;
    Eigen::Vector3d angular_velocity{0, 0, rz};

    Eigen::Vector3d un_vel_0 = tmp_Q * vel_0;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity);
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_vel_1 = tmp_Q * linear_vel;

    Eigen::Vector3d un_acc = 0.5 * (un_vel_0 + un_vel_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    vel_0    = linear_vel;
    wh_gyr_0 = angular_velocity;

}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}


std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

std::vector<std::tuple<std::vector<sensor_msgs::ImuConstPtr>, std::vector<nav_msgs::Odometry>, sensor_msgs::PointCloudConstPtr>>
getAllMeasurements()
{
    std::vector<std::tuple<std::vector<sensor_msgs::ImuConstPtr>, 
                std::vector<nav_msgs::Odometry>,
                sensor_msgs::PointCloudConstPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        
        //if(wh_odom_buf.back().header.stamp.toSec() > imu_buf.back()->header.stamp.toSec())
        {
        //   ROS_WARN("Throwing wheel odom msg as it is ahead of the imu msg");
        //   wh_odom_buf.pop();     
        }
        
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();
        
        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        std::vector<nav_msgs::Odometry> wh_odoms;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();

            if(!wh_odom_buf.empty())
            {
                if(wh_odom_buf.front().header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
                {
                    wh_odoms.emplace_back(wh_odom_buf.front());
                    wh_odom_buf.pop();   
                }
            }
        }
        IMUs.emplace_back(imu_buf.front());
        if(!wh_odom_buf.empty())
            wh_odoms.emplace_back(wh_odom_buf.front());

        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, wh_odoms, img_msg);
    }
    return measurements;
}


void set_wh_odom(nav_msgs::Odometry wh_odom)
{
  wh_odom_g = wh_odom;
}

void get_wh_odom(nav_msgs::Odometry& wh_odom)
{
  wh_odom = wh_odom_g;
}

void wh_odom_callback(const nav_msgs::Odometry &wh_odom_msg)
{
  set_wh_odom(wh_odom_msg);
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder! %f",imu_msg->header.stamp.toSec() );
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    //getting the wh odom data
    nav_msgs::Odometry wh_odom_msg;
    get_wh_odom(wh_odom_msg);
    double t_wh = wh_odom_msg.header.stamp.toSec();

    bool predict_with_wh = false;
    m_buf.lock();
    imu_buf.push(imu_msg);
    if(t_wh >= last_imu_t - 0.02 && t_wh <= last_imu_t + 0.02 && fabs(wh_odom_msg.twist.twist.linear.x) < 1)
    {   
        ROS_WARN("Adding Odom with IMU");
        wh_odom_buf.push(wh_odom_msg);
        predict_with_wh = true;
    }
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        //predict imu (no residual error)
        if(predict_with_wh)
            predict_with_wheel(wh_odom_msg);
        else
            predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{
    //printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

// thread: visual-inertial odometry
void process()
{
    while (true)
    {
        std::vector<std::tuple<std::vector<sensor_msgs::ImuConstPtr>, std::vector<nav_msgs::Odometry>,
                    sensor_msgs::PointCloudConstPtr> > measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getAllMeasurements()).size() != 0;
                 });
        lk.unlock();
        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = std::get<2>(measurement);
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            double vx = 0, wh_rz = 0;
            auto imu_msg = std::get<0>(measurement);
            auto wh_msg  = std::get<1>(measurement);
            for (size_t i = 0; i < imu_msg.size(); ++i)
            {
                double t = imu_msg[i]->header.stamp.toSec();
                double img_t = img_msg->header.stamp.toSec() + estimator.td;

                bool wh_odom_avail = false;
                double wh_t;
                if(i < wh_msg.size())
                {
                    wh_t = wh_msg[i].header.stamp.toSec() + estimator.td;
                    wh_odom_avail = true;
                }

                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg[i]->linear_acceleration.x;
                    dy = imu_msg[i]->linear_acceleration.y;
                    dz = imu_msg[i]->linear_acceleration.z;
                    rx = imu_msg[i]->angular_velocity.x;
                    ry = imu_msg[i]->angular_velocity.y;
                    rz = imu_msg[i]->angular_velocity.z;
                    if(!wh_odom_avail)
                    {
                        estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));   
                    }
                    else
                    {  
                      double dt_wh; 
                      if(i == 0){
                        dt_wh = wh_t - current_time;
                        current_time_wh = wh_t;
                      }
                      else {
                          dt_wh = wh_t - current_time_wh;
                      }
                      
                      vx = wh_msg[i].twist.twist.linear.x; wh_rz = wh_msg[i].twist.twist.angular.z;
                      estimator.processIMUWhOdom(dt, dt_wh, Vector3d(vx,0,0), Vector3d(0,0,wh_rz), Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));                    
                    }       
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;                       
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg[i]->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg[i]->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg[i]->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg[i]->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg[i]->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg[i]->angular_velocity.z;                 
                    
                    if(wh_odom_avail && wh_t > img_t)
                    {

                        double dt_wh_1 = img_t - current_time;
                        double dt_wh_2 = wh_t - img_t;
                        ROS_ASSERT(dt_wh_1 >= 0);
                        ROS_ASSERT(dt_wh_2 >= 0);
                        ROS_ASSERT(dt_wh_1 + dt_wh_2 > 0);

                        double w_wh_1 = dt_wh_2 / (dt_wh_1 + dt_wh_2);
                        double w_wh_2 = dt_wh_1 / (dt_wh_1 + dt_wh_2);

                        vx    = w_wh_1 * vx + w_wh_2 * wh_msg[i].twist.twist.linear.x;
                        wh_rz = w_wh_1 * wh_rz + w_wh_2 * wh_msg[i].twist.twist.angular.z;

                        estimator.processIMUWhOdom(dt_1, dt_wh_1, Vector3d(vx,0,0), Vector3d(0,0,wh_rz), Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));                                                                      
                    }
                    else {
                         estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                         //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                    }
                    current_time = img_t;
                }
            }
            // set relocalization frame
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> image;

            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {

                int v = img_msg->channels[0].values[i] + 0.5;

                int feature_id = v / NUM_OF_CAM;
 
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                double depth = img_msg->channels[5].values[i] / 1000.0;

                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
                xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
            }

            estimator.processImage(image, img_msg->header);

            double whole_t = t_s.toc();

            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";
            // utility/visualization.cpp
            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_msg != NULL)
                pubRelocalization(estimator);
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");
    registerPub(n);
    
    USE_WH_ODOM = 1;

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_wh_odom = n.subscribe("/velocity_controller/odom", 2000, wh_odom_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    //topic from pose_graph, notify if there's relocalization
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

    std::thread measurement_process{process};
    ros::spin();

    return 0;
}