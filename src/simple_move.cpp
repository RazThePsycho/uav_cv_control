#include <cmath>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <hector_uav_msgs/Altimeter.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>

#include <numeric>

#include "pid.hpp"

#define RATE 500

Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

static const std::string OPENCV_WINDOW = "Camera from Quadrator";
static const std::string OPENCV_WINDOW2 = "Camera from Quadrator2";

static int low_H = 0, low_S = 0, low_V = 60;
static int high_H = 180, high_S = 25, high_V = 120;

static int lowr_H = 0, lowr_S = 170, lowr_V = 60;
static int highr_H = 15, highr_S = 255, highr_V = 120;

static int lowb_H = 100, lowb_S = 100, lowb_V = 40;
static int highb_H = 126, highb_S = 255, highb_V = 120;

inline bool compare(cv::Vec3f a, cv::Vec3f b)
{
    return (a(2) < b(2));
}

inline int sign(double x)
{
    return (x > 0) - (x < 0);
}

static float dim = 2;



class SimpleMover {

    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber image_sub;
    ros::Subscriber image_sub2;
    ros::Subscriber sonar_sub;
    ros::Subscriber position_sub;
    ros::ServiceClient motor_client;
    ros::Rate rate = ros::Rate(RATE);

    // Altitude variables
    double _sonar_height;
    bool _sonar_flag = false;
    double _3m_height;

    Eigen::Vector3d _point_position;
    double _yaw_vel;

    // Odometry containers
    Eigen::Vector3d _position;
    Eigen::Quaterniond _orientation;
    Eigen::Vector3d _linearspeed;
    Eigen::Vector3d _angularspeed;

    Eigen::Vector3d _line_diff;

    // TOR coordinate and flag
    Eigen::Vector3d _blue_tor {0,0,0};
    Eigen::Vector3d _red_tor {0,0,0};
    bool _blue_flag = false;
    bool _red_flag = false;

    int STAGE = 0;

    cv_bridge::CvImagePtr cv_ptr;

    // Intrinsic matrix for camera
    Eigen::Matrix3d _K = (Eigen::Matrix3d() << 159.99941228826285, 0.0, 160.5, 0.0, 159.99941228826285, 120.5, 0.0, 0.0, 1.0).finished(); 
    const int _height = 240;
    const int _width = 320;
    // Transform to cameras
    Eigen::Vector3d base_to_cam1_tr {0,0,-0.1};
    Eigen::Vector3d base_to_cam2_tr {0,0,-0.2};
    Eigen::Quaterniond base_to_cam1_q { 0, 0.707, -0.707, 0.000};
    Eigen::Quaterniond base_to_cam2_q { 0.500, -0.500, 0.500, -0.500};
    
    // PID coefficient
    double px, py, pz, pxi, pyi, pzi, pxd, pyd, pzd, pyaw;

  public:
    
    SimpleMover() {
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        image_sub = nh.subscribe("/cam_1/camera/image", 1, &SimpleMover::camera_cb, this);
        image_sub2 = nh.subscribe("/cam_2/camera/image", 1, &SimpleMover::camera_cb2, this);
        position_sub = nh.subscribe("/ground_truth/state", 1, &SimpleMover::odom_cb, this);
        sonar_sub = nh.subscribe("/sonar_height", 1, &SimpleMover::sonar_cb, this);
        motor_client = nh.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");

        getROSParamDouble("/profi2022_bachelor_solution/px", &px);
        getROSParamDouble("/profi2022_bachelor_solution/py", &py);
        getROSParamDouble("/profi2022_bachelor_solution/pz", &pz);

        getROSParamDouble("/profi2022_bachelor_solution/pxi", &pxi);
        getROSParamDouble("/profi2022_bachelor_solution/pyi", &pyi);
        getROSParamDouble("/profi2022_bachelor_solution/pzi", &pzi);

        getROSParamDouble("/profi2022_bachelor_solution/pxd", &pxd);
        getROSParamDouble("/profi2022_bachelor_solution/pyd", &pyd);
        getROSParamDouble("/profi2022_bachelor_solution/pzd", &pzd);

        getROSParamDouble("/profi2022_bachelor_solution/pyaw", &pyaw);

        cv::namedWindow(OPENCV_WINDOW);
        cv::namedWindow(OPENCV_WINDOW2);

        ros::Duration(3).sleep();       // требуется для инициализации времени
    }                                   // при слишком быстром старте узла

    // Function for configure PID by .launch file
    void getROSParamDouble(std::string paramname, double *a)
    {
        if (!nh.getParam(paramname, *a)) {
                ROS_ERROR_STREAM("Failed to get param '" << paramname << "'");
            }
    }

    ~SimpleMover() {
        cv::destroyWindow(OPENCV_WINDOW);
        cv::destroyWindow(OPENCV_WINDOW2);
    }

    // Camera1 subscriber function and detect line
    void camera_cb(const sensor_msgs::Image::ConstPtr &msg) {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat img = cv_ptr->image;
        cv::Mat hsv;
        cv::Mat hsv_thresh;
        cv::Mat hsv_B;
        cv::Mat hsv_R;
        cv::Mat bw;

        cv::cvtColor(img, hsv, CV_BGR2HSV);

        cv::inRange(hsv, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), hsv_thresh);
        cv::inRange(hsv, cv::Scalar(lowr_H, lowr_S, lowr_V), cv::Scalar(highr_H, highr_S, highr_V), hsv_R);
        cv::inRange(hsv, cv::Scalar(lowb_H, lowb_S, lowb_V), cv::Scalar(highb_H, highb_S, highb_V), hsv_B);

        Eigen::Vector3d vec(0, 0,  -_sonar_height);
        Eigen::Vector3d XYZ_base = _orientation.toRotationMatrix().transpose()*vec;
        Eigen::Vector3d XYZ_cam = base_to_cam1_q.toRotationMatrix()*(XYZ_base + base_to_cam1_tr);

        double Zaxis = _sonar_height/_orientation.toRotationMatrix()(2,2);
        
        Eigen::Vector3d uv = _K*XYZ_cam;
        double u = uv(0)/uv(2), v = uv(1)/uv(2);

        int window_w = 80, window_h = 30, Y = v - window_h/2, X = u - window_w/2;
  
        if (X > 0 & X < _width - window_w){
            cv::Rect redr = cv::Rect(X, _height - 1.5*window_h, window_w, window_h/2);
            cv::Mat window_B = hsv_B(redr);
            cv::Mat window_R = hsv_R(redr);

            std::vector<cv::Point> nonZeroPixelsB;
            cv::findNonZero(window_B, nonZeroPixelsB);

            std::vector<cv::Point> nonZeroPixelsR;
            cv::findNonZero(window_R, nonZeroPixelsR);
            
            if(nonZeroPixelsR.size() > 400){
                _red_flag = false;
            }
            if(nonZeroPixelsB.size() > 400){
                _blue_flag = false;
            }
        }

        double max_nonZeroPixels = 0;
        int Xi = 0;
        double findX = 0;
        while (Y > window_h/2 & Y < _height - 1.5*window_h & Xi >= 0 & Xi <= _width - window_w){
            cv::Rect r = cv::Rect(Xi,Y,window_w,window_h);
            cv::Mat window = hsv_thresh(r);

            std::vector<cv::Point> nonZeroPixels;
            cv::findNonZero(window, nonZeroPixels);

            double sum_x = 0, sum_y = 0;
            for (const cv::Point &pixel : nonZeroPixels){
                sum_x += pixel.x;
                sum_y += pixel.y;
            }
            double mean_x = sum_x / nonZeroPixels.size() + Xi;
            double mean_y = sum_y / nonZeroPixels.size() + Y;

            if (max_nonZeroPixels < nonZeroPixels.size()){
                max_nonZeroPixels = nonZeroPixels.size();
                findX = mean_x;
            }
            rectangle(img,r,cv::Scalar(0, 0, 128),1,8,0);
            Xi += 20;
        }

        X = findX - window_w/2;
        cv::Point centerCircleW;
        std::vector<Eigen::Vector3d> trajectoryXYZ;
        std::vector<double> meanX;
        int i = 0;
        while (++i < 10 & Y > window_h/2 & Y < _height - 1.5*window_h & X > 0 & X < _width - window_w){
            cv::Rect r = cv::Rect(X,Y,window_w,window_h);
            cv::Mat window = hsv_thresh(r);

            std::vector<cv::Point> nonZeroPixels;
            cv::findNonZero(window, nonZeroPixels);

            double sum_x = 0, sum_y = 0;
            for (const cv::Point &pixel : nonZeroPixels){
                sum_x += pixel.x;
                sum_y += pixel.y;
            }
            double mean_x = sum_x / nonZeroPixels.size() + X;
            double mean_y = sum_y / nonZeroPixels.size() + Y;

            X = mean_x - window_w/2;
            Y = mean_y - window_h/2 - 10;

            if (window_w > 30) window_w -= 10;
            
            centerCircleW = cv::Point(mean_x, mean_y);
            int radiusPoint = 2;

            Eigen::Vector3d XYZ_local;

            XYZ_local[2] = Zaxis;
            XYZ_local[0] = XYZ_local[2]*(mean_x - _K(0,2))/_K(0,0);
            XYZ_local[1] = XYZ_local[2]*(mean_y - _K(1,2))/_K(1,1);

            Eigen::Vector3d XYZ_global = _position + _orientation.toRotationMatrix()*base_to_cam1_q.toRotationMatrix()*XYZ_local;
            trajectoryXYZ.push_back(XYZ_global);
            meanX.push_back(mean_x);

            rectangle(img,r,cv::Scalar(0, 128, 0),1,8,0);
            cv::circle(img, centerCircleW, radiusPoint, cv::Scalar(0, 255, 0), cv::FILLED);
        }      
        
        if (trajectoryXYZ.size() == 0){
            _point_position = _position;
            _line_diff = Eigen::Vector3d::Ones();
            _yaw_vel = 0.1;
        } else {
            for (size_t i = 0; i < trajectoryXYZ.size(); i++) {
                if (pow(trajectoryXYZ[i][0] - _position[0], 2) + pow(trajectoryXYZ[i][1] - _position[1], 2) > 0.2){
                    _point_position = trajectoryXYZ[i];
                    _yaw_vel = meanX[i] - meanX[0];
                    break;
                }
            }
        }

        if (trajectoryXYZ.size() >= 5) {
            _line_diff = trajectoryXYZ[trajectoryXYZ.size() - 1] - trajectoryXYZ[trajectoryXYZ.size() - 5];
        }

        cv::Point centerCircle(_K(0, 2), _K(1, 2));
        cv::Point centerCircleD(u, v);
        
    	cv::Scalar colorCircle(0, 0, 255); // (B, G, R)
        cv::Scalar colorCircleD(0, 255, 0); // (B, G, R)
        int radiusCircle = 5;

	    cv::circle(img, centerCircle, radiusCircle, colorCircle, cv::FILLED);
        cv::circle(img, centerCircleD, radiusCircle, colorCircleD, cv::FILLED);

        show_image(&img);
    }

    // Camera2 subscriber function and detect TOR
    void camera_cb2(const sensor_msgs::Image::ConstPtr &msg) {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat img = cv_ptr->image;
        
        cv::Mat hsv;
        cv::Mat hsv_R;
        cv::Mat hsv_B;
        cv::Mat hsv_thresh;
        cv::Mat bw;

        cv::cvtColor(img, hsv, CV_BGR2HSV);
        
        cv::inRange(hsv, cv::Scalar(lowb_H, lowb_S, lowb_V), cv::Scalar(highb_H, highb_S, highb_V), hsv_B);
        cv::inRange(hsv, cv::Scalar(lowr_H, lowr_S, lowr_V), cv::Scalar(highr_H, highr_S, highr_V), hsv_R);
        
        _red_tor = find_tor(&img, &hsv_R, cv::Scalar(255, 0, 0));
        _blue_tor = find_tor(&img, &hsv_B, cv::Scalar(0, 0, 255));

        show_image2(&img);
    }

    Eigen::Vector3d find_tor(cv::Mat *img, cv::Mat *bw, cv::Scalar COLOR){
        cv::GaussianBlur(*bw, *bw, cv::Size(7, 7), 0, 0);

        int largest_area=0;
        int largest_contour_index=0;
        cv::Rect bounding_rect;
        std::vector<std::vector<cv::Point> > contours; // Vector for storing contours

        findContours(*bw, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image

        for( size_t i = 0; i< contours.size(); i++ ) // iterate through each contour.
        {
            double area = contourArea( contours[i] );  //  Find the area of contour

            if( area > largest_area )
            {
                largest_area = area;
                largest_contour_index = i;               //Store the index of largest contour
                bounding_rect = boundingRect( contours[i] ); // Find the bounding rectangle for biggest contour
            }
        }

        
        cv::Mat count = cv::Mat::zeros(img->size(), CV_8UC3);
        cv::drawContours(count, contours, largest_contour_index, cv::Scalar(255, 255, 255),  1); // Draw the largest contour using previously stored index.
        cv::drawContours(*img, contours, largest_contour_index, cv::Scalar(0, 255, 0), 2 ); // Draw the largest contour using previously stored index.

        cv::Mat countC;
        cv::cvtColor(count, countC, CV_BGR2GRAY);
        cv::GaussianBlur(countC, countC, cv::Size(3, 3), 0, 0);

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(countC, circles, CV_HOUGH_GRADIENT, 1, img->rows/1.5, 100, 20, 20, 230);

        Eigen::Vector3d tor;
        if (circles.size()){
            cv::Vec3f max_circ = *std::max_element(circles.begin(), circles.end(), compare);
            Eigen::Vector3d coordinate;

            coordinate[2] = _K(0, 0)*dim/(2*max_circ[2]);  
            coordinate[0] = ((max_circ[0] - _K(0, 2))*coordinate[2])/_K(0, 0);
            coordinate[1] = ((max_circ[1] - _K(1, 2))*coordinate[2])/_K(1, 1);

            cv::Point center(max_circ[0],max_circ[1]);
            cv::circle(*img, center, max_circ[2], COLOR, 2);

            tor = _position + _orientation.toRotationMatrix()*base_to_cam2_q.toRotationMatrix()*coordinate;
        } else {
            tor = Eigen::Vector3d(0,0,0);
        }
        return tor;
    }
    
    // Odometry subscriber function
    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg) {
        _position[0] = msg->pose.pose.position.x;
        _position[1] = msg->pose.pose.position.y;
        _position[2] = msg->pose.pose.position.z;

        _orientation.w() = msg->pose.pose.orientation.w;
        _orientation.x() = msg->pose.pose.orientation.x;
        _orientation.y() = msg->pose.pose.orientation.y;
        _orientation.z() = msg->pose.pose.orientation.z;

        _linearspeed[0] = msg->twist.twist.linear.x;
        _linearspeed[2] = msg->twist.twist.linear.y; 
        _linearspeed[1] = msg->twist.twist.linear.z; 

        _angularspeed[0] = msg->twist.twist.angular.x;
        _angularspeed[1] = msg->twist.twist.angular.y;
        _angularspeed[2] = msg->twist.twist.angular.z;
    }

    // Sonar subscriber function with adaptability
    void sonar_cb(const sensor_msgs::Range::ConstPtr &msg){
        _sonar_height = msg->range;

        if (_sonar_height >= 3){
            if (!_sonar_flag) {
                _3m_height = _position[2];
                _sonar_flag = true;
            }
            _sonar_height = _position[2] - _3m_height + 3;
        }
        if (_sonar_height < 3) {
            _sonar_flag = false;
        }
    }

    void show_image(const cv::Mat *img) {
        cv::imshow(OPENCV_WINDOW, *img);
        cv::waitKey(3);
    }

    void show_image2(const cv::Mat *img) {
        cv::imshow(OPENCV_WINDOW2, *img);
        cv::waitKey(3);
    }

    void enable_motors() {
        ros::service::waitForService("/enable_motors");
        hector_uav_msgs::EnableMotors srv;
        srv.request.enable = true;
        if (!motor_client.call(srv)) {
            ROS_ERROR("Failed to call service enable_motors");
        }
    }


    void take_off() {

        enable_motors();

        double cur_time = ros::Time::now().toSec();
        double end_time = cur_time + 2;
        geometry_msgs::Twist twist_msg;
        Eigen::Vector3d start = _position;


        while ( nh.ok() && (cur_time < end_time)) {
            twist_msg.linear.x = 0;
            twist_msg.linear.y = 0;
            twist_msg.linear.z = (1.9 - _sonar_height)*pz;
            cmd_vel_pub.publish(twist_msg);
            ros::spinOnce();
            rate.sleep();
            cur_time = ros::Time::now().toSec();
            std::cout << "STAGE " << STAGE << std::endl;
        }
        STAGE += 1;
    }

void spin() {

        take_off();
        
        // PID initialization
        Eigen::VectorXd kp(4); kp << px, py, pz, pyaw;
        Eigen::VectorXd ki(4); ki << pxi, pyi, pzi, 0;
        Eigen::VectorXd kd(4); kd << pxd, pyd, pzd, 0;
        double T = 1.0/RATE;
        PID pid(kp, ki, kd, T);
        Eigen::VectorXd v(4), res(4);
        v << 0,0,0,0;
        double drone_altitude = 2;
        // Important part of PID initialization
        pid.init(v);

        Eigen::Vector3d e_global, e;
        Eigen::Vector3d path_diff;
        Eigen::Vector3d red_coord, blue_coord;

        Eigen::Matrix3d rot_q;
        double start_time = ros::Time::now().toSec();
        double delta_time = 0;
        double red_time;
        double nnow = 0;
        double Y_dist_blue = 0;

        int count_blue = 0;
        while (nh.ok()) {
            geometry_msgs::Twist twist_msg;

            rot_q = _orientation.toRotationMatrix().transpose();

            if (!_red_flag && _red_tor[0] != 0 & _red_tor[1] != 0 & _red_tor[2] != 0){
                if((_position - _red_tor).norm() < 2.5){
                    _red_flag = true;
                    red_coord = _red_tor;
                }
            }

            if (!_blue_flag && _blue_tor[0] != 0 & _blue_tor[1] != 0 & _blue_tor[2] != 0){
                if((_position - _blue_tor).norm() < 2){
                    _blue_flag = true;
                    count_blue += 1;
                    blue_coord = _blue_tor;
                }
            }

            if (_red_flag){
                drone_altitude = red_coord[2] + 1.2;
                red_time = ros::Time::now().toSec();
            }
            else if(sqrt(pow(_position[0] - _red_tor[0],2) + pow(_position[1] - _red_tor[1],2)) < 0.4){
                drone_altitude = red_coord[2] + 1;
            }

            if (!_red_flag & !_blue_flag){
                drone_altitude = 2;
            }

            if (drone_altitude < 1){
                drone_altitude = 1;
            }
            
            e_global[0] = _point_position[0] - _position[0];
            e_global[1] = _point_position[1] - _position[1];
            e_global[2] = drone_altitude - _sonar_height;

            e = rot_q*e_global;

            Y_dist_blue = (rot_q*(blue_coord - _position))[1];
            if (_blue_flag & abs(Y_dist_blue) > dim/2-0.1 & abs(Y_dist_blue) < dim - 0.2){
                if (abs(drone_altitude - blue_coord[2]) > 0.2) drone_altitude = blue_coord[2];
                e[1] += Y_dist_blue - sign(Y_dist_blue)*(dim/2 - 0.1);
            } else if (_blue_flag & abs(Y_dist_blue) > dim - 0.2){
                _blue_flag = false;
            }

            path_diff = rot_q*_line_diff;
            
            if (abs(path_diff[1]) < 0.01 & !_red_flag) {
                    if (pid.kp[0] <= 1.8*kp[0]) {
                        pid.kp[0] += 0.02;
                    }
                } else {
                    if (pid.kp[0] > kp[0]) {
                        pid.kp[0] -= 0.2;
                    }
                }

            v[0] = e[0];
            v[1] = e[1];
            v[2] = e[2];
            v[3] = -_yaw_vel;

            res = pid.dotPI(v);

            twist_msg.linear.x = res[0];
            twist_msg.linear.y = res[1];
            twist_msg.linear.z = res[2];

            twist_msg.angular.z = res[3];

            cmd_vel_pub.publish(twist_msg);

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "simple_mover");

    SimpleMover simpleMover;
    simpleMover.spin();

  return 0;
}
