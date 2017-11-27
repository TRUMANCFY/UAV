#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <cmath>
//EIgen SVD libnary, may help you solve SVD
//JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

using namespace cv;
using namespace aruco;
using namespace Eigen;

//global varialbles for aruco detector
aruco::CameraParameters CamParam;
MarkerDetector MDetector;
vector<Marker> Markers;
float MarkerSize = 0.20 / 1.5 * 1.524;
float MarkerWithMargin = MarkerSize * 1.2;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;
Board TheBoardDetected;
ros::Publisher pub_odom_yourwork;
ros::Publisher pub_odom_ref;
cv::Mat K, D;

// test function, can be used to verify your estimation
void calculateReprojectionError(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const cv::Mat R, const cv::Mat t)
{
    puts("calculateReprojectionError begins");
    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
    for (unsigned int i = 0; i < pts_3.size(); i++)
    {
        cv::Mat p_mat(3, 1, CV_64FC1);
        p_mat.at<double>(0, 0) = pts_3[i].x;
        p_mat.at<double>(1, 0) = pts_3[i].y;
        p_mat.at<double>(2, 0) = pts_3[i].z;
        cv::Mat p = (R * p_mat + t);
        printf("(%f, %f, %f) -> (%f, %f) and (%f, %f)\n",
               pts_3[i].x, pts_3[i].y, pts_3[i].z,
               un_pts_2[i].x, un_pts_2[i].y,
               p.at<double>(0) / p.at<double>(2), p.at<double>(1) / p.at<double>(2));
    }
    puts("calculateReprojectionError ends");
}

// the main function you need to work with
// pts_id: id of each point
// pts_3: 3D position (x, y, z) in world frame
// pts_2: 2D position (u, v) in image frame
void process(const vector<int> &pts_id, const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const ros::Time& frame_time)
{
    //version 1, as reference
    cv::Mat r, rvec, t;
    cv::solvePnP(pts_3, pts_2, K, D, rvec, t);
    cv::Rodrigues(rvec, r);
    Matrix3d R_ref;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
            R_ref(i,j) = r.at<double>(i, j);
        }
    Quaterniond Q_ref;
    Q_ref = R_ref;
    nav_msgs::Odometry odom_ref;
    odom_ref.header.stamp = frame_time;
    odom_ref.header.frame_id = "world";
    odom_ref.pose.pose.position.x = t.at<double>(0, 0);
    odom_ref.pose.pose.position.y = t.at<double>(1, 0);
    odom_ref.pose.pose.position.z = t.at<double>(2, 0);
    odom_ref.pose.pose.orientation.w = Q_ref.w();
    odom_ref.pose.pose.orientation.x = Q_ref.x();
    odom_ref.pose.pose.orientation.y = Q_ref.y();
    odom_ref.pose.pose.orientation.z = Q_ref.z();
    pub_odom_ref.publish(odom_ref);

    //std::cout<<1<<"\n\n\n\n";
    // version 2, your work
/*    int n_points=pts_3.size();
    Matrix3d R;
    Vector3d T;
    R.setIdentity();
    T.setZero();
    //T0.setZero();
    //R0.setZero();
    ROS_INFO("write your code here!");
    //std::cout<<D<<"\n\n\n";
    //...
    //...
    //std::cout<<2<<"\n\n\n\n";
    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
    
    Matrix<double,Dynamic,Dynamic> A;
    A.setZero(2*n_points,9);
    for(int i=0; i<n_points; i++){
        VectorXd temp1(9);
        temp1<<(pts_3[i].x),(pts_3[i].y),1,0,0,0,((-1)*(pts_3[i].x)*(un_pts_2[i].x)),((-1)*(pts_3[i].y)*(un_pts_2[i].x)),((-1)*(un_pts_2[i].x));
        A.row(2*i)=temp1;
        VectorXd temp2(9);
        temp2<<0,0,0,(pts_3[i].x),(pts_3[i].y),1,((-1)*(pts_3[i].x)*(un_pts_2[i].y)),((-1)*(pts_3[i].y)*(un_pts_2[i].y)),((-1)*(un_pts_2[i].y));
        A.row(2*i+1)=temp2;
    }
    //std::cout<<3<<"\n\n\n\n";
    JacobiSVD<MatrixXd> svda(A, ComputeThinU | ComputeThinV);
    //std::cout<<31<<"\n\n\n\n";
    Matrix<double,Dynamic,Dynamic> V=svda.matrixV();
    VectorXd h=V.rightCols(1);
    Matrix3d H,k;
    H<<h(0),h(1),h(2),h(3),h(4),h(5),h(6),h(7),h(8);
    //std::cout<<4<<"\n\n\n\n";
    k<<K.at<double>(0,0),K.at<double>(0,1),K.at<double>(0,2),K.at<double>(1,0),K.at<double>(1,1),K.at<double>(1,2),K.at<double>(2,0),K.at<double>(2,1),K.at<double>(2,2);
    Matrix<double,3,3> H1;
    //H=k.inverse()*H;
    if(H(2,2)<0)
	H=-H;
    H1=H;
    H1.col(2)=(H1.col(0)).cross(H1.col(1));
    //std::cout<<5<<"\n\n\n\n";
    JacobiSVD<MatrixXd> svdh(H1, ComputeThinU | ComputeThinV);
    R=(svdh.matrixU())*((svdh.matrixV()).transpose());
    T=H.col(2)/((H1.col(0)).norm());
    //std::cout<<T0<<"\n\n\n\n";

    Matrix<double,6,6> a;
    VectorXd b(6), the(6);
    double phi, theta, psi;
    phi=asin(R(1,2));
    psi=atan2(-R(1,0)/cos(phi),R(1,1)/cos(phi));
    theta=atan2(-R(0,2)/cos(phi),R(2,2)/cos(phi));
    Matrix<double, 2, 3> d;
    d<<D.at<double>(0,0),D.at<double>(0,1),D.at<double>(0,2),D.at<double>(1,0),D.at<double>(1,1),D.at<double>(1,2);
    
    
    //std::cout<<7<<"\n\n\n\n";
    do{
        a.setZero();
        b.setZero();
    
    the<<phi,theta,psi,T(0),T(1),T(2);
	//std::cout<<the<<"\n\n\n\n";
    for(int i=0; i<n_points; i++){
    
    Matrix<double,2,6> Ji;
        Vector3d po;
        
    
        Matrix<double,3,1> pt3;
        Matrix<double,2,1> pt2;
        pt3<<(pts_3[i].x),(pts_3[i].y),(pts_3[i].z);
        pt2<<(un_pts_2[i].x),(un_pts_2[i].y);
        double x=pt3(0);
        double y=pt3(1);
        double z=pt3(2);
        
        po=(R*pt3+T);
        
        double d1=1/po(2);
        double d2=0;
        double d3=0;
        double d4=0;
        double d5=1/po(2);
        double d6=0;
        
        double k1=1;
        double k2=0;
        double k3=0;
        double k4=0;
        double k5=1;
        double k6=0;
        double k7=0;
        double k8=0;
        double k9=1;
        
        
        
        Ji<<- d1*(x*(k2*sin(phi)*sin(psi) + k3*cos(phi)*cos(theta)*sin(psi) - k1*cos(phi)*sin(psi)*sin(theta)) - y*(k2*cos(psi)*sin(phi) + k3*cos(phi)*cos(psi)*cos(theta) - k1*cos(phi)*cos(psi)*sin(theta)) + z*(k2*cos(phi) - k3*cos(theta)*sin(phi) + k1*sin(phi)*sin(theta))) - d2*(x*(k5*sin(phi)*sin(psi) + k6*cos(phi)*cos(theta)*sin(psi) - k4*cos(phi)*sin(psi)*sin(theta)) - y*(k5*cos(psi)*sin(phi) + k6*cos(phi)*cos(psi)*cos(theta) - k4*cos(phi)*cos(psi)*sin(theta)) + z*(k5*cos(phi) - k6*cos(theta)*sin(phi) + k4*sin(phi)*sin(theta))) - d3*(x*(k8*sin(phi)*sin(psi) + k9*cos(phi)*cos(theta)*sin(psi) - k7*cos(phi)*sin(psi)*sin(theta)) - y*(k8*cos(psi)*sin(phi) + k9*cos(phi)*cos(psi)*cos(theta) - k7*cos(phi)*cos(psi)*sin(theta)) + z*(k8*cos(phi) - k9*cos(theta)*sin(phi) + k7*sin(phi)*sin(theta))), d1*(z*(k1*cos(phi)*cos(theta) + k3*cos(phi)*sin(theta)) + x*(k1*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k3*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))) + y*(k1*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - k3*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)))) + d2*(z*(k4*cos(phi)*cos(theta) + k6*cos(phi)*sin(theta)) + x*(k4*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k6*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))) + y*(k4*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - k6*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)))) + d3*(z*(k7*cos(phi)*cos(theta) + k9*cos(phi)*sin(theta)) + x*(k7*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k9*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))) + y*(k7*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - k9*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)))), d1*(x*(k1*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + k3*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + k2*cos(phi)*cos(psi)) - y*(k1*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + k3*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k2*cos(phi)*sin(psi))) + d2*(x*(k4*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + k6*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + k5*cos(phi)*cos(psi)) - y*(k4*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + k6*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k5*cos(phi)*sin(psi))) + d3*(x*(k7*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + k9*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + k8*cos(phi)*cos(psi)) - y*(k7*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + k9*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k8*cos(phi)*sin(psi))), - d1*(k1*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + k3*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k2*cos(phi)*sin(psi)) - d2*(k4*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + k6*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k5*cos(phi)*sin(psi)) - d3*(k7*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + k9*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k8*cos(phi)*sin(psi)), - d1*(k1*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + k3*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + k2*cos(phi)*cos(psi)) - d2*(k4*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + k6*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + k5*cos(phi)*cos(psi)) - d3*(k7*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + k9*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + k8*cos(phi)*cos(psi)), - d1*(k2*sin(phi) + k3*cos(phi)*cos(theta) - k1*cos(phi)*sin(theta)) - d2*(k5*sin(phi) + k6*cos(phi)*cos(theta) - k4*cos(phi)*sin(theta)) - d3*(k8*sin(phi) + k9*cos(phi)*cos(theta) - k7*cos(phi)*sin(theta)),
        - d4*(x*(k2*sin(phi)*sin(psi) + k3*cos(phi)*cos(theta)*sin(psi) - k1*cos(phi)*sin(psi)*sin(theta)) - y*(k2*cos(psi)*sin(phi) + k3*cos(phi)*cos(psi)*cos(theta) - k1*cos(phi)*cos(psi)*sin(theta)) + z*(k2*cos(phi) - k3*cos(theta)*sin(phi) + k1*sin(phi)*sin(theta))) - d5*(x*(k5*sin(phi)*sin(psi) + k6*cos(phi)*cos(theta)*sin(psi) - k4*cos(phi)*sin(psi)*sin(theta)) - y*(k5*cos(psi)*sin(phi) + k6*cos(phi)*cos(psi)*cos(theta) - k4*cos(phi)*cos(psi)*sin(theta)) + z*(k5*cos(phi) - k6*cos(theta)*sin(phi) + k4*sin(phi)*sin(theta))) - d6*(x*(k8*sin(phi)*sin(psi) + k9*cos(phi)*cos(theta)*sin(psi) - k7*cos(phi)*sin(psi)*sin(theta)) - y*(k8*cos(psi)*sin(phi) + k9*cos(phi)*cos(psi)*cos(theta) - k7*cos(phi)*cos(psi)*sin(theta)) + z*(k8*cos(phi) - k9*cos(theta)*sin(phi) + k7*sin(phi)*sin(theta))), d4*(z*(k1*cos(phi)*cos(theta) + k3*cos(phi)*sin(theta)) + x*(k1*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k3*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))) + y*(k1*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - k3*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)))) + d5*(z*(k4*cos(phi)*cos(theta) + k6*cos(phi)*sin(theta)) + x*(k4*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k6*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))) + y*(k4*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - k6*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)))) + d6*(z*(k7*cos(phi)*cos(theta) + k9*cos(phi)*sin(theta)) + x*(k7*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k9*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))) + y*(k7*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - k9*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)))), d4*(x*(k1*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + k3*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + k2*cos(phi)*cos(psi)) - y*(k1*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + k3*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k2*cos(phi)*sin(psi))) + d5*(x*(k4*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + k6*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + k5*cos(phi)*cos(psi)) - y*(k4*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + k6*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k5*cos(phi)*sin(psi))) + d6*(x*(k7*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + k9*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + k8*cos(phi)*cos(psi)) - y*(k7*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + k9*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k8*cos(phi)*sin(psi))), - d4*(k1*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + k3*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k2*cos(phi)*sin(psi)) - d5*(k4*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + k6*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k5*cos(phi)*sin(psi)) - d6*(k7*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + k9*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - k8*cos(phi)*sin(psi)), - d4*(k1*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + k3*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + k2*cos(phi)*cos(psi)) - d5*(k4*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + k6*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + k5*cos(phi)*cos(psi)) - d6*(k7*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + k9*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + k8*cos(phi)*cos(psi)), - d4*(k2*sin(phi) + k3*cos(phi)*cos(theta) - k1*cos(phi)*sin(theta)) - d5*(k5*sin(phi) + k6*cos(phi)*cos(theta) - k4*cos(phi)*sin(theta)) - d6*(k8*sin(phi) + k9*cos(phi)*cos(theta) - k7*cos(phi)*sin(theta));
        
    
        //std::cout<<9<<"\n\n\n\n";
    
    
    
    
	//std::cout<<Ji<<"\n\n\n\n\n\n\n\n";
        MatrixXd pi(2,3);
        pi<<d1,d2,d3,d4,d5,d6;
    a=a+Ji.transpose()*Ji;
    b=b-(Ji.transpose())*(pt2-pi*(R*pt3+T));
    }
    if(((a.inverse())*b).norm()>0.1) break;
        
    the=the+(a.inverse())*b;
    phi=the(0);
    theta=the(1);
    psi=the(2);
    T<<the(3),the(4),the(5);
        //std::cout<<((a.inverse())*b)<<"\n\n\n\n\n";
    //std::cout<<the<<"\n\n\n\n\n\n\n\n\n\n";

    R<<cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta),  cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), -cos(phi)*sin(theta),      -cos(phi)*sin(psi),cos(phi)*cos(psi),             sin(phi),cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),  sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi),  cos(phi)*cos(theta);
        
    }while (((a.inverse())*b).norm()>0.00001);
    //std::cout<<(R_ref-R)<<"\n\n\n\n";
    //...
	//R=R0;
	//T=T0;
    Quaterniond Q_yourwork;
    Q_yourwork = R;
    nav_msgs::Odometry odom_yourwork;
    odom_yourwork.header.stamp = frame_time;
    odom_yourwork.header.frame_id = "world";
    odom_yourwork.pose.pose.position.x = T(0);
    odom_yourwork.pose.pose.position.y = T(1);
    odom_yourwork.pose.pose.position.z = T(2);
    odom_yourwork.pose.pose.orientation.w = Q_yourwork.w();
    odom_yourwork.pose.pose.orientation.x = Q_yourwork.x();
    odom_yourwork.pose.pose.orientation.y = Q_yourwork.y();
    odom_yourwork.pose.pose.orientation.z = Q_yourwork.z();
    pub_odom_yourwork.publish(odom_yourwork);*/
}

cv::Point3f getPositionFromIndex(int idx, int nth)
{
    int idx_x = idx % 18, idx_y = idx / 18;
    double p_x = idx_x * MarkerWithMargin - (3 + 2.5 * 0.2) * MarkerSize;
    double p_y = idx_y * MarkerWithMargin - (12 + 11.5 * 0.2) * MarkerSize;
    return cv::Point3f(p_x + (nth == 1 || nth == 2) * MarkerSize, p_y + (nth == 2 || nth == 3) * MarkerSize, 0.0);
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    double t = clock();
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    MDetector.detect(bridge_ptr->image, Markers);
    float probDetect = TheBoardDetector.detect(Markers, TheBoardConfig, TheBoardDetected, CamParam, MarkerSize);
    ROS_DEBUG("p: %f, time cost: %f\n", probDetect, (clock() - t) / CLOCKS_PER_SEC);

    vector<int> pts_id;
    vector<cv::Point3f> pts_3;
    vector<cv::Point2f> pts_2;
    for (unsigned int i = 0; i < Markers.size(); i++)
    {
        int idx = TheBoardConfig.getIndexOfMarkerId(Markers[i].id);

        char str[100];
        sprintf(str, "%d", idx);
        cv::putText(bridge_ptr->image, str, Markers[i].getCenter(), CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        for (unsigned int j = 0; j < 4; j++)
        {
            sprintf(str, "%d", j);
            cv::putText(bridge_ptr->image, str, Markers[i][j], CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        }

        for (unsigned int j = 0; j < 4; j++)
        {
            pts_id.push_back(Markers[i].id * 4 + j);
            pts_3.push_back(getPositionFromIndex(idx, j));
            pts_2.push_back(Markers[i][j]);
        }
    }

    //begin your function
    if (pts_id.size() > 5)
        process(pts_id, pts_3, pts_2, img_msg->header.stamp);

    //cv::imshow("in", bridge_ptr->image);
    cv::waitKey(10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_detector");
    ros::NodeHandle n("~");

    ros::Subscriber sub_img = n.subscribe("image_raw", 2, img_callback);
    pub_odom_yourwork = n.advertise<nav_msgs::Odometry>("odom_yourwork",10);
    pub_odom_ref = n.advertise<nav_msgs::Odometry>("odom_ref",10);
    //init aruco detector
    string cam_cal, board_config;
    n.getParam("cam_cal_file", cam_cal);
    n.getParam("board_config_file", board_config);
    CamParam.readFromXMLFile(cam_cal);
    TheBoardConfig.readFromFile(board_config);

    //init intrinsic parameters
    cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);
    param_reader["camera_matrix"] >> K;
    param_reader["distortion_coefficients"] >> D;

    //init window for visualization
    //cv::namedWindow("in", 1);

    ros::spin();
}
