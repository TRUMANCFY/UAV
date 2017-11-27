#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);
MatrixXd A = MatrixXd::Zero(15,15);
MatrixXd B = MatrixXd::Zero(15,6);
MatrixXd U = MatrixXd::Zero(15,12);
MatrixXd X = MatrixXd::Zero(15,1);
MatrixXd dX = MatrixXd::Zero(15,1);
MatrixXd C = MatrixXd::Identity(6,15);
MatrixXd W = MatrixXd::Identity(6,6);
MatrixXd Z = MatrixXd::Zero(6,1);
MatrixXd covar = MatrixXd::Identity(15,15);
double imu_time=-1.0;
double odo_time=-1.0;
double pi = acos(-1);

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //your code for propagation
    
    if (imu_time<0) return;
    double time=msg->header.stamp.toSec();
    double dt=time-imu_time;
    double p=msg->angular_velocity.x;
    double q=msg->angular_velocity.y;
    double r=msg->angular_velocity.z;
    double ax=msg->linear_acceleration.x;
    double ay=msg->linear_acceleration.y;
    double az=msg->linear_acceleration.z;
    double phi=X(3);
    double theta=X(4);
    double psi=X(5);
    double vx=X(6);
    double vy=X(7);
    double vz=X(8);
    double bg1=X(9);
    double bg2=X(10);
    double bg3=X(11);
    double ba1=X(12);
    double ba2=X(13);
    double ba3=X(14);
    double g=9.81;
    
    dX<<vx,
    vy,
    vz,
    - (cos(theta)*(bg1 + 0 - p)) - (sin(theta)*(bg3 + 0 - r)),
    q - 0 - bg2 + (cos(theta)*sin(phi)*(bg3 + 0 - r))/cos(phi) - (sin(phi)*sin(theta)*(bg1 + 0 - p))/cos(phi),
    (sin(theta)*(bg1 + 0 - p))/cos(phi) - (cos(theta)*(bg3 + 0 - r))/cos(phi),
    cos(phi)*sin(psi)*(ba2 - ay + 0) - (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(ba3 - az + 0) - (cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))*(ba1 - ax + 0),
    - (cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta))*(ba1 - ax + 0) - (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(ba3 - az + 0) - cos(phi)*cos(psi)*(ba2 - ay + 0),
    g - sin(phi)*(ba2 - ay + 0) + cos(phi)*sin(theta)*(ba1 - ax + 0) - cos(phi)*cos(theta)*(ba3 - az + 0),
 0,0,0,0,0,0;
 
 
 A<<
 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
     0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                         (sin(theta)*(bg1 + 0 - p)) - (cos(theta)*(bg3 + 0 - r)), 0, 0, 0, 0,                              -cos(theta),  0,                             -sin(theta), 0, 0, 0,
     0, 0, 0, (cos(phi)*cos(theta)*(bg3 + 0 - r))/(cos(phi)) - (cos(phi)*sin(theta)*(bg1 + 0 - p))/(cos(phi)) + (cos(theta)*sin(phi)*(sin(phi))*(bg3 + 0 - r))/(cos(phi))/cos(phi) - (sin(phi)*sin(theta)*(sin(phi))*(bg1 + 0 - p))/(cos(phi))/cos(phi), - (cos(theta)*sin(phi)*(bg1 + 0 - p))/(cos(phi)) - (sin(phi)*sin(theta)*(bg3 + 0 - r))/(cos(phi)), 0, 0, 0, 0, -(sin(phi)*sin(theta))/(cos(phi)), -1, (cos(theta)*sin(phi))/(cos(phi)), 0, 0, 0,
     0, 0, 0,                                                                                                                                                                                                   (sin(theta)*(sin(phi))*(bg1 + 0 - p))/(cos(phi))/cos(phi) - (cos(theta)*(sin(phi))*(bg3 + 0 - r))/(cos(phi))/cos(phi),                     (cos(theta)*(bg1 + 0 - p))/(cos(phi)) + (sin(theta)*(bg3 + 0 - r))/(cos(phi)), 0, 0, 0, 0,             sin(theta)/(cos(phi)),  0,           -cos(theta)/(cos(phi)), 0, 0, 0,
    
     0, 0, 0, cos(phi)*sin(psi)*sin(theta)*(ba1 - ax + 0) - cos(phi)*cos(theta)*sin(psi)*(ba3 - az + 0) - sin(phi)*sin(psi)*(ba2 - ay + 0), (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(ba1 - ax + 0) - (cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))*(ba3 - az + 0), (cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta))*(ba1 - ax + 0) + (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(ba3 - az + 0) + cos(phi)*cos(psi)*(ba2 - ay + 0), 0, 0, 0, 0, 0, 0,   sin(phi)*sin(psi)*sin(theta) - cos(psi)*cos(theta),  cos(phi)*sin(psi), - cos(psi)*sin(theta) - cos(theta)*sin(phi)*sin(psi),
     0, 0, 0, cos(psi)*sin(phi)*(ba2 - ay + 0) + cos(phi)*cos(psi)*cos(theta)*(ba3 - az + 0) - cos(phi)*cos(psi)*sin(theta)*(ba1 - ax + 0), (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*(ba1 - ax + 0) - (cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta))*(ba3 - az + 0), cos(phi)*sin(psi)*(ba2 - ay + 0) - (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*(ba3 - az + 0) - (cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta))*(ba1 - ax + 0), 0, 0, 0, 0, 0, 0, - cos(theta)*sin(psi) - cos(psi)*sin(phi)*sin(theta), -cos(phi)*cos(psi),   cos(psi)*cos(theta)*sin(phi) - sin(psi)*sin(theta),
     0, 0, 0,                            cos(theta)*sin(phi)*(ba3 - az + 0) - cos(phi)*(ba2 - ay + 0) - sin(phi)*sin(theta)*(ba1 - ax + 0),                                                                   cos(phi)*sin(theta)*(ba3 - az + 0) + cos(phi)*cos(theta)*(ba1 - ax + 0),                                                                                                                                                                                  0, 0, 0, 0, 0, 0, 0,                                  cos(phi)*sin(theta),          -sin(phi),                                 -cos(phi)*cos(theta),
    
 
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
 
 
U<<     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        -cos(theta),  0, -sin(theta), 0, 0, 0, 0, 0, 0, 0, 0, 0,
        -(sin(phi)*sin(theta))/(cos(phi)), -1, (cos(theta)*sin(phi))/(cos(phi)), 0, 0, 0, 0, 0, 0, 0, 0, 0,
        sin(theta)/(cos(phi)),  0,  -cos(theta)/(cos(phi)), 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0,   sin(phi)*sin(psi)*sin(theta) - cos(psi)*cos(theta),  cos(phi)*sin(psi), - cos(psi)*sin(theta) - cos(theta)*sin(phi)*sin(psi), 0, 0, 0, 0, 0, 0,
        0, 0, 0, - cos(theta)*sin(psi) - cos(psi)*sin(phi)*sin(theta), -cos(phi)*cos(psi),   cos(psi)*cos(theta)*sin(phi) - sin(psi)*sin(theta), 0, 0, 0, 0, 0, 0,
        0, 0, 0,                                  cos(phi)*sin(theta),          -sin(phi),                                 -cos(phi)*cos(theta), 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    
    
    
    MatrixXd F;
    MatrixXd V;
    Matrix3d Rot;
    X=X+dt*dX;
    F=MatrixXd::Identity(15,15)+dt*A;
    V=dt*U;
    covar=F*covar*(F.transpose())+V*Q*(V.transpose());
    
    imu_time=time;
    
    Matrix3d wRt;
    wRt<<0,1,0,1,0,0,0,0,-1;
    Vector3d p_pub=wRt*(X.block(0,0,3,1));
    Vector3d v_pub=wRt*(X.block(6,0,3,1));
    
    nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "world";
    odom.pose.pose.position.x=p_pub(0);
    odom.pose.pose.position.y=p_pub(1);
    odom.pose.pose.position.z=p_pub(2);
    
    Rot<<cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),
    cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi),
    -cos(phi)*sin(theta),           sin(phi),                                cos(phi)*cos(theta);
    Quaterniond m=Quaterniond(wRt*Rot);
    odom.pose.pose.orientation.w=m.w();
    odom.pose.pose.orientation.x=m.x();
    odom.pose.pose.orientation.y=m.y();
    odom.pose.pose.orientation.z=m.z();
    odom.twist.twist.linear.x=v_pub(0);
    odom.twist.twist.linear.y=v_pub(1);
    odom.twist.twist.linear.z=v_pub(2);
    odom_pub.publish(odom);
}


//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //your code for update
    //camera position in the IMU frame = (0, -0.04, -0.02)
    //camera orientaion in the IMU frame = Quaternion(0, 0, 1, 0); w x y z, respectively
    //					   RotationMatrix << -1, 0, 0,
    //							      0, 1, 0,
    //                                                        0, 0, -1;
    //Rcam*Z + (0 -0.04 -0.02) =IMU frame
    
    MatrixXd R;
    MatrixXd K;
    
    
    
    
    MatrixXd offset=MatrixXd::Zero(3,1);
    offset<<0,-0.04,-0.02;
    odo_time=msg->header.stamp.toSec();
    
    double x=msg->pose.pose.position.x;
    double y=msg->pose.pose.position.y;
    double z=msg->pose.pose.position.z;
    double qw=msg->pose.pose.orientation.w;
    double qx=msg->pose.pose.orientation.x;
    double qy=msg->pose.pose.orientation.y;
    double qz=msg->pose.pose.orientation.z;
    Quaterniond m=Quaterniond(qw,qx,qy,qz);
    R=m.toRotationMatrix();
    
    double phi_imu=X(3,0);
    double theta_imu=X(4,0);
    double psi_imu=X(5,0);
    
    Z<<x,y,z,0,0,0;
    
    
    Z.block(0,0,3,1)=R.transpose()*(Rcam.transpose()*(-offset)-Z.block(0,0,3,1));
    Matrix3d R0=((Rcam*R).transpose());

    double phi=asin(R0(2,1));
    double psi = atan2(-R0(0,1)/cos(phi),R0(1,1)/cos(phi));
    double theta = atan2(-R0(2,0)/cos(phi),R0(2,2)/cos(phi));
    Z(3,0)=phi;
    Z(4,0)=theta;
    Z(5,0)=psi;

    if (imu_time==-1.0) {
        imu_time=odo_time;
        X.block(0,0,6,1)=Z;
    }
    
    if (phi_imu>phi && phi_imu-phi>2*pi-phi_imu+phi) Z(3,0)=phi+2*pi;
    if (phi_imu<phi && phi-phi_imu > 2*pi-phi+phi_imu) Z(3,0)=Z(3,0)-2*pi;
    if (theta_imu>theta && theta_imu-theta>2*pi-theta_imu+theta) Z(4,0)=theta+2*pi;
    if (theta_imu<theta && theta-theta_imu > 2*pi-theta+theta_imu) Z(4,0)=Z(4,0)-2*pi;
    if (psi_imu>psi && psi_imu-psi>2*pi-psi_imu+psi) Z(5,0)=psi+2*pi;
    if (psi_imu<psi && psi-psi_imu > 2*pi-psi+psi_imu) Z(5,0)=Z(5,0)-2*pi;
    
    K=covar*(C.transpose())*((C*covar*(C.transpose())+W*Rt*(W.transpose())).inverse());
    X=X+K*(Z-(X.block(0,0,6,1)));
    covar=covar-K*C*covar;
    
   
    
}

int main(int argc, char **argv)
{
    //double var_na=0.01; double var_nba=0.01; double var_ng=0.01; double var_nbg=0.01;
    //double var_p=0.5; double var_q=0.5;
    //Q.block(0,0,3,3)=Matrix3d::Identity()*var_na;
    //Q.block(3,3,3,3)=Matrix3d::Identity()*var_ng;
    //Q.block(6,6,3,3)=Matrix3d::Identity()*var_nba;
    //Q.block(9,9,3,3)=Matrix3d::Identity()*var_nbg;
    //Rt.block(0,0,3,3)=Matrix3d::Identity()*var_p;
    //Rt.block(3,3,3,3)=Matrix3d::Identity()*var_q;
    //Rt(5,5)=0.05;
    
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    Rcam = Quaterniond(0, 0, -1, 0).toRotationMatrix();
    cout << "R_cam" << endl << Rcam << endl;
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6);
    Rt.topLeftCorner(3, 3) = 0.05 * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = 0.05 * Rt.bottomRightCorner(3, 3);
    //Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);

    ros::spin();
}
