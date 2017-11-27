#ifndef trajectory_h
#define trajectory_h
#include <stdio.h>
#include <math.h>

using namespace Eigen;

/*
 * this function is to get desired states for specific trajectory, just generated, at time dt.
 * input:
 * dT   -> the time
 * hover_pos -> the desired position where you want quadrotor to hover
 * now_vel -> maybe useless
 *
 * output:
 * desired_pos -> desired position at dT
 * desired_vel -> desired velocity at dT
 * desired_acc -> desired acceleration at dT
 * return:
 * true  -> you have already configured desired states
 * false -> no desired state
 */
bool trajectory_control(const double dT, 
        const Vector3d hover_pos,
        const Vector3d now_vel,
        Vector3d & desired_pos,
        Vector3d & desired_vel,
        Vector3d & desired_acc,
        const Matrix<double,Dynamic,Dynamic> coef,
        const VectorXd T
        )
{
    //if you don't want to use Eigen, then you can use these arrays
    //or you can delete them and use Eigen
    double hover_p[3], now_v[3], desired_p[3], desired_v[3], desired_a[3];
    hover_p[0] = hover_pos.x();
    hover_p[1] = hover_pos.y();
    hover_p[2] = hover_pos.z();
    now_v[0] = now_vel.x();
    now_v[1] = now_vel.y();
    now_v[2] = now_vel.z();
    //your code // please use coefficients from matlab to get desired states
    
    
    for(int i=1; i<T.rows();i++){
        if(dT<T(i)){
            double tt =dT-T(i-1);
            //std::cout<<tt<<'\n';
            //std::cout<<"T(i-1): "<<T(i-1)<<'\n';
            
            Matrix<double, 1, 8> t_p;
            t_p<<pow(tt,7), pow(tt,6), pow(tt,5), pow(tt,4), pow(tt,3), pow(tt,2), tt, 1;
            Matrix<double, 1, 8> t_v;
            t_v<<7*pow(tt,6), 6*pow(tt,5), 5*pow(tt,4), 4*pow(tt,3), 3*pow(tt,2), 2*tt, 1, 0;
            Matrix<double, 1, 8> t_a;
            t_a<<42*pow(tt,5), 30*pow(tt,4), 20*pow(tt,3), 12*pow(tt,2), 6*tt, 2, 0, 0;
            
            Matrix<double,Dynamic,Dynamic> coef1;
            coef1=coef.block(8*i-8,0,8,3);
            
            desired_p[0]=t_p*coef1.col(0);
            //std::cout<<desired_p[0]<<"\n\n\n\n";
            
            desired_p[1]=t_p*coef1.col(1);
            desired_p[2]=t_p*coef1.col(2);
            desired_v[0]=t_v*coef1.col(0);
            desired_v[1]=t_v*coef1.col(1);
            desired_v[2]=t_v*coef1.col(2);
            desired_a[0]=t_a*coef1.col(0);
            desired_a[1]=t_a*coef1.col(1);
            desired_a[2]=t_a*coef1.col(2);
            break;
        }
    }
    if(dT>T(T.rows()-1)) return 0;
    
    
    
    //output
    desired_pos.x() = desired_p[0];
    desired_pos.y() = desired_p[1];
    desired_pos.z() = desired_p[2];
    desired_vel.x() = desired_v[0];
    desired_vel.y() = desired_v[1];
    desired_vel.z() = desired_v[2];
    desired_acc.x() = desired_a[0];
    desired_acc.y() = desired_a[1];
    desired_acc.z() = desired_a[2];

    return true; // if you have got desired states, true.
}
#endif
