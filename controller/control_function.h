#ifndef control_funtion_h
#define control_funtion_h

#include <math.h>

/*please fill this controller function
 * input:
 * des_pos -> desired position
 * des_vel -> desired velocity
 * des_acc -> desired acceleration
 * des_yaw -> desired yaw angle
 * now_pos -> now psition
 * now_vel -> body velocity
 * Kp      -> P gain for position loop
 * Kd      -> P gain for velocity loop
 * Mass    -> quality of the quadrotor
 *
 
 * output:
 * rpy               ->    target attitude for autopilot
 * target_thrust     ->    target thrust of the quadrotor
 * */
void SO3Control_function( const double des_pos[3],
                          const double des_vel[3],
                          const double des_acc[3],
                          const double des_yaw,
                          const double now_pos[3],
                          const double now_vel[3],
                          const double now_yaw,
                          const double Kp[3],
                          const double Kd[3],
                          const double Mass,
                          const double Gravity,
                          double rpy[3],
                          double &target_thrust
                        )
{
   /* double des_yaw_temp=0;
    if(des_yaw>now_yaw && des_yaw-now_yaw>2*M_PI-des_yaw+now_yaw)
        des_yaw_temp=des_yaw-2*M_PI;
    if(des_yaw<now_yaw && now_yaw-des_yaw>2*M_PI-now_yaw+des_yaw)
        des_yaw_temp=des_yaw+2*M_PI;
    */
    target_thrust=Mass*(Gravity+Kd[2]*(des_vel[2]-now_vel[2])+Kp[2]*(des_pos[2]-now_pos[2]));
    rpy[0]=((Kd[0]*(des_vel[0]-now_vel[0])+Kp[0]*(des_pos[0]-now_pos[0]))*sin(now_yaw)-(Kd[1]*(des_vel[1]-now_vel[1])+Kp[1]*(des_pos[1]-now_pos[1]))*cos(now_yaw))/Gravity;
    rpy[1]=((Kd[0]*(des_vel[0]-now_vel[0])+Kp[0]*(des_pos[0]-now_pos[0]))*cos(now_yaw)+(Kd[1]*(des_vel[1]-now_vel[1])+Kp[1]*(des_pos[1]-now_pos[1]))*sin(now_yaw))/Gravity;
    rpy[2]=des_yaw;
    
    std::cout<<"az "<<Kd[2]*(des_vel[2]-now_vel[2])+Kp[2]*(des_pos[2]-now_pos[2])<<"\n ax "<<(Kd[0]*(des_vel[0]-now_vel[0])+Kp[0]*(des_pos[0]-now_pos[0]))<<"\n ay "<<(Kd[1]*(des_vel[1]-now_vel[1])+Kp[1]*(des_pos[1]-now_pos[1]))<<'\n';
}
#endif
