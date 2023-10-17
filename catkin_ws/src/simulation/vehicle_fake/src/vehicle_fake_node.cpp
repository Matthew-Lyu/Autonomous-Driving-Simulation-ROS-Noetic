#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <cmath>

//—————————————————全局变量——————————————————————//
double cmd_timeout;  // 命令超时时间
ros::Time last_callback;  // 上次回调时间

double x = 0.;  // x轴位置
double y = 0.;  // y轴位置
double th = 0.;  // 角度
double vx = 0.;  // x轴线速度
double vy = 0.;  // y轴线速度
double vth = 0.;  // 角速度
double xC[] = {x, y, th, vx, vy, vth};  // 状态数组6维

double evx = 0.;  // 期望x轴速度
double evy = 0.;  // 期望y轴速度
double evphi = 0.; // 期望角速度

double min_vx = 0.2; // 最小x轴速度
double max_vx = 0.5; // 最大x轴速度
double min_vy = 0.0; // 最小y轴速度
double max_vy = 0.0; // 最大y轴速度
double left_phi = -0.8; // 向左最大舵角
double right_phi = 0.8; // 向右最大舵角
double min_vphi = -0.8; // 最小角速度
double max_vphi = 0.8; // 最大角速度

//—————————————————双PID控制器————————————————————//
// 定义PID控制器参数
// 油门
double throttle_Kp = 1.0;
double throttle_Ki = 0.0;
double throttle_Kd = 0.0;
// 舵角
double steering_Kp = 0.8;
double steering_Ki = 8.0;
double steering_Kd = 0.0;

// 双PID控制器的输出
double throttle_delta[2] = {0.}; // 油门变化量2维,{油门, 舵角};

// 定义双PID控制器
class PIDController {
private:
  double error; // 误差
  double integral; // 积分
  double derivative; // 微分
  double previous_error; // 上次误差

public:
  double Kp; // 比例系数
  double Ki; // 积分系数
  double Kd; // 微分系数

  // 构造函数
  PIDController(double kp, double ki, double kd) {
    Kp = kp; 
    Ki = ki;
    Kd = kd;
    error = 0.0;
    integral = 0.0;
    derivative = 0.0;
    previous_error = 0.0;
  }
  
  // 计算输出（油门，舵角）
  double calculate(double expectation, double measured, double dt) {
    error = expectation - measured;
    double output;

    integral += error ;
    derivative = (error - previous_error);
    output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
    return output;
  }
};

// 创建PID控制器对象
PIDController throttle_controller(throttle_Kp, throttle_Ki, throttle_Kd);
PIDController steering_controller(steering_Kp, steering_Ki, steering_Kd);


//—————————————————状态更新函数————————————————————//
// 状态更新，输入参数为当前状态xC，油门变化throttle_delta，以及时间间隔dt
void update_xC(double* xC, double* throttle_delta, double dt) {
    
    // 定义车辆参数
    double Cm1 = 0.287;  // 前轮滑移力矩系数
    double Cm2 = 0.0545;  // 前轮滑移力矩系数
    double Cr0 = 0.0218;  // 轮胎滚动阻力系数
    double Cr2 = 0.00035;  // 轮胎滚动阻力系数
    double B_r = 3.3852;  // 后轮侧向力系数
    double C_r = 1.2691;  // 后轮侧向力系数
    double D_r = 0.1737;  // 后轮纵向力系数
    double B_f = 2.579;  // 前轮侧向力系数
    double C_f = 1.2;  // 前轮侧向力系数
    double D_f = 0.192;  // 前轮纵向力系数

    // 定义车辆质量和惯性矩
    double m = 0.041;  // 车辆质量
    double Iz = 27.8e-6;  // 车辆惯性矩
    double l_f = 0.029;  // 前轴到质心的距离
    double l_r = 0.033;  // 后轴到质心的距离
    double g = 9.8;  // 重力加速度

    // 计算前后轴负荷
    double Nf = m * g * l_r / (l_f + l_r);
    double Nr = m * g * l_f / (l_f + l_r);


    // 获取当前状态
    double phi = xC[2];  // 当前车辆的航向角
    double v_x = xC[3];  // 当前车辆的纵向速度
    double v_y = xC[4];  // 当前车辆的横向速度
    double omega = xC[5];  // 当前车辆的角速度
    double D = throttle_delta[0];  // 油门变化量
    double delta = throttle_delta[1];  // 方向盘转角

    // 计算前后轮侧偏角
    double alpha_f = std::atan2((l_f * omega + v_y), std::abs(v_x)) - delta;
    double alpha_r = std::atan2((v_y - l_r * omega), std::abs(v_x));

    // 计算前后轮侧向力和纵向力
    double F_fy = D_f * std::sin(C_f * std::atan(-B_f * alpha_f));
    double F_fx = -Cr0 * Nf - Cr2 * v_x * v_x;
    double F_ry = D_r * std::sin(C_r * std::atan(-B_r * alpha_r));
    double F_rx = (Cm1 * D - Cm2 * D * v_x - Cr0 * Nr - Cr2 * v_x * v_x);

    // 计算dx
    double dx[6] = {0.0};
    dx[0] = v_x * std::cos(phi) - v_y * std::sin(phi);
    dx[1] = v_y * std::cos(phi) + v_x * std::sin(phi); 
    dx[2] = omega;

    dx[3] = 1 / m * (F_rx + F_fx * std::cos(delta) - F_fy * std::sin(delta) + m * v_y * omega);
    dx[4] = 1 / m * (F_ry + F_fx * std::sin(delta) + F_fy * std::cos(delta) - m * v_x * omega); 
    dx[5] = 1 / Iz * (F_fx * std::sin(delta) * l_f + F_fy * l_f * std::cos(delta) - F_ry * l_r);

    //更新xC
    for (int i = 0; i < 6; i++) {xC[i] = xC[i] + dx[i]*dt ;}
    
    // 初始状态下限定位置和速度为0
    if(throttle_delta[0] == 0.0 && throttle_delta[1] == 0.0){xC[0] = 0.0;xC[3] = 0.0;}

    // 限制x轴方向速度
    if (xC[3] < min_vx && xC[3]!=0) {xC[3] = min_vx;}
    if (xC[3] > max_vx) {xC[3] = max_vx;}
    
    // 限制y轴方向速度
    xC[4] = 0.0;
    
    // 限制角速度
    if (xC[5] > max_vphi) {xC[5] = max_vphi;}
    if (xC[5] < min_vphi) {xC[5] = min_vphi;}
}

// 命令回调函数，计算油门和舵角
void cmdCallback(geometry_msgs::Twist::ConstPtr cmd)
{
  static ros::Time last_time = ros::Time::now(); // 上次回调时间
  ros::Time current_time = ros::Time::now(); // 当前时间
  double dt = ( current_time - last_time ).toSec(); // 计算时间间隔

  // 计算油门输出
  throttle_delta[0] = throttle_controller.calculate(cmd->linear.x, xC[3], dt);
  // 计算舵角输出
  throttle_delta[1] = steering_controller.calculate(cmd->angular.z, xC[5], dt);
  
  // 期望速度
  evphi = cmd->angular.z;
  evx = cmd->linear.x;
  evy = cmd->linear.y;

  // 限制油门大小
  if(throttle_delta[0] < -1.){throttle_delta[0] = -1.;}
  if(throttle_delta[0] > 1.){throttle_delta[0] = 1.;}

  // 限制方向盘转角
  if(throttle_delta[1] < left_phi){throttle_delta[1] = left_phi;}
  if(throttle_delta[1] > right_phi){throttle_delta[1] = right_phi;}

  //ROS_INFO("D = %f, delta = %f",throttle_delta[0],throttle_delta[1]);

  last_callback = last_time = current_time; // 更新上次回调时间
}

//————————————主函数——————————————————————//
int main(int argc, char** argv)
{
  // 创建节点
  ros::init(argc, argv, "vehicle_fake");
  
  // 创建节点句柄
  ros::NodeHandle nh("vehicle");

  // 创建发布者
  ros::Publisher odom_pub = nh.advertise< nav_msgs::Odometry >("odom", 10);
  tf::TransformBroadcaster odom_broadcaster;
  ros::Subscriber cmd_sub = nh.subscribe< geometry_msgs::Twist >("cmd_vel", 10, cmdCallback);

  nh.param( "cmd_timeout", cmd_timeout, 5.0 );

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time    = ros::Time::now();

  ros::Rate rate(10.0);
  double rtt = 0.;
  while(nh.ok())
  {
    ros::spinOnce();
    
    if(xC[3]!=0){
        rtt += 0.01;
        ROS_INFO("time is %f",rtt);
    }
    current_time = ros::Time::now();

    if ( (current_time - last_callback).toSec() > cmd_timeout )
    {
      ROS_WARN( "subscriber timeout, set all velocity to Zero" );
      xC[3] = 0; xC[4] = 0; xC[5] = 0;
    }

    double dt  = (current_time - last_time).toSec();
    update_xC(xC, throttle_delta, dt);

    // 设置tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = xC[0];
    odom_trans.transform.translation.y = xC[1];
    odom_trans.transform.translation.z = 0.0;

    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(xC[2]);
  
    odom_trans.transform.rotation      = quat;

    // 发布tf
    odom_broadcaster.sendTransform(odom_trans);

    // 设置odom
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "base_link";

    odom.pose.pose.position.x = xC[0];
    odom.pose.pose.position.y = xC[1];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = quat;

    odom.child_frame_id = "odom";
    odom.twist.twist.linear.x = xC[3];
    odom.twist.twist.linear.y = xC[4];
    odom.twist.twist.angular.z = xC[5];

    // 发布odom
    odom_pub.publish(odom);

    last_time = current_time;
    rate.sleep();
  }
}