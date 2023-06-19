#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <csignal>

#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

int maxloops = 1000;
int rosloops = 0;

Eigen::VectorXd hdp_vision(4);


Eigen::VectorXd pos(4);


Eigen::VectorXd vel(4);


Eigen::VectorXd axes(6);


Eigen::VectorXd hdp(4);

Eigen::VectorXd vc(4);

ros::Publisher pub_obj;

std::vector<double> quaternion_to_euler(double qw, double qx, double qy, double qz)
{
    // Calcula los ángulos de Euler en la convención ZYX
    // yaw (Z), pitch (Y), roll (X)
    
    // Normaliza el cuaternio
    double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    qw /= norm;
    qx /= norm;
    qy /= norm;
    qz /= norm;
    
    // Calcula los ángulos de Euler
    double yaw = std::atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
    double pitch = std::asin(2*(qw*qy - qx*qz));
    double roll = std::atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy));
    
    // Devuelve los ángulos de Euler en radianes
    return {roll, pitch, yaw};
}

void odo_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double x_real = msg->pose.pose.position.x;
    double y_real = msg->pose.pose.position.y;
    double z_real = msg->pose.pose.position.z;
    double qx_real = msg->pose.pose.orientation.x;
    double qy_real = msg->pose.pose.orientation.y;
    double qz_real = msg->pose.pose.orientation.z;
    double qw_real = msg->pose.pose.orientation.w;

    double vx_real = msg->twist.twist.linear.x;
    double vy_real = msg->twist.twist.linear.y;
    double vz_real = msg->twist.twist.linear.z;
    double wx_real = msg->twist.twist.angular.x;
    double wy_real = msg->twist.twist.angular.y;
    double wz_real = msg->twist.twist.angular.z;

    //Eigen::Quaterniond quaternion(qw_real, qx_real, qy_real, qz_real);
    //Eigen::VectorXd euler= quaternion.toRotationMatrix().eulerAngles(0,1,2);
    
    std::vector<double> euler_angles = quaternion_to_euler(qw_real, qx_real, qy_real, qz_real);
    
    pos << x_real, y_real, z_real, euler_angles[2];
    vel << vx_real, vy_real, vz_real, wz_real;
}

void visual_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    double vx_visual = msg->linear.x;
    double vy_visual = msg->linear.y;
    double vz_visual = msg->linear.z;
    double wx_visual = msg->angular.x;
    double wy_visual = msg->angular.y;
    double wz_visual = msg->angular.z;

    hdp_vision << vx_visual, vy_visual, vz_visual, wz_visual;
}

void rc_callback(const sensor_msgs::Joy::ConstPtr& data)
{
    // Extraer los datos individuales del mensaje
    Eigen::VectorXd axes_aux(6);
    axes_aux << data->axes[0], data->axes[1], data->axes[2], data->axes[3], data->axes[4], data->axes[5];
    
    double psi = -M_PI / 2;

    Eigen::Matrix<double, 6, 6> R;
    R << cos(psi), -sin(psi), 0, 0, 0, 0,
         sin(psi), cos(psi), 0, 0, 0, 0,
         0, 0, -1, 0, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    axes = R * axes_aux;
}


void sendvalues(const Eigen::VectorXd& vc)
{
    geometry_msgs::Twist msg;
    msg.linear.x = vc(0);
    msg.linear.y = vc(1);
    msg.linear.z = vc(2);
    msg.angular.z = vc(3);
    pub_obj.publish(msg);
}


void signalHandler(int signum)
{
    ROS_INFO("\nInterrupt signal received. Shutting down...");
    Eigen::VectorXd vfinal(4); 
    vfinal << 0,0,0,0;

    sendvalues(vfinal);
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "S_UAV_simple_node");
    ros::NodeHandle nh;

    ros::Subscriber odo_sub = nh.subscribe("/dji_sdk/odometry", 10, odo_callback);
    ros::Subscriber visual_sub = nh.subscribe("/dji_sdk/visual_servoing/vel/drone", 10, visual_callback);
    ros::Subscriber rc_sub = nh.subscribe("/dji_sdk/rc", 10, rc_callback);
  

    pub_obj = nh.advertise<geometry_msgs::Twist>("/m100/velocityControl", 10);

    

     std::cout << "OK, controller is running!!!!" << std::endl;

    double hz = 30;  // Frecuencia de actualización
    double ts = 1 / hz;

    // Inicialización de matrices
    hdp << 0,0,0,0;
    axes << 0,0,0,0,0,0;
    vel << 0,0,0,0;
    pos << 0,0,0,0;
    hdp_vision << 0,0,0,0;

    Eigen::MatrixXd J(4, 4);
    J.setZero();

    Eigen::VectorXd Gains(16);
    Gains << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    Gains *= 0.5;

    Eigen::MatrixXd K1(4, 4);
    K1.diagonal() << Gains[0], Gains[1], Gains[2], Gains[3];

    Eigen::MatrixXd K2(4, 4);
    K2.diagonal() << Gains[4], Gains[5], Gains[6], Gains[7];

    Eigen::MatrixXd K3(4, 4);
    K3.diagonal() << Gains[8], Gains[9], Gains[10], Gains[11];

    Eigen::MatrixXd K4(4, 4);
    K4.diagonal() << Gains[12], Gains[13], Gains[14], Gains[15];

    double a = 0;
    double b = 0;

    // Inicialización de vc_anterior
    Eigen::VectorXd vc(4);
    vc.setZero();
    Eigen::VectorXd vc_anterior = vc;

    double ros_rate = 30;  // Tasa de ROS en Hz
    ros::Rate rate(ros_rate);

    // Registrar el manejador de señal para SIGINT
    signal(SIGINT, signalHandler);

    while (ros::ok())
    {
        // Verificar si hay eventos pendientes y llamar a los callbacks
        ros::spinOnce();

        double condicion = axes(5);
        

        if (condicion == -4545.0)
        {
            
            hdp << axes(0), axes(1), axes(3), axes(2);
            std::cout << "\rKinematic: " << std::fixed << std::setprecision(2) << hdp.transpose() << std::flush;
        }
        else if (condicion == -10000.0)
        {
            
            hdp << hdp_vision(0), hdp_vision(1), axes(3) / 4, hdp_vision(3);
            std::cout << "\rServo-visual: " << std::fixed << std::setprecision(2) << hdp.transpose()  << std::flush;
        }
        else
        {   
            hdp << 0.0, 0, 0.0, 0;
            std::cout << "\rManual: " << std::fixed << std::setprecision(2) << hdp.transpose()  << std::flush;
        }

        Eigen::VectorXd h(4); 
        Eigen::VectorXd v(4);
        h << pos(0),pos(1),pos(2),pos(3);
        v << vel(0),vel(1),vel(2),vel(3);


        double psi = h(3);

        J(0, 0) = cos(psi);
        J(0, 1) = -sin(psi);
        J(1, 0) = sin(psi);
        J(1, 1) = cos(psi);
        J(2, 2) = 1;
        J(3, 3) = 1;

        vc = hdp;

        Eigen::VectorXd vcp = (vc - vc_anterior) / ts;

        Eigen::VectorXd x(18);
        x << 0.3259, 0, 0.3787, 0, 0.4144, 0, 0, 0.2295, 0.7623, 0, 0.8279, 0, 0.8437, 0, 0, 0, 0, 1.0390;
        double w = v[3];

        // INERTIAL MATRIX
        double M11 = x[0];
        double M12 = 0;
        double M13 = 0;
        double M14 = a * w * x[1];
        double M21 = 0;
        double M22 = x[2];
        double M23 = 0;
        double M24 = b * w * x[3];
        double M31 = 0;
        double M32 = 0;
        double M33 = x[4];
        double M34 = 0;
        double M41 = a * w * x[5];
        double M42 = b * w * x[6];
        double M43 = 0;
        double M44 = x[7];

        Eigen::MatrixXd M(4, 4);
        M << M11, M12, M13, M14,
             M21, M22, M23, M24,
             M31, M32, M33, M34,
             M41, M42, M43, M44;

        // CENTRIOLIS MATRIX
        double C11 = x[8];
        double C12 = 0;
        double C13 = 0;
        double C14 = a * w * x[9];
        double C21 = 0;
        double C22 = x[10];
        double C23 = 0;
        double C24 = b * w * x[11];
        double C31 = 0;
        double C32 = 0;
        double C33 = x[12];
        double C34 = 0;
        double C41 = b * (w * w) * x[13];
        double C42 = a * (w * w) * x[14];
        double C43 = 0;
        double C44 = (a * a) * (w * w) * x[15] + (b * b) * (w * w) * x[16] + x[17];

        Eigen::MatrixXd C(4, 4);
        C << C11, C12, C13, C14,
             C21, C22, C23, C24,
             C31, C32, C33, C34,
             C41, C42, C43, C44;

        // GRAVITATIONAL MATRIX
        double G11 = 0;
        double G21 = 0;
        double G31 = 0;
        double G41 = 0;

        Eigen::VectorXd G(4);
        G << G11, G21, G31, G41;

        Eigen::VectorXd ve = vc - v;
        
        // std::cout << v << std::endl;
        // //Eigen::VectorXd control = vcp + K3 * tanh(K3.inverse() * K4 * ve);
        Eigen::VectorXd control = vcp + K3 * ((K3.inverse() * K4 * ve).array().tanh().matrix());

        //Eigen::VectorXd control = vcp;
        Eigen::VectorXd vref;
        vref = M * control + C * vc + G;
        // 
        // vref << 0,0,0,0;

        sendvalues(vref);
        
        vc_anterior = vc;
        // Esperar para cumplir con la frecuencia de publicación deseada
        rate.sleep();
         
    }


    return 0;
}