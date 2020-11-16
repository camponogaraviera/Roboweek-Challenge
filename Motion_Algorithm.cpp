#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <cfloat>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tf/transform_datatypes.h>

//Variaveis Globais Para Leitura de Dados
sensor_msgs::LaserScan current_laser;
geometry_msgs::Quaternion orientacao;
geometry_msgs::Point posicao;

bool laserPronto = false,
     posePronto = false;

//Funcao Callback do Laser
void lasercallback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
	current_laser = *laser;
    laserPronto = true;

    return;
}

//Funcao Callback da Pose
void posecallback(const nav_msgs::Odometry::ConstPtr& pose)
{

    posicao = pose->pose.pose.position; // Posicao em x y z
    orientacao = pose->pose.pose.orientation; // Orientacao em z
    posePronto = true;

    return;
}

//float median(int n, float x[]) {
//    float temp;
//    int i, j;
//    // the following two loops sort the array x in ascending order
//    for(i=0; i<n-1; i++) {
//        for(j=i+1; j<n; j++) {
//            if(x[j] < x[i]) {
//                // swap elements
//                temp = x[i];
//                x[i] = x[j];
//                x[j] = temp;
//            }
//        }
//    }
//
//    if(n%2==0) {
//        // if there is an even number of elements, return mean of the two elements in the middle
//        return((x[n/2] + x[n/2 - 1]) / 2.0);
//    } else {
//        // else return the element in the middle
//        return x[n/2];
//    }
//}


//int findFarthest(sensor_msgs::LaserScan laser){
//    int i, j, k;
//
//    float curr_values[SPREAD];
//    float m;
//    
//    int best_i = 0;
//    float best_median = 0;
//    for(i = 90; i < N_LASER-90; i++){
//        for(j = -SPREAD/2; j < SPREAD/2; j++)
//            curr_values[j+SPREAD/2] = laser.ranges[i+j];
//
//        m = median(SPREAD, curr_values);
//
//        if (m > best_median){
//            best_median = m;
//            best_i = i;
//        }
//    }
//
//    ROS_INFO("Best Median: %f\n", best_median);
//
//    return best_i;
//}

#define N_LASER 270
#define SPREAD 30
#define DIV_V 1.0/SPREAD
int findFarthest(sensor_msgs::LaserScan laser){
    int i, j, k;
    
    int best_i = 0;
    double best_avg = 0.0;
    double avg = 0.0;
    for(i = 60; i < N_LASER-60; i++){
        avg = 0.0;

        for(j = -SPREAD/2; j < SPREAD/2; j++)
            avg = avg + laser.ranges[i+j];

        if (avg > best_avg){
            best_avg = avg;
            best_i = i;
        }
    }

    return best_i;
}

int findSmallestAngleInRegion(sensor_msgs::LaserScan laser, int min_a, int max_a){
    int i;

    float cur_val = 0.0;
    float smallest = laser.ranges[min_a];
    int best_i = min_a;
    for(i = min_a+1; i < max_a; i++){
        cur_val = laser.ranges[i];
        if (cur_val < smallest){
            smallest = cur_val;
            best_i = i;
        }
    }

    return best_i;
}

int findDirection(sensor_msgs::LaserScan laser, int best_i){
    int i;
    float left_avg;
    float right_avg;

    float val = 0;
    for(i = best_i-SPREAD/2; i < best_i; i++)
        val = val + laser.ranges[i];

    left_avg = val / SPREAD/2;

    val = 0;
    for(i = best_i; i < best_i + SPREAD/2; i++)
        val = val + laser.ranges[i];

    right_avg = val / SPREAD/2;

    if (right_avg > left_avg)
        return 1;
    else
        return -1;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "resgate");  // Inicializacao do Nodo

    ros::NodeHandle n; 		     

    // Configuracao do topico a ser publicado
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10); 

    // Configuracao dos topicos a serem lidos
    ros::Subscriber sub = n.subscribe("scan", 10, lasercallback);
    ros::Subscriber sub1 = n.subscribe("ground_truth", 10, posecallback);

    // Define a frequencia do loop
    ros::Rate loop_rate(20);

    // Declaracoes
    double goalx=-4.1, goaly=3.6; // Posição aproximada da pessoa
    double v1=0.0, v2=0.0;  // Velocidades v1=linear v2=angular
    double oriRad, oriGraus; // Orientacao em radianos e em graus

    // Aguarda a primeira mensagem de cada sensor para evitar erros
    while (ros::ok() && !laserPronto && !posePronto)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    //current_laser.angle_increment = 0.2;

    //current_laser.angle_increment = .105108;
    float ang_min = current_laser.angle_min;
    float ang_max = current_laser.angle_max;
    float ang_inc = current_laser.angle_increment;
    //float n_lasers = (std::abs(ang_min) + std::abs(ang_max)) / ang_inc;

    float front, left_1, right_1, left_2, right_2, left_3, right_3;
    float i1_dist, i2_dist;

    float turn_speed = 0.75;
    float linear_speed = 1.0;
    float dist = 1.0;

    float n_laser;

    int farthest_i;
    int dir;

    int closest_i1, closest_i2;

    long int ticks = 0;
    //Loop Principal. Executa enquanto o roscore estiver 'ok'
    while(ros::ok()) 
    {
        ROS_INFO("Ticks Run: %ld\n", ticks);
        // Calcula orientacao tanto em graus quanto radianos
        oriRad = tf::getYaw(orientacao);
        oriGraus = oriRad * 180.0 / M_PI;

        //ROS_INFO("Front: %f\n", current_laser.ranges[269]);
        //ROS_INFO("Left: %f\n", current_laser.ranges[270]);
        //ROS_INFO("Right: %f\n", current_laser.ranges[50]);

        left_1 = current_laser.ranges[180];
        left_2 = current_laser.ranges[225];
        left_3 = current_laser.ranges[157];
        right_1 = current_laser.ranges[90];
        right_2 = current_laser.ranges[45];
        right_3 = current_laser.ranges[112];

        closest_i1 = findSmallestAngleInRegion(current_laser, 100, 135);
        closest_i2 = findSmallestAngleInRegion(current_laser, 135, 165);
        i1_dist = current_laser.ranges[closest_i1];
        i2_dist = current_laser.ranges[closest_i2];

        farthest_i = findFarthest(current_laser);

        ROS_INFO("Closest to the right: %d - Value: %f\n", closest_i1, i1_dist);
        ROS_INFO("Closest to the left: %d - Value: %f\n", closest_i2, i2_dist);

        v1 = linear_speed;

        if (farthest_i > 134)
            v2 = turn_speed;
        else if (farthest_i < 134)
            v2 = -turn_speed;

        if (i1_dist < 1.0 || i2_dist < 1.0){
            if (i1_dist > i2_dist){
                v2 = -turn_speed * (1.0 / i2_dist);
                v1 = linear_speed * i2_dist;
            }
            else if (i2_dist > i1_dist){
                v2 = +turn_speed * (1.0 / i1_dist);
                v1 = linear_speed * i1_dist;
            }
        }

        // Envia Sinal de Velocidade
        geometry_msgs::Twist vel;

        //v1 = 0.1;

        vel.linear.x=v1;
        vel.angular.z=v2;

        vel_pub.publish(vel);

        ros::spinOnce();
        loop_rate.sleep();
        ticks++;
    }
    return 0;
}
